#include "ParticleFilter.hpp"

// Used for simulateRangeScan
#include "occupancy_grid_utils/ray_tracer.h"

/**
 * Just place all particles on a line along x=y. This should be
 * replaced with something more sensible, like drawing particles
 * from a Gaussian distribution with large variance centered on
 * the supplied initial pose, or just placing them in a regular
 * grid across the map.
 */
void MyLocaliser::initialisePF( const geometry_msgs::PoseWithCovarianceStamped& initialpose ) // {{{
{
  double x = initialpose.pose.pose.position.x;
  double y = initialpose.pose.pose.position.y;
  double yaw = tf::getYaw(initialpose.pose.pose.orientation);

  std::normal_distribution<> d(0, 3);

  for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
  {
    particleCloud.poses[i].position.x = x + d(gen);
    particleCloud.poses[i].position.y = y + d(gen);
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw + d(gen));
    particleCloud.poses[i].orientation = odom_quat;
  }

} // }}}

/**
 * Your implementation of this should sample from a random
 * distribution instead of blindly adding the odometry increment. It
 * should also update the angle of the particles.
 */
void MyLocaliser::applyMotionModel( double deltaX, double deltaY, double deltaT ) // {{{
{
  double d = hypot(deltaX, deltaY);

  // don't update if we haven't moved/turned
  static constexpr double eps = 1e-5;
  if (std::abs(d) < eps && std::abs(deltaT) < eps)
    return;

  static double total = 0.0;
  // physicists will hate me
  total += d + std::abs(deltaT) / 2;
  has_moved_enough = false;
  static constexpr double DELTA = 1;
  if (total > DELTA) {
    has_moved_enough = true;
    total = 0.0;
  }

  for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
  {
    static const double STDDEV = 0.05;
    static std::normal_distribution<> nd(0, STDDEV);

    double d_noisy = d + nd(gen);
    double deltaT_noisy = deltaT + nd(gen);

    double yaw = tf::getYaw(particleCloud.poses[i].orientation);

    double x = d_noisy * cos(yaw + deltaT_noisy);
    double y = d_noisy * sin(yaw + deltaT_noisy);

    particleCloud.poses[i].position.x += x;
    particleCloud.poses[i].position.y += y;
    particleCloud.poses[i].orientation = tf::createQuaternionMsgFromYaw(yaw + deltaT_noisy);
  }
} // }}}

/**
 * After the motion model moves the particles around, approximately
 * according to the odometry readings, the sensor model is used to
 * weight each particle according to how likely it is to get the
 * sensor reading that we have from that pose.
 */
void MyLocaliser::applySensorModel( const sensor_msgs::LaserScan& scan ) // {{{
{
  if (!has_moved_enough) {
    return;
  }

  weights = std::valarray<double>(0.0, particleCloud.poses.size());
  /* This method is the beginning of an implementation of a beam
   * sensor model */
  for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
  {
    geometry_msgs::Pose sensor_pose;
    sensor_pose = particleCloud.poses[i];
    /* If the laser and centre of the robot weren't at the same
     * position, we would first apply the tf from /base_footprint
     * to /base_laser here. */
    sensor_msgs::LaserScan::Ptr simulatedScan;

    // trick simulateRangeScan into thinking we only have MAX_RAYS
    // (this is a dirty hack, ogu::simulateRangeScan() should be fixed instead)
    static const int MAX_RAYS = 30;
    sensor_msgs::LaserScan mangled_scan(scan);
    int s = scan.ranges.size();
    const double angle_range = mangled_scan.angle_max - mangled_scan.angle_min;
    mangled_scan.angle_increment = angle_range / (MAX_RAYS - 1);
    mangled_scan.ranges.resize(MAX_RAYS);
    for (int i = 0; i < MAX_RAYS; ++i) {
      mangled_scan.ranges[i] = scan.ranges[i * s/MAX_RAYS];
    }

    try {
      simulatedScan = occupancy_grid_utils::simulateRangeScan(this->map, sensor_pose, mangled_scan, true);
    }
    catch (occupancy_grid_utils::PointOutOfBoundsException)
    {
      continue;
    }

    /* Now we have the actual scan, and a simulated version ---
     * i.e., how a scan would look if the robot were at the pose
     * that particle i says it is in. So now we should evaluate how
     * likely this pose is; i.e., the actual sensor model. */
    weights[i] = 1.0;
    for (unsigned int k = 0; k < mangled_scan.ranges.size(); ++k) {
      double z = simulatedScan->ranges[k];
      double z_scan = mangled_scan.ranges[k];

      // FIXME use sensible values here, I really have no clue
      double z_hit = 0.4, z_short = 0.25, z_max = 0.20, z_rand = 0.15;
      double p_hit{}, p_short{}, p_max{}, p_rand{};
      static constexpr double LAMBDA_SHORT = 0.5;
      static constexpr double SIGMA_HIT = 0.1;

      if (z >= mangled_scan.range_max) {
        p_max = 1;
      }

      if (z >= 0 && z <= z_scan) {
        double N = 1 / (1 - exp(-LAMBDA_SHORT*z_scan));
        p_short = N * LAMBDA_SHORT * exp( -LAMBDA_SHORT*z);
      }

      if (z >= 0 && z <= mangled_scan.range_max) {
        p_rand = 1 / mangled_scan.range_max;
      }

      if (z >= 0 && z <= mangled_scan.range_max) {
        p_hit = exp( - pow(z_scan - z, 2) / (2*SIGMA_HIT*SIGMA_HIT)) / (SIGMA_HIT * sqrt(2*M_PI));
      }

      weights[i] *= (z_hit*p_hit) + (z_short*p_short) + (z_max*p_max) + (z_rand*p_rand);
    }
  }

  // normalize weights
  weights /= weights.sum();
} // }}}

/**
 * This is where resampling should go, after applying the motion and
 * sensor models.
 */
geometry_msgs::PoseArray
MyLocaliser::updateParticleCloud ( const sensor_msgs::LaserScan& scan, // {{{
                                   const nav_msgs::OccupancyGrid& map,
                                   const geometry_msgs::PoseArray& particleCloud )
{
  if (!has_moved_enough) {
    return this->particleCloud;
  }

  static double STDDEV = 0.0;
    STDDEV = 0.1;
  static std::normal_distribution<> d(0, STDDEV);

  int idx = rand() % particleCloud.poses.size();
  double beta = 0.0;
  double max_weight = weights.max();
  ROS_INFO("max_weight %f", max_weight);
  static std::uniform_real_distribution<> p(0,1);
  for (int i = 0; i < particleCloud.poses.size(); ++i) {
    beta += p(gen) * 2.0 * max_weight;
    while (beta > weights[idx]) {
      beta -= weights[idx];
      idx = (idx + 1) % particleCloud.poses.size();
    }
    this->particleCloud.poses[i].position.x = particleCloud.poses[idx].position.x + d(gen);
    this->particleCloud.poses[i].position.y = particleCloud.poses[idx].position.y + d(gen);
    double yaw = tf::getYaw(particleCloud.poses[idx].orientation);
    this->particleCloud.poses[i].orientation = tf::createQuaternionMsgFromYaw(yaw + d(gen));
  }

  return this->particleCloud;
} // }}}

/**
 * Update and return the most likely pose.
 */
geometry_msgs::PoseWithCovariance MyLocaliser::updatePose() // {{{
{
  double mean_x{}, mean_y{}, mean_t{}, mean_tc{}, mean_ts{};
  for (int i = 0; i < particleCloud.poses.size(); ++i) {
    mean_x += particleCloud.poses[i].position.x;
    mean_y += particleCloud.poses[i].position.y;
    mean_tc += cos(tf::getYaw(particleCloud.poses[i].orientation));
    mean_ts += sin(tf::getYaw(particleCloud.poses[i].orientation));
  }

  int s = particleCloud.poses.size();
  mean_x /= s;
  mean_y /= s;
  mean_t = atan2(mean_ts / s, mean_tc / s);

  estimatedPose.pose.pose.position.x = mean_x;
  estimatedPose.pose.pose.position.y = mean_y;
  estimatedPose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(mean_t);

  //TODO: also update the covariance
  return this->estimatedPose.pose;
} // }}}
