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
  double map_pos_x  = map.info.origin.position.x * map.info.resolution;
  double map_pos_y  = map.info.origin.position.y * map.info.resolution;
  double map_width  = map.info.width  * map.info.resolution;
  double map_height = map.info.height * map.info.resolution;

  std::normal_distribution<> dx(map_pos_x + map_width  / 2, map_width  / 6);
  std::normal_distribution<> dy(map_pos_y + map_height / 2, map_height / 6);

  for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
  {
    particleCloud.poses[i].position.x = dx(gen);
    particleCloud.poses[i].position.y = dy(gen);
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(i*0.1);
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
  if (deltaX > 0 or deltaY > 0)
    ROS_DEBUG( "applying odometry: %f %f %f", deltaX, deltaY, deltaT );

  double d = hypot(deltaX, deltaY);
  double theta = -atan2(deltaX,deltaY);

  static double d_total = 0.0;
  d_total += d;
  has_moved_enough = false;
  if (d_total > DELTA_D) {
    has_moved_enough = true;
    d_total = 0.0;
  }

  ROS_INFO_STREAM("d " << d);

  for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
  {
    static const double STDDEV = 0.00;
    static std::normal_distribution<> nd(0, STDDEV);

    double d_noisy = d + nd(gen);
    double theta_noisy = theta + nd(gen);
    double yaw = tf::getYaw(particleCloud.poses[i].orientation);
    double x = d_noisy * cos(yaw + theta_noisy + M_PI/2);
    double y = d_noisy * sin(yaw + theta_noisy + M_PI/2);
//    ROS_INFO("yaw(%f) x(%f) y(%f)", yaw, x, y);

    ROS_INFO("theta %f yaw %f d %f", theta, yaw, d);

    particleCloud.poses[i].position.x += x;
    particleCloud.poses[i].position.y += y;
    particleCloud.poses[i].orientation = tf::createQuaternionMsgFromYaw(yaw + deltaT + nd(gen));
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
    try {
      simulatedScan = occupancy_grid_utils::simulateRangeScan(this->map, sensor_pose, scan, true);
    }
    catch (occupancy_grid_utils::PointOutOfBoundsException)
    {
      continue;
    }

    /* Now we have the actual scan, and a simulated version ---
     * i.e., how a scan would look if the robot were at the pose
     * that particle i says it is in. So now we should evaluate how
     * likely this pose is; i.e., the actual sensor model. */
    static const int MAX_RAYS = 30;
    int s = simulatedScan->ranges.size() / MAX_RAYS;
    weights[i] = 1.0;
    for (unsigned int k = 0; k < MAX_RAYS; ++k) {
      double SIGMA = 1;
      double VAR = SIGMA * SIGMA;
      double norm = exp( - pow(scan.ranges[k*s] - simulatedScan->ranges[k*s], 2) / (2*VAR)) / sqrt(2*M_PI*VAR);

      weights[i] *= norm;
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

  for (int i = 0; i < weights.size(); ++i) {
    ROS_INFO("weights[%i] %f", i, weights[i]);
  }
  static constexpr double STDDEV = 0.05;
  static std::normal_distribution<> d(0, STDDEV);

  int idx = rand() % particleCloud.poses.size();
  double beta = 0.0;
  double max_weight = weights.max();
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
geometry_msgs::PoseWithCovariance MyLocaliser::updatePose()
{
  double mean_x{}, mean_y{}, mean_t{};
  for (int i = 0; i < particleCloud.poses.size(); ++i) {
    mean_x += particleCloud.poses[i].position.x;
    mean_y += particleCloud.poses[i].position.y;
    mean_t += tf::getYaw(particleCloud.poses[i].orientation);
  }

  mean_x /= particleCloud.poses.size();
  mean_y /= particleCloud.poses.size();
  mean_t /= particleCloud.poses.size();

  estimatedPose.pose.pose.position.x = mean_x;
  estimatedPose.pose.pose.position.y = mean_y;
  estimatedPose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(mean_t);

  //TODO: also update the covariance
  return this->estimatedPose.pose;
}
