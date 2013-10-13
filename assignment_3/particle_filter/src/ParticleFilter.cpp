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
void MyLocaliser::initialisePF( const geometry_msgs::PoseWithCovarianceStamped& initialpose )
{
  double map_pos_x  = map.info.origin.position.x * map.info.resolution;
  double map_pos_y  = map.info.origin.position.y * map.info.resolution;
  double map_width  = map.info.width  * map.info.resolution;
  double map_height = map.info.height * map.info.resolution;

  std::normal_distribution<> dx(map_pos_x + map_width  / 2, map_width  / 4);
  std::normal_distribution<> dy(map_pos_y + map_height / 2, map_height / 4);

  for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
  {
    particleCloud.poses[i].position.x = dx(gen);
    particleCloud.poses[i].position.y = dy(gen);
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(i*0.1);
    particleCloud.poses[i].orientation = odom_quat;
  }

}

/**
 * Your implementation of this should sample from a random
 * distribution instead of blindly adding the odometry increment. It
 * should also update the angle of the particles.
 */
void MyLocaliser::applyMotionModel( double deltaX, double deltaY, double deltaT ) // {{{
{
  if (deltaX > 0 or deltaY > 0)
    ROS_DEBUG( "applying odometry: %f %f %f", deltaX, deltaY, deltaT );

  static const double STDDEV = 0.05;

  for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
  {
    std::normal_distribution<> dx(deltaX, STDDEV);
    std::normal_distribution<> dy(deltaY, STDDEV);
    std::normal_distribution<> dt(deltaT, STDDEV);

    particleCloud.poses[i].position.x += dx(gen);
    particleCloud.poses[i].position.y += dy(gen);
    geometry_msgs::Quaternion odom_quat = particleCloud.poses[i].orientation;
    double yaw = tf::getYaw(odom_quat);
    particleCloud.poses[i].orientation = tf::createQuaternionMsgFromYaw(yaw + dt(gen));
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
  weights = std::vector<double>(particleCloud.poses.size(), 0.0);
  double sum = 0.0;
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
    for (unsigned int k = 0; k < MAX_RAYS; ++k) {
      static const double VAR = 0.1;
      double norm = (1/sqrt(2*M_PI*VAR)) * exp( -pow((scan.ranges[k*s] - simulatedScan->ranges[k*s]), 2) / (2*VAR));
//      ROS_INFO("scan[%i] %f simulated[%i] %f norm %f", k*s, scan.ranges[k*s], k*s, simulatedScan->ranges[k*s], norm);
      weights[i] += norm;
    }
    sum += weights[i];
  }
  ROS_INFO_STREAM("sum " << sum);
  for (unsigned int i = 0; i < particleCloud.poses.size(); ++i) {
    weights[i] /= sum;
    ROS_INFO_STREAM("weights["<<i<<"] " << weights[i]);
  }
} // }}}

/**
 * This is where resampling should go, after applying the motion and
 * sensor models.
 */
geometry_msgs::PoseArray
MyLocaliser::updateParticleCloud ( const sensor_msgs::LaserScan& scan,
                                   const nav_msgs::OccupancyGrid& map,
                                   const geometry_msgs::PoseArray& particleCloud )
{
  for (int i = 1; i < particleCloud.poses.size(); ++i) {
    weights[i] += weights[i-1];
  //  ROS_INFO_STREAM("weights["<<i<<"] " << weights[i]);
  }

  std::uniform_real_distribution<> p(0,1);
  for (int i = 0; i < particleCloud.poses.size(); ++i) {
    std::vector<double>::iterator it = std::upper_bound(weights.begin(), weights.end(), p(gen));
    int j = it - weights.begin();
    this->particleCloud.poses[i] = particleCloud.poses[j];
  }

  return this->particleCloud;
}

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
