/* Autonomous Systems : Bayes filter
// For this assignment you are going to implement a bayes filter

- uniformly distibute the belief over all beliefstates
- implement a representation of the world for every beliefstate
- implement the measurement model
- implement the motion model (forward & turn)
*/

#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "laser_to_wall/WallScan.h"
#include "std_msgs/Int32.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include <boost/lexical_cast.hpp>
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <sstream>


class BayesFilter {


public:
  // Construst a new BayesFilter object and hook up this ROS node
  // to the simulated robot's velocity control and laser topics
  BayesFilter(ros::NodeHandle& nh) : rotateStartTime(ros::Time::now()),rotateDuration(1.8f), moveDuration(0.75f) {
    // Initialize random time generator
    srand(time(NULL));
    // Advertise a new publisher for the simulated robot's velocity command topic
    // (the second argument indicates that if multiple command messages are in
    //  the queue to be sent, only the last command will be sent)
    commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    kidnapPub = nh.advertise<geometry_msgs::Pose2D>("pose", 1);
    // Subscribe to the simulated robot's wall scan topic and tell ROS to call
    // this->commandCallback() whenever a new message is published on that topic
    wallSub = nh.subscribe("wall_scan", 1, &BayesFilter::commandCallbackWallScan, this);
    actionSub = nh.subscribe("action", 1, &BayesFilter::commandCallbackAction, this);
    markerPub = nh.advertise<visualization_msgs::MarkerArray>("beliefs",1);
    movenoise = false;
    measnoise = false;

    /*=TODO - INSERT-CHANGE CODE HERE IF NEEDED=*/
    // Initial belief distribution
    for (int i = 0; i<NUM_STATES; i++) {
        beliefStates.push_back(1.0/NUM_STATES);
    };
   /*============================================*/
   };

  // {{{ publish visual information to RVIZ of the beliefstates
  void publishBeliefMarkers()
  {
     visualization_msgs::MarkerArray beliefs;
     for (int i = 0; i < NUM_STATES; i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time();
        marker.ns = "beliefs";
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        if (i >= 10) {
                marker.pose.position.x = -0.8;
                marker.pose.position.y =  4.5 -i%10;
        }
        else{
                marker.pose.position.x = 0.8;
                marker.pose.position.y = -4.5 +i;
        }
        marker.pose.position.z = 0.2;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0  * beliefStates[i]         ;
        marker.id = i;
        beliefs.markers.push_back(marker);

        //Text
        visualization_msgs::Marker marker2;
        marker2.header.frame_id = "/map";
        marker2.header.stamp = ros::Time();
        marker2.ns = "beliefs";
        marker2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker2.action = visualization_msgs::Marker::ADD;
        if (i >= 10) {
                marker2.pose.position.x = -0.8;
                marker2.pose.position.y =  4.5 -i%10;
        }
        else{
                marker2.pose.position.x = 0.8;
                marker2.pose.position.y = -4.5 +i;
        }
        marker2.pose.position.z = 0.2;
        marker2.pose.orientation.x = 0.0;
        marker2.pose.orientation.y = 0.0;
        marker2.pose.orientation.z = 0.0;
        marker2.pose.orientation.w = 1.0;
        marker2.scale.x = 0.5;
        marker2.scale.y = 1.0;
        marker2.scale.z = 0.15;
        // Set the color -- be sure to set alpha to something non-zero!
        marker2.color.r = 1.0f;
        marker2.color.g = 1.0f;
        marker2.color.b = 1.0f;
        marker2.color.a = 1.0;
        //std::string text = boost::lexical_cast<string>(i);
        std::ostringstream oss;
        oss << i;
        std::ostringstream oss2;
        oss2 << beliefStates[i];
        marker2.text = "State: " + oss.str() + "\nBelief:\n" + oss2.str();
        marker2.id = NUM_STATES + i;
        beliefs.markers.push_back(marker2);
     }
     markerPub.publish(beliefs);
  }; // }}}

  const double p_sense (int x_t) const // {{{ 
  {
      double retval = 1.0;
      switch (x_t) {
              // no doors
          case  9:
          case 19:
              if (wall_left)  retval *= 0.8; else retval *= 0.2;
              if (wall_right) retval *= 0.8; else retval *= 0.2;
              if (wall_front) retval *= 0.7; else retval *= 0.3;
              break;

              // door on the front
          case  0:
          case  2:
          case  4:
          case  5:
          case  6:
          case  8:
          case 10:
          case 11:
          case 13:
          case 14:
          case 15:
          case 17:
              if (wall_left)  retval *= 0.8; else retval *= 0.2;
              if (wall_right) retval *= 0.8; else retval *= 0.2;
              if (wall_front) retval *= 0.3; else retval *= 0.7;
              break;

              // door on the left and front
          case  1:
          case  7:
          case 16:
              if (wall_left)  retval *= 0.2; else retval *= 0.8;
              if (wall_right) retval *= 0.8; else retval *= 0.2;
              if (wall_front) retval *= 0.3; else retval *= 0.7;
              break;

              // door on the right and front
          case  3:
          case 12:
          case 18:
              if (wall_left)  retval *= 0.8; else retval *= 0.2;
              if (wall_right) retval *= 0.2; else retval *= 0.8;
              if (wall_front) retval *= 0.3; else retval *= 0.7;
              break;
      }
      return retval;
  }; // }}}

  const static double p_move (int to, int from) // {{{
  {
      if (from ==  9 && to ==  9) return 1.0;
      if (from == 19 && to == 19) return 1.0;
      if (from ==  8 && to ==  9) return 0.9;
      if (from ==  8 && to ==  8) return 0.1;
      if (from == 18 && to == 19) return 0.9;
      if (from == 18 && to == 18) return 0.1;
      if (from ==  9 && to == 10) return 0.0;
      if (from ==  9 && to == 11) return 0.0;
      if (from ==  8 && to == 10) return 0.0;

      switch (to - from) {
          case 0:  return 0.1;
          case 1:  return 0.8;
          case 2:  return 0.1;
          default: return 0.0;
      }
  }; // }}}

  const static double p_turn (int x_t, int x_t1) // {{{
  {
      if (x_t == x_t1) {
          return 0.1;
      } else if (x_t1 == NUM_STATES-1 - x_t) {
          return 0.9;
      } else {
          return 0.0;
      }
  }; // }}}

  void bayes_filter_sense () // {{{
  {
      double sum = 0.0;

      std::vector<double> new_beliefs(20);

      for (int i = 0; i < NUM_STATES; ++i) {
          new_beliefs[i] = p_sense(i) * beliefStates[i];

          sum += new_beliefs[i];
      }
      for (int i = 0; i < NUM_STATES; ++i) {
          beliefStates[i] = new_beliefs[i] / sum;
      }
  }; // }}}

  void bayes_filter_action (const double (*p)(int,int)) // {{{
  {
      std::vector<double> new_beliefs(20);

      for (int i = 0; i < NUM_STATES; ++i) {
          new_beliefs[i] = 0.0;
          for (int j = 0; j < NUM_STATES; ++j) {
               new_beliefs[i] += (*p)(i, j) * beliefStates[j];
          }
      }
      for (int i = 0; i < NUM_STATES; ++i) {
          beliefStates[i] = new_beliefs[i];
      }
  }; // }}}

  /*=TODO - INSERT-CHANGE CODE HERE IF NEEDED=*/
  void updateMove() {
    bayes_filter_action(&p_move);
  };
  /*==========================================*/

  /*=TODO - INSERT-CHANGE CODE HERE IF NEEDED=*/
  void updateTurn() {
    bayes_filter_action(&p_turn);
  };
  /*==========================================*/

  /*=TODO - INSERT-CHANGE CODE HERE IF NEEDED=*/
  void updateSensing() {
    bayes_filter_sense();
  }
  /*==========================================*/

  // Send a velocity command
  void move(double linearVelMPS, double angularVelRadPS) // {{{
  {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub.publish(msg);
  } // }}}

  // introduce discrete movement noise
  int movementNoise() // {{{ 
  {
        if (movenoise)
        {
         int val = rand()%100;
         if (val<LOWER_NOISE_THRESHOLD) return 0;
         if (val>=UPPER_NOISE_THRESHOLD) return 2;
        }
        return 1;
  } // }}}

  // Introduce measurement noise
  bool measurementNoise(bool measurement) // {{{
  {
        if (measnoise)
        {
          int val = rand()%100;
          if (measurement) {
            if (val>=80)
                return !measurement;
          }
          else
            if (val>=70)
              return !measurement;
        }
        return measurement;
  } // }}}

  // Process the incoming action message
  void commandCallbackAction(const std_msgs::Int32::ConstPtr& msg) { // {{{
        int steps = movementNoise();
        if (msg->data == 0) {
          for (int i = 0; i<steps; i++) {
            if (!obstacle) {
              moveStartTime = ros::Time::now();
              while (ros::Time::now() - moveStartTime <= moveDuration)
                move(FORWARD_SPEED_MPS,0);
            }
            ros::Duration(0.2).sleep();
          }
          updateMove();
        }
        if (msg->data == 1) {
          for (int i = 0; i<std::min(steps,1); i++) {
            rotateStartTime = ros::Time::now();
            while (ros::Time::now() - rotateStartTime <= rotateDuration)
                move(0,ROTATE_SPEED_RADPS);
          }
          updateTurn();
        }
        if (msg->data == 2) {
          updateSensing();
        }
        if (msg->data == 3) {
          if (movenoise==false) movenoise = true;
          else movenoise = false;
          ROS_INFO_STREAM("movementnoise: " << movenoise);
        }
        if (msg->data == 4) {
          if (measnoise==false) measnoise = true;
          else measnoise = false;
          ROS_INFO_STREAM("measurementnoise: " << measnoise);
        }
        if (msg->data == 5) {
          kidnapRobot();
          ROS_INFO_STREAM("A herring sandwich! I LIKE herring sandwiches!");
        }
        publishBeliefMarkers();
  } // }}}

  // Process the incoming wall scan message
  void commandCallbackWallScan(const laser_to_wall::WallScan::ConstPtr& msg) { // {{{
        wall_left  =  measurementNoise((bool)msg->wall_left);
        wall_right =  measurementNoise((bool)msg->wall_right);
        wall_front =  measurementNoise((bool)msg->wall_front);
        obstacle =  (bool)msg->wall_front;
  } // }}}

  void kidnapRobot() // {{{
  {
      geometry_msgs::Pose2D msg;
      static const float MAX_X = + NUM_STATES / 4 - 0.5;
      static const float MIN_X = - NUM_STATES / 4 + 0.5;

      int pos = rand() % (NUM_STATES / 2);
      msg.x = MIN_X + pos;
      msg.y = 0;

      int rot = rand() % 2;
      msg.theta = rot * M_PI;

      kidnapPub.publish(msg);
  } // }}}

protected:
  ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
  ros::Publisher kidnapPub;
  ros::Publisher markerPub;
  ros::Subscriber wallSub; // Subscriber to the simulated robot's wall scan topic
  ros::Subscriber actionSub; // Subscriber to the action topic
  ros::Time rotateStartTime; // Start time of the rotation
  ros::Duration rotateDuration; // Duration of the rotation
  ros::Time moveStartTime; // Start time of the rotation
  ros::Duration moveDuration; // Duration of the rotation
  std::vector<double> beliefStates;
  bool wall_front, wall_left, wall_right, movenoise, measnoise, obstacle;
  const static int NUM_STATES = 20;
  const static double FORWARD_SPEED_MPS = 1.0;
  const static double ROTATE_SPEED_RADPS = M_PI/2;
  const static int UPPER_NOISE_THRESHOLD = 90;
  const static int LOWER_NOISE_THRESHOLD = 10;
  /*=TODO - INSERT-CHANGE CODE HERE IF NEEDED=*/


  /*==========================================*/
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "bayes_filter"); // Initiate new ROS node named "bayes_filter"
  ros::NodeHandle n;
  BayesFilter filter(n); // Create new filter object
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
//  ros::spin();
  return 0;
};
