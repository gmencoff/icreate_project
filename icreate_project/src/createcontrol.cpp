#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>
#include <vector>
#include <cmath>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>

// this class will calculate the twist message to send to cmd_vel
class Createdriver
{
// define variables privately
private:
  float x_c; //create current location x
  float y_c; // create current location y
  float theta_c; // create current theta
  float x_d; // desired x
  float y_d; // desired y
  float theta_d; // desired turn angle
  float x_r; // x component of r vector
  float y_r; // y component of r vector
  float theta_r; //theta r
  float mag_r; // length of r
  int loccalled;
  static const float lin_vel_max; // stores the maximum allowable linear velocity
  static const float rot_vel_max; // stores the maximum allowable rotational velocity
  static const float theta_reduced_speed; // stores the theta value where rotational velcotiy reduction should occur
  static const float r_error; // stores the allowable distance error from the desired location

// functions are defined publically
public:

  //constructor function initializes values
  Createdriver()
  {
    x_c=0;
    y_c=0;
    theta_c=0;
    x_d=0;
    y_d=0;
    theta_d=0;
    loccalled=0;
  };

  // gets current position
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
    x_c=msg->pose.pose.position.x; // get robot x position
    y_c=msg->pose.pose.position.y; // get robot y position

    //create quaternion
    tf::Quaternion q(msg->pose.pose.orientation.x,
                     msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z,
                     msg->pose.pose.orientation.w);

    //create quaternion matrix and get RPY, where theta_c is yaw
    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    //set theta_c to yaw, make sure it is positive
    theta_c = yaw;
    if(theta_c<0){
      theta_c=2*M_PI+theta_c;
    };

    ROS_INFO_STREAM("roll: " << roll << "pitch: " << pitch <<  "yaw: " << yaw);

    };

  // gets desired position
  void locCallback(const geometry_msgs::Pose2D::ConstPtr& msg){
    x_d=msg->x;
    y_d=msg->y;
    loccalled=1;
  };

  // calculate desired linear velocity and angular velocity, outputs vector [lin,ang]
  std::vector<float> getVel(){
    std::vector<float>  vel(2); vel[0]=0;vel[1]=0;

    // if desured location has been set, update velocity
    if (loccalled==1){
      //if desired location has been set, update velocity

      // get the vector components between the desired and current locations
      x_r=x_d-x_c;
      y_r=y_d-y_c;

      // get the magnitude of r and calculate theta_d
      mag_r=sqrt(x_r*x_r+y_r*y_r);
      theta_r=acos(x_r/mag_r);
      if (y_r<0){
        theta_r=2*M_PI-theta_r;
      };

      // calculatet theta_d
      if(theta_c<=theta_r){
        if((2*M_PI-theta_r+theta_c)<=(theta_r-theta_c)){
          //turn clockwise
          theta_d=-(2*M_PI-theta_r+theta_c);
        }
        else{
          //turn counterclockwise
          theta_d=(theta_r-theta_c);
        };
      }
      else{
        if((2*M_PI-theta_c+theta_r)<=(theta_c-theta_r)){
          //turn counterclockwise
          theta_d=(2*M_PI-theta_c+theta_r);
        }
        else{
          //turn clockwise
          theta_d=-(theta_c-theta_r);
        };

      };

      // set velocity
      if (mag_r<r_error){
      // if length of r < error allowed, then velocity is 0
        vel[0]=0; vel[1]=0;
      }
      else if (abs(theta_d)>=theta_reduced_speed){
      // if the desired theta is greater than the reduced speed theta, set theta to max
        vel[0]=0;

        // determine whether to turn clockwise or counterclockwise
        if(theta_d>=0){
          // turn ccw
          vel[1]=rot_vel_max;
        }
        else{
          //turn clockwise
          vel[1]=-rot_vel_max;
        };

      }
      else if (abs(theta_d)<theta_reduced_speed){
      // if the desired theta is less than the reduced speed theta, set theta according to control rule
        vel[0]=lin_vel_max;
        vel[1]=rot_vel_max/theta_reduced_speed*theta_d;
      };
    ROS_INFO_STREAM("theta_d: " << theta_d << "\n");
    ROS_INFO_STREAM("theta_c: " << theta_c << "\n");
    ROS_INFO_STREAM("theta_r: " << theta_r << "\n");
    //ROS_INFO_STREAM("roll: " << roll << "pitch: " << pitch <<  "yaw: " << yaw);

    }

    // if desired location has not been set, velocity should remain 0
    else{
      return vel;
    };
  };

};

// Define constant driver variables
const float Createdriver::lin_vel_max=.5;
const float Createdriver::rot_vel_max=4.25;
const float Createdriver::theta_reduced_speed=3.14/4;
const float Createdriver::r_error=.2;


//void poseCallback(const turtlesim::Pose::ConstPtr& msg){
//  ROS_INFO("TurtlePose: [%f,%f,%f]",msg->x,msg->y,msg->theta);
//};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "createcontrol"); //initialize createcontrol node
  ros::NodeHandle n; // create a ros node handle

  // Specify that this node will publish a geometry msg to cmd vel
  ros::Publisher cmdvel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  // create a Turtledriver object and subscriber will subscribe in that object
  Createdriver createdriver;
  ros::Subscriber pose_sub = n.subscribe("/odom",1000,&Createdriver::poseCallback,&createdriver);
  ros::Subscriber loc_sub = n.subscribe("/create_desired_loc",1000,&Createdriver::locCallback,&createdriver);

  //ros::Subscriber sub = n.subscribe("/turtle1/pose",1000,poseCallback);

  // 1 Hz publishing rate is the same as the create internal rate
  ros::Rate loop_rate(10);


  int count = 0; // count number of times looped

  while (ros::ok())
  {
    geometry_msgs::Twist vel; //twist object to send to cmd_vel

    ros::spinOnce(); // call the queued callback functions

    std::vector<float> velvec = createdriver.getVel(); // get the calculated velocity based on desired and current location


    //set linear and angular velocity according to the output from getVel
    vel.linear.x = velvec[0];
    vel.angular.z = velvec[1];

    cmdvel_pub.publish(vel); // publish message as specified by cmdvel_pub

    //ROS_INFO_STREAM("xVel: " << vel.linear.x << "\n" << "thetaVel: " << vel.angular.z << "\n");

    loop_rate.sleep(); //execute at frequency specified by loop rate
    ++count;
  }


  return 0;
}
