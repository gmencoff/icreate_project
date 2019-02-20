
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
  int loop_frequency; // stores loop frequency
  float loop_rate; // stores loop rate
  std::vector<float>  vel; // stores velocity
  static const float lin_vel_max; // stores the maximum allowable linear velocity
  static const float rot_vel_max; // stores the maximum allowable rotational velocity
  static const float lin_accel_max; // stores max allowed linear acceleration
  static const float rot_accel_max; // stores max allowed roatational acceleration
  static const float theta_reduced_speed; // stores the theta value where rotational velcotiy reduction should occur
  static const float r_error; // stores the allowable distance error from the desired location

// functions are defined publically
public:

  //constructor function initializes values
  Createdriver() : vel(2) {
    x_c=0;
    y_c=0;
    theta_c=0;
    x_d=0;
    y_d=0;
    theta_d=0;
    loccalled=0;
    loop_frequency=100; //set loop frequency to 100 Hz
    loop_rate=1/(float)loop_frequency;
    vel[0]=0.0;vel[1]=0.0;
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

    //ROS_INFO_STREAM("roll: " << roll << "pitch: " << pitch <<  "yaw: " << yaw);

    };

  // gets desired position
  void locCallback(const geometry_msgs::Pose2D::ConstPtr& msg){
    x_d=msg->x;
    y_d=msg->y;
    loccalled=1;
  };


  //get loop frequency
  int getLoopF(){
    return loop_frequency;
  };

  // calculate desired linear velocity and angular velocity, outputs vector [lin,ang]
  std::vector<float> getVel(){

    std::vector<float> oldvel(2);
    oldvel[0]=vel[0];oldvel[1]=vel[1]; // old velocity is equal to velocity when this function was called

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
      // if length of r < error allowed, then don't move
        vel[0]=0; vel[1]=0;
      }
      else if (fabs(theta_d)>=theta_reduced_speed){
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

        float lin_diff=vel[0]-oldvel[0];
        float rot_diff=vel[1]-oldvel[1];

        int lin_diff_dir=1; // stores sign of linear difference
        int rot_diff_dir=1; // stores sign of rotational difference

        if(lin_diff<0){
        	lin_diff_dir=-1;
        };
        if(rot_diff<0){
        	rot_diff_dir=-1;
        };

        // check whether linear acceleration is exceeded, if it is than add max accel to current vel
        if((fabs(lin_diff)/loop_rate)>lin_accel_max){
           vel[0]=oldvel[0]+lin_diff_dir*lin_accel_max*loop_rate; //set new velocity
        };

        //check whether max rot acceleration is exceeded, if it is than add max accel to current vel
        if((fabs(rot_diff)/loop_rate)>rot_accel_max){
          vel[1]=oldvel[1]+rot_diff_dir*rot_accel_max*loop_rate;
        };

        //check whether max lin vel is exceeded, if it is, then set to max
        if(fabs(vel[0])>lin_vel_max){
        	if(vel[0]>0){
        		vel[0]=lin_vel_max;
        	}
        	else{
        		vel[0]=-lin_vel_max;
        	};
        };

        //check whether max rot vel is exceeded, if it is, then set to max
        if(fabs(vel[1])>rot_vel_max){
        	if(vel[1]>0){
        		vel[1]=rot_vel_max;
        	}
        	else{
        		vel[1]=-rot_vel_max;
        	};
        };

      }
      else if (fabs(theta_d)<theta_reduced_speed){

        // if the desired theta is less than the reduced speed theta, set theta according to control rule
        vel[0]=lin_vel_max;
        vel[1]=rot_vel_max/theta_reduced_speed*theta_d;

        float lin_diff=vel[0]-oldvel[0];
        float rot_diff=vel[1]-oldvel[1];

        int lin_diff_dir=1; // stores sign of linear difference
        int rot_diff_dir=1; // stores sign of rotational difference

        if(lin_diff<0){
        	lin_diff_dir=-1;
        };
        if(rot_diff<0){
        	rot_diff_dir=-1;
        };


        // check whether linear acceleration is exceeded, if it is than add max accel to current vel
        if((fabs(lin_diff)/loop_rate)>lin_accel_max){
           vel[0]=oldvel[0]+lin_diff_dir*lin_accel_max*loop_rate; //set new velocity
        };

        //check whether max rot acceleration is exceeded, if it is than add max accel to current vel
        if((fabs(rot_diff)/loop_rate)>rot_accel_max){
          vel[1]=oldvel[1]+rot_diff_dir*rot_accel_max*loop_rate;
        };

        //check whether max lin vel is exceeded, if it is, then set to max
        if(fabs(vel[0])>lin_vel_max){
        	if(vel[0]>0){
        		vel[0]=lin_vel_max;
        	}
        	else{
        		vel[0]=-lin_vel_max;
        	};
        };

        //check whether max rot vel is exceeded, if it is, then set to max
        if(fabs(vel[1])>rot_vel_max){
        	if(vel[1]>0){
        		vel[1]=rot_vel_max;
        	}
        	else{
        		vel[1]=-rot_vel_max;
        	};
        };
        
        ROS_INFO_STREAM("vel0: " << vel[0] << "vel1: " << vel[1] << "\n" << "oldvel0: " << oldvel[0] << "oldvel1: " << oldvel[1] << "\n");

      };
    //ROS_INFO_STREAM("vel0: " << vel[0] << "vel1: " << vel[1] "\n");
    //ROS_INFO_STREAM("theta_c: " << theta_c << "\n");
    //ROS_INFO_STREAM("theta_r: " << theta_r << "\n");
    //ROS_INFO_STREAM("roll: " << roll << "pitch: " << pitch <<  "yaw: " << yaw);

    };
  return vel;
  };

};

// Define constant driver variables
const float Createdriver::lin_vel_max=.5;
const float Createdriver::rot_vel_max=4.25;
const float Createdriver::theta_reduced_speed=3.14/2;
const float Createdriver::r_error=.2;
const float Createdriver::lin_accel_max=.5;
const float Createdriver::rot_accel_max=7;



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
  ros::Rate loop_rate(createdriver.getLoopF());


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
