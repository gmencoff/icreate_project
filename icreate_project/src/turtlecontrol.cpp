#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>
#include <vector>

// this class will calculate the twist message to send to cmd_vel
class Turtledriver
{

private:
  float tx;
  float ty;
  float ttheta;
  float dx;
  float dy;
  int loccalled;
public:
  // gets current position
  void poseCallback(const turtlesim::Pose::ConstPtr& msg){
    tx = msg->x;
    ty = msg->y;
    ttheta = msg->theta;
    };

  // gets desired position
  void locCallback(const turtlesim::Pose::ConstPtr& msg){
    dx = msg->x;
    dy = msg->y;
    loccalled = 1;
  };

  // calculate desired linear velocity and angular velocity, outputs vector [lin,ang]
  std::vector<float> getVel(){
    std::vector<float>  vel(2); vel[0]=0;vel[1]=0;

    if (loccalled==1){
    //if desired location has been set, update velocity
    }
    else{
    //if desired location has not been set, return velocity is 0
      return vel;
    }
  };

};

//void poseCallback(const turtlesim::Pose::ConstPtr& msg){
//  ROS_INFO("TurtlePose: [%f,%f,%f]",msg->x,msg->y,msg->theta);
//};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlecontrol"); //initialize turtledriver node
  ros::NodeHandle n; // create a ros node handle

  // Specify that this node will publish a geometry msg to turtle cmd vel
  ros::Publisher cmdvel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

  // create a Turtledriver object and subscriber will subscribe in that object
  Turtledriver turtledriver;
  ros::Subscriber pose_sub = n.subscribe("/turtle1/pose",1000,&Turtledriver::poseCallback,&turtledriver);
  ros::Subscriber loc_sub = n.subscribe("/desired_loc",1000,&Turtledriver::locCallback,&turtledriver);

  //ros::Subscriber sub = n.subscribe("/turtle1/pose",1000,poseCallback);

  // 1 Hz publishing rate
  ros::Rate loop_rate(1);


  int count = 0; // count number of times looped

  while (ros::ok())
  {
    geometry_msgs::Twist vel; //twist object to send to cmd_vel

    ros::spinOnce(); // call the queued callback functions

    std::vector<float> velvec = turtledriver.getVel(); // get the calculated velocity based on desired and current location

    vel.linear.x = velvec[0]; //set velocity according to output from turtle driver
    vel.angular.z = velvec[1];

    cmdvel_pub.publish(vel); // publish message as specified by cmdvel_pub

    ROS_INFO_STREAM("xVel: " << vel.linear.x << "\n" << "thetaVel: " << vel.angular.z << "\n");

    loop_rate.sleep(); //execute at frequency specified by loop rate
    ++count;
  }


  return 0;
}
