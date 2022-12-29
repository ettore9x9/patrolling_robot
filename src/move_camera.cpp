#include <ros/ros.h>
#include <patrolling_robot/MoveCamera.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

class MoveCamera
{
  ros::NodeHandle nh;
  ros::Subscriber command_sub;
  ros::Subscriber joint_sub;
  ros::Publisher rotate_pub;
  ros::Publisher raise_pub;
  ros::Publisher pitch_pub;
  ros::Publisher turn_pub;
  bool turn = 0;

public:
  MoveCamera()
  {
    command_sub = nh.subscribe("/camera_command", 1, &MoveCamera::camCb, this);
    joint_sub   = nh.subscribe("/joint_states", 1, &MoveCamera::turnCb, this);
    rotate_pub  = nh.advertise<std_msgs::Float64>("/yawl_joint_velocity_controller/command",1);
    raise_pub   = nh.advertise<std_msgs::Float64>("/vertical_joint_position_controller/command",1);
    pitch_pub   = nh.advertise<std_msgs::Float64>("/pitch_joint_position_controller/command",1);
    turn_pub    = nh.advertise<std_msgs::Bool>("/camera_turn",1);
  }

  void camCb(const patrolling_robot::MoveCamera msg)
  {
  	std_msgs::Float64 omega;
  	std_msgs::Float64 raise;
  	std_msgs::Float64 pitch;
  	
  	omega.data = msg.omega;
  	raise.data = msg.raise;
  	pitch.data = msg.pitch;

    rotate_pub.publish(omega);
    raise_pub.publish(raise);
    pitch_pub.publish(pitch);
  }

  void turnCb(const sensor_msgs::JointState msg) {
    if (msg.position[6] > 0.01) {
      float theta = std::fmod(msg.position[6], 6.28);
      if (theta > 0 && theta < 0.01 && !turn){
        std::cout << std:: endl<< "turn" << std::endl;
        std_msgs::Bool res;
        res.data = 1;
        turn = 1;
        turn_pub.publish(res);
      } else {
        turn = 0;
      }
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_camera");
  MoveCamera MC;
  ros::spin();
  return 0;
}