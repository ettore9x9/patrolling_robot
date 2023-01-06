/*! 
 * \file move_camera.cpp 
 * \brief Node to move the robot's camera
 * \author Ettore Sani
 * \version 0.1 
 * \date 06/01/2023
 * 
 * \details
 * 
 * Subscribes to: <BR>
 *   - /camera/camera_command
 *   - /joint_states
 * 
 * Publishes to: <BR>
 *   - /yawl_joint_velocity_controller/command
 *   - /vertical_joint_position_controller/command
 *   - /pitch_joint_position_controller/command
 *   - /camera/camera_turn
 * 
 * Description : 
 * 
 * Node to move the camera of the robot. It is an adapter from the camera/camera_command topic
 * to all the topics needed for moving each joint of the robot.
 * It also subscribes to the /joint_states topic, and triggers the /camera/camera_turn 
 * whenever the camera completed a full rotation.
 */
#include <ros/ros.h>
#include <patrolling_robot/MoveCamera.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

/*!
 * \class MoveCamera
 *
 * \brief This class provides a ROS node for moving the camera.
 *
 * This class is able to publish the commands to the joints to move the camera, and to detect
 * if the yawl_joint has made a full turn.
 */
class MoveCamera
{
  ros::NodeHandle nh;         //!< ROS node handle
  ros::Subscriber commandSub; //!< Subscriber for camera movement commands
  ros::Subscriber jointSub;   //!< Subscriber for joint state information
  ros::Publisher rotatePub;   //!< Publisher for rotating the camera
  ros::Publisher raisePub;    //!< Publisher for raising or lowering the camera
  ros::Publisher pitchPub;    //!< Publisher for changing the pitch of the camera
  ros::Publisher turnPub;     //!< Publisher to trigger when the camera has completed a full rotation
  bool turn = 0;              //!< Used for avoid repeating to trigger the turn topic

public:
  /*!
    * \brief Constructor for the MoveCamera class
    *
    * Initializes the subscribers and publishers for the ROS node.
    */
  MoveCamera()
  {
    commandSub = nh.subscribe("/camera/camera_command", 1, &MoveCamera::CamCb, this);
    jointSub   = nh.subscribe("/joint_states", 1, &MoveCamera::TurnCb, this);
    rotatePub  = nh.advertise<std_msgs::Float64>("/yawl_joint_velocity_controller/command",1);
    raisePub   = nh.advertise<std_msgs::Float64>("/vertical_joint_position_controller/command",1);
    pitchPub   = nh.advertise<std_msgs::Float64>("/pitch_joint_position_controller/command",1);
    turnPub    = nh.advertise<std_msgs::Bool>("/camera/camera_turn",1);
  }

  /*!
   * \brief Callback function for the camera movement command subscriber
   *
   * This function is called when the "/camera/camera_command" topic receives a message of type "MoveCamera".
   * It extracts the values for omega, raise, and pitch from the message and publishes them to the appropriate topics.
   *
   * \param command A message of type "MoveCamera" containing the camera movement command.
   */
  void CamCb(const patrolling_robot::MoveCamera command)
  {
  	std_msgs::Float64 omega;
  	std_msgs::Float64 raise;
  	std_msgs::Float64 pitch;
  	
  	omega.data = command.omega;
  	raise.data = command.raise;
  	pitch.data = command.pitch;

    rotatePub.publish(omega);
    raisePub.publish(raise);
    pitchPub.publish(pitch);
  }

  /*!
   * \brief Callback function for the joint state subscriber
   *
   * This function is called when a message is received on the "/joint_states" topic.
   * It checks the value of the position of the seventh joint (yawl).
   * If it is a multiple of 2pi, it publishes on the turnPub, to advertise that the camera performed a full rotation.
   * 
   * \param state A message of type "JointState" containing the joint state information.
   */
  void TurnCb(const sensor_msgs::JointState state) {
    if (state.position[6] > 0.02) {
      float theta = std::fmod(state.position[6], 6.28);
      if (theta > 0 && theta < 0.02 && !turn){
        ROS_INFO("Full camera's turn.");
        std_msgs::Bool res;
        res.data = 1;
        turn = 1;
        turnPub.publish(res);
      } else {
        turn = 0;
      }
    }
  }
};

/*!
 * \brief Main function for the move_camera ROS node
 *
 * This function initializes the ROS node and creates an instance of the MoveCamera class.
 * It then enters a loop to process incoming messages and publish the appropriate responses.
 *
 * \param argc The number of command line arguments
 * \param argv The array of command line arguments
 *
 * \return 0 if the node shuts down properly, -1 if there is an error.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_camera");
  MoveCamera mc;
  ros::spin();
  return 0;
}