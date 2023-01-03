#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/aruco.h>
#include <iostream>
#include <patrolling_robot/MoveCamera.h>
#include <unistd.h>
#include <std_msgs/Bool.h>
#include <patrolling_robot/RoomConnection.h>
#include <patrolling_robot/RoomInformation.h>
#include <surveillance_robot/Statement.h>

const int  NUM_MARKERS = 7;
static const std::string OPENCV_WINDOW = "Image window";

class FindMarker
{
  ros::NodeHandle nh;
  ros::Subscriber image_sub;
  ros::Subscriber turn_sub;
  ros::Publisher movecam_pub;
  ros::Publisher statement_pub;
  ros::ServiceClient marker_cli;
  aruco::MarkerDetector MD;
  std::vector<int> detected;
  int turn = 0;

public:
  FindMarker()
  {
  	MD.setDictionary("ARUCO");

    // Subscribe to input video feed
    image_sub   = nh.subscribe("/camera/rgb/image_raw", 1, &FindMarker::imageCb, this);
    turn_sub    = nh.subscribe("/camera_turn", 1, &FindMarker::turnCb, this);
    movecam_pub = nh.advertise<patrolling_robot::MoveCamera>("/camera_command",1);
    statement_pub = nh.advertise<surveillance_robot::Statement>("/map/statement",1);
    marker_cli  = nh.serviceClient<patrolling_robot::RoomInformation>("/room_info");

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~FindMarker()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    std::vector<aruco::Marker> markers=MD.detect(cv_ptr->image);

    for(size_t i=0;i<markers.size();i++){
      if (!std::count(detected.begin(), detected.end(), markers[i].id)) {
        detected.push_back(markers[i].id);
        ROS_INFO("Aruco with id %d detected.", markers[i].id);
        patrolling_robot::RoomInformation srv;
        srv.request.id = markers[i].id;
        if (marker_cli.call(srv)){
          surveillance_robot::Statement msg;
          msg.location = srv.response.room.c_str();
          msg.door = srv.response.connections[0].through_door.c_str();
          msg.stamp = ros::Time::now();
          statement_pub.publish(msg);
          ROS_INFO("Room %s detected", srv.response.room.c_str());
        }
      }
    	markers[i].draw(cv_ptr->image);
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
  }

  void turnCb(const std_msgs::Bool& msg){
    if (msg.data == true)
      turn++;
  }

  void loop(){
    switch (turn){
    case 0:
      camCommand(0.8, 0.034, 0.2);
      break;
    case 1:
      camCommand(0.8, 0.034, -0.5);
      break;
    default:
      camCommand(0.8, 0.034, rand()/RAND_MAX - 0.5);
      break;
    }
  }

  void camCommand (float omega, float raise, float pitch) {
    patrolling_robot::MoveCamera moveCam;
    moveCam.omega = omega;
    moveCam.raise = raise;
    moveCam.pitch = pitch;
    movecam_pub.publish(moveCam);
  }

  int markers_found(){
    if (detected.size() == NUM_MARKERS)
      return 1;
    return 0;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "find_markers");
  FindMarker ic;
  sleep(5);
  ros::Rate loop_rate(100);
  while (!ic.markers_found()){
    ic.loop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  ic.camCommand(0.0, 0.0, 0.0);
  return 0;
}