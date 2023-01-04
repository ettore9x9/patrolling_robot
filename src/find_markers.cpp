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
#include <patrolling_robot/SetRoomPosition.h>
#include <surveillance_robot/Statement.h>
#include <mutex>

const int  NUM_MARKERS = 7;
static const std::string OPENCV_WINDOW = "Image window";

class FindMarker
{
private:
  ros::NodeHandle nh;
  ros::Subscriber image_sub;
  ros::Subscriber turn_sub;
  ros::Publisher movecam_pub;
  ros::Publisher statement_pub;
  ros::ServiceClient marker_cli;
  ros::ServiceClient room_cli;
  ros::Timer timerPublishStatement;
  aruco::MarkerDetector MD;
  std::vector<int> detected;
  std::mutex m;
  int turn = 0;

public:
  std::vector<std::pair<std::string, std::string>> Statement;

  FindMarker()
  {
  	MD.setDictionary("ARUCO");

    // Subscribe to input video feed
    image_sub     = nh.subscribe("/camera/rgb/image_raw", 1, &FindMarker::imageCb, this);
    turn_sub      = nh.subscribe("/camera_turn", 1, &FindMarker::turnCb, this);
    movecam_pub   = nh.advertise<patrolling_robot::MoveCamera>("/camera_command",1);
    statement_pub = nh.advertise<surveillance_robot::Statement>("/map/statement",1);
    marker_cli    = nh.serviceClient<patrolling_robot::RoomInformation>("/room_info");
    room_cli      = nh.serviceClient<patrolling_robot::SetRoomPosition>("/set_room_position");
    timerPublishStatement = nh.createTimer(ros::Duration(1.0 / 10.0), std::bind(&FindMarker::publishStatement, this));

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
        patrolling_robot::RoomInformation infoSrv;
        infoSrv.request.id = markers[i].id;
        if (marker_cli.call(infoSrv)){

          patrolling_robot::SetRoomPosition roomSrv;
          roomSrv.request.room = infoSrv.response.room;
          roomSrv.request.x = infoSrv.response.x;
          roomSrv.request.y = infoSrv.response.y;
          room_cli.call(roomSrv);
          
          m.lock();
          for (int j=0; j<infoSrv.response.connections.size(); j++) {
            Statement.push_back(std::make_pair(infoSrv.response.room.c_str(),infoSrv.response.connections[j].through_door.c_str()));
          }
          m.unlock();
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

  void publishStatement(){
    if (!Statement.empty()){
      m.lock();
      surveillance_robot::Statement msg;
      msg.location = Statement[0].first;
      msg.door = Statement[0].second;
      msg.stamp = ros::Time::now();
      statement_pub.publish(msg);
      Statement.erase(Statement.begin());
      m.unlock();
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "find_markers");
  FindMarker ic;
  ros::Duration(4).sleep();
  ros::Rate loop_rate(100);
  while (!ic.markers_found()){
    ic.loop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  ic.camCommand(0.0, 0.0, 0.0);
  ic.Statement.push_back(std::make_pair("",""));

  while(!ic.Statement.empty()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}