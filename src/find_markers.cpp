/*! 
 * \file find_markers.cpp 
 * \brief Node to find the markers in the environment
 * \author Ettore Sani
 * \version 0.1 
 * \date 06/01/2023
 * 
 * \param [in] nMarker number of markers to detect
 * 
 * \details
 * 
 * Subscribes to: <BR>
 *   - /camera/rgb/image_raw
 *   - /camera/camera_turn
 * 
 * Publishes to: <BR>
 *   - /camera/camera_command
 *   - /map/statement
 * 
 * Client of: <BR>
 *   - /room_info 
 *   - /info/set_room_position
 * 
 * Description : 
 * 
 * Node to detect markers around the robot by moving the camera.
 * It request to the marker_server node to know the markrer meaning, and publishes the 
 * received informations to the nodes state_machine and rosbot_state.
 */
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/aruco.h>
#include <patrolling_robot/MoveCamera.h>
#include <std_msgs/Bool.h>
#include <patrolling_robot/RoomConnection.h>
#include <patrolling_robot/RoomInformation.h>
#include <patrolling_robot/SetRoomPosition.h>
#include <surveillance_robot/Statement.h>
#include <boost/optional.hpp>
#include <mutex>

/*! \brief Constant string for the name of the OpenCV window. */
static const std::string c_opencvWindow = "Image window";

/*!
 * \class FindMarker
 *
 * \brief A class for detecting markers in images and publishing the results.
 *
 * This class is able to find Aruco markers around the robot, it subscribes to image and turn data, 
 * processes the data to detect markers, and publishes the results.
 * It publishes to the move_camera node to let the markers enter in the field of view of the camera, 
 * more in details, the method Loop is used to decide how the camera must move and the method CamCommand
 * is used for sending the camera commands.
 * Moreover, it publishes asynchronously the statements to the state_machine node, because one marker detection
 * can correspond to multiple statements.
 * It also calls the set_room_position server to store the x y positions of each room.
 */
class FindMarker {
private:
  ros::NodeHandle nh;               //!< ROS node handle
  ros::Subscriber imageSub;         //!< ROS image subscriber
  ros::Subscriber turnSub;          //!< ROS turn subscriber
  ros::Publisher moveCamPub;        //!< ROS publisher for moving the camera
  ros::Publisher statPub;           //!< ROS publisher for statements
  ros::ServiceClient markerCli;     //!< ROS client for marker_server
  ros::ServiceClient roomCli;       //!< ROS client for room information service
  ros::Timer timerPublishStatement; //!< ROS timer for publishing statements
  aruco::MarkerDetector md;         //!< Aruco marker detector
  std::vector<int> markersDetected; //!< Vector of detected marker IDs
  std::mutex mutex;                 //!< Mutex for synchronizing access to markersDetected
  int turn = 0;                     //!< Number of complete turns
  int nMarker = 0;                  //!< Total number of markers to detect

public:
  std::vector<std::pair<std::string, std::string>> statPairVec; //!< Vector of pairs of status information. The first element is the location, the second the door
  /*!
   * \brief Constructor for FindMarker class.
   *
   * This constructor sets up the ROS subscribers, publishers, and service clients,
   * and initializes the marker detector. It also sets up the OpenCV window for
   * displaying images.
   */
  FindMarker() {
    ros::param::get("env/marker_number", nMarker);
    ROS_INFO("@find_markers: The markers to detect: %d", nMarker);

  	md.setDictionary("ARUCO");
    ROS_INFO("@find_markers: Set dictionary 'ARUCO' to detect markers.");

    imageSub   = nh.subscribe("/camera/rgb/image_raw", 10, &FindMarker::ImageCb, this);
    turnSub    = nh.subscribe("/camera/camera_turn", 10, &FindMarker::TurnCb, this);
    moveCamPub = nh.advertise<patrolling_robot::MoveCamera>("/camera/camera_command",10);
    statPub    = nh.advertise<surveillance_robot::Statement>("/map/statement",10);
    markerCli  = nh.serviceClient<patrolling_robot::RoomInformation>("/room_info");
    roomCli    = nh.serviceClient<patrolling_robot::SetRoomPosition>("/info/set_room_position");

    timerPublishStatement = nh.createTimer(ros::Duration(1.0 / 10.0), std::bind(&FindMarker::PublishStatement, this));

    cv::namedWindow(c_opencvWindow);
  }

  /*!
   * \brief Destructor for FindMarker class.
   *
   * This destructor closes the OpenCV window.
   */
  ~FindMarker() {
    cv::destroyWindow(c_opencvWindow);
  }

  /*!
   * \brief Callback function for image data.
   *
   * This function processes the image data to detect markers, and stores the
   * detected marker IDs in the markersDetected vector. It also calls the
   * marker information and set room position services, and displays the image
   * with the detected markers on the OpenCV window.
   *
   * \param imageFeed The image data.
   */
  void ImageCb(const sensor_msgs::ImageConstPtr& imageFeed) {
    cv_bridge::CvImagePtr cvPtr;
    try {
      cvPtr = cv_bridge::toCvCopy(imageFeed, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("@find_markers: cv_bridge exception: %s", e.what());
      return;
    }

    std::vector<aruco::Marker> markers = md.detect(cvPtr->image);

    for( size_t i=0; i < markers.size(); i++ ) {
      if ( !std::count(markersDetected.begin(), markersDetected.end(), markers[i].id) ) {
        markersDetected.push_back(markers[i].id);
        ROS_INFO("@find_markers: Aruco with id %d detected.", markers[i].id);

        patrolling_robot::RoomInformation infoSrv;
        infoSrv.request.id = markers[i].id;

        if (markerCli.call(infoSrv)) {
          mutex.lock();
          for ( size_t j=0; j < infoSrv.response.connections.size(); j++ )
            statPairVec.push_back(std::make_pair(infoSrv.response.room.c_str(),infoSrv.response.connections[j].through_door.c_str()));
          mutex.unlock();

          patrolling_robot::SetRoomPosition roomSrv;
          roomSrv.request.room = infoSrv.response.room;
          roomSrv.request.x = infoSrv.response.x;
          roomSrv.request.y = infoSrv.response.y;
          roomCli.call(roomSrv);
          ROS_INFO("@find_markers: Location %s is at coordinates x:%f, y:%f", infoSrv.response.room.c_str(), infoSrv.response.x, infoSrv.response.y);
        } else 
          ROS_WARN("@find_markers: Marker with id %d not found in the marker_server.", markers[i].id);
      }
    	markers[i].draw(cvPtr->image);
    }
    cv::imshow(c_opencvWindow, cvPtr->image);
    cv::waitKey(3);
  }

  /*!
   * \brief Callback function for camera turning trigger.
   *
   * This function increments the turn variable when the trigger is received.
   *
   * \param trigger The camera turning trigger.
   */
  void TurnCb(const std_msgs::Bool& trigger){
    if (trigger.data == true)
      turn++;
  }

  /*!
   * \brief Function for sending camera movement commands based on the current turn.
   *
   * This function sends a camera movement command based on the current value of the
   * turn variable.
   *
   * The camera movement command is sent to the moveCamPub publisher.
   */
  void Loop(){
    switch (turn){
    case 0:
      CamCommand(0.8, 0.034, 0.2);
      break;
    case 1:
      CamCommand(0.8, 0.034, -0.5);
      break;
    default:
      CamCommand(0.8, 0.034, rand()/RAND_MAX - 0.5);
      break;
    }
  }

  /*!
   * \brief Function for sending camera movement commands.
   *
   * This function sends a camera movement command with the specified angles
   * to the moveCamPub publisher.
   *
   * \param omega The rotational angle about the vertical axis in radians.
   * \param raise The distance of elevation in meters.
   * \param pitch The angle of inclination in radians.
   */
  void CamCommand (float omega, float raise, float pitch) {
    patrolling_robot::MoveCamera moveCam;
    moveCam.omega = omega;
    moveCam.raise = raise;
    moveCam.pitch = pitch;
    moveCamPub.publish(moveCam);
  }

  /*!
   * \brief Function for checking if all markers have been detected.
   *
   * This function returns 1 if all markers have been detected, and 0 otherwise.
   *
   * \return 1 if all markers have been detected, 0 otherwise.
   */
  int MarkersFound(){
    if (markersDetected.size() == nMarker)
      return 1;
    return 0;
  }

  /*!
   * \brief Function for publishing statements.
   *
   * This function publishes statements stored in the statPairVec vector to the statPub
   * publisher, and removes the published statements from the vector. The operations on 
   * the statPairVec are protected by a mutex.
   */
  void PublishStatement(){
    if (!statPairVec.empty()){
      surveillance_robot::Statement statToSend;
      mutex.lock();
      statToSend.location = statPairVec[0].first;
      statToSend.door = statPairVec[0].second;
      statPairVec.erase(statPairVec.begin());
      mutex.unlock();
      statToSend.stamp = ros::Time::now();
      statPub.publish(statToSend);
    }
  }
};

/*!
 * \brief Main function for the find_markers node.
 *
 * This function creates an instance of the FindMarker class and waits for 4 seconds, waiting for 
 * the state_machine node.
 * It then enters a loop where it sends camera movement commands and spins the ROS event
 * loop until all markers have been detected.
 *
 * After all markers have been detected, the camera is stopped and the statPairVec vector
 * is pushed a blank pair, this means that the ontology is complete. 
 * The loop then continues until the statPairVec vector is empty,
 * at which point the function exits.
 *
 * \param argc The number of command line arguments.
 * \param argv The array of command line arguments.
 *
 * \return 0 on success, non-zero otherwise.
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "find_markers");
  FindMarker fm;
  ros::Duration(4).sleep();
  ros::Rate loop_rate(100);
  while (!fm.MarkersFound()){
    fm.Loop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  fm.CamCommand(0.0, 0.0, 0.0);
  fm.statPairVec.push_back(std::make_pair("",""));
  while(!fm.statPairVec.empty()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}