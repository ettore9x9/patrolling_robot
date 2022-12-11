#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/aruco.h>
#include <iostream>

static const std::string OPENCV_WINDOW = "Image window";

class FindMarker
{
  ros::NodeHandle nh_;
  ros::Subscriber image_sub_;
  ros::Publisher image_pub_;
  aruco::MarkerDetector MDetector;

public:
  FindMarker()
  {
  	//MDetector.setDictionary("ARUCO_MIP_36h12");

    // Subscribe to input video feed
    image_sub_ = nh_.subscribe("/camera/rgb/image_raw", 1, &FindMarker::imageCb, this);

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

    std::vector<aruco::Marker> markers=MDetector.detect(cv_ptr->image);

    for(size_t i=0;i<markers.size();i++){ 
    	markers[i].draw(cv_ptr->image);
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "find_markers");
  FindMarker ic;
  ros::spin();
  return 0;
}