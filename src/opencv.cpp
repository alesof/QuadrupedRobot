#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "zbar.h"
#include "std_msgs/String.h"
using namespace cv;
using namespace zbar;

class ROS_IMG_READER {
  ros::NodeHandle _nh;
  ros::Subscriber _ros_img_sub; 
  ros::Publisher  _opencv_image_pub;
  ros::Publisher  _marker_pub;


public:
  ROS_IMG_READER() {
    _ros_img_sub = _nh.subscribe("/camera/image_raw", 1, &ROS_IMG_READER::imageCb, this);
    _opencv_image_pub = _nh.advertise<sensor_msgs::Image>("/opencv/raw_image", 1);
    _marker_pub = _nh.advertise<std_msgs::String>("/opencv/marker_data", 1);

  }

  void QRscan(cv_bridge::CvImagePtr cv_ptr){

      ImageScanner scanner;
      scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1); //Config settings
      cv::Mat frame;
      cv::Mat frame_grayscale;

      frame = cv_ptr -> image; //Use a frame from cv messages
      cvtColor(frame,frame_grayscale,CV_BGR2GRAY); //Gray scale conversion
      cv::threshold(frame_grayscale,frame_grayscale,0,255,cv::THRESH_BINARY+cv::THRESH_OTSU); 
      imshow("Display window", frame_grayscale);
      cv::waitKey(10);
      //Get Dimensions and pointer
      int width = frame_grayscale.cols;
      int height = frame_grayscale.rows;
      uchar *raw = (uchar *)(frame_grayscale.data);

      //Define Image
      Image image(width, height, "Y800", raw, width*height);

      //Scan the Image
      scanner.scan(image);



      std_msgs::String _marker_msg;
      for(Image::SymbolIterator symbol = image.symbol_begin(); symbol!= image.symbol_end(); ++symbol){
        _marker_msg.data= symbol->get_data();
        _marker_pub.publish(_marker_msg);
      }




  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg) {

    cv_bridge::CvImagePtr cv_ptr;
    namespace enc = sensor_msgs::image_encodings;
    cv::Mat img;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
            img = cv_ptr->image;
            /* imshow("Display window", img);
            cv::waitKey(10); Wait for a keystroke in the window */
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    QRscan(cv_ptr);
    _opencv_image_pub.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ROS_IMG_READER");
  ROS_IMG_READER ic;
  ros::spin();
  return 0;
}

