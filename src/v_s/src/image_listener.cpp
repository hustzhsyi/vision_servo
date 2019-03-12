#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sensor_msgs/image_encodings.h> 
 
using std::cout;
using std::endl;
using std::stringstream;
using std::string;
 
unsigned int fileNum = 1;
bool saveCloud(false);
 
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  /*
  cv::imshow("Show depth-greyImage", cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::TYPE_8UC1)->image);
  cv::Mat img,dst_img;
  img = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::TYPE_8UC1)->image;
  cv::equalizeHist(img,dst_img);
  cv::imshow("Show depth-equalizeImage", dst_img);
  */
  char key;
  cv::Mat depthimage;
  depthimage = cv_bridge::toCvShare(msg)->image;
  cv::imshow("Show desired depthimage", depthimage);
  cv::waitKey(30);
  key=cv::waitKey(33);
  if(key==32)           //the Ascii of "Space key" is 32 
  saveCloud = true;
  if(saveCloud)
  {
    stringstream stream;
    stringstream stream1;
    stream <<"Goal depth-greyImage" << fileNum<<".jpg";
    stream1 <<"/home/zhsyi/vision_servo_zsy/src/v_s/src/image/" << fileNum <<".jpg";
    string filename = stream.str();
    string filename1 = stream1.str();
//深度图的编码的方式有：TYPE_8UC1 TYPE_64FC1 ... 
   // cv::imwrite(filename1,cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::TYPE_8UC1)->image);

    cv::imwrite(filename1,depthimage);
    saveCloud = false;
    fileNum++;
    cout << filename1 << " had  Saved."<< endl;
  }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("Show depth-greyImage");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/kinect2/qhd/image_depth_rect", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("Show depth-greyImage");
}



