#include <ros/ros.h>
#include <image_transport/image_transport.h>


#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <fstream>
using namespace std;
using namespace cv;
#define WINDOW "desired_image"
#define WINDOW1 "depth_image"
void image_depthCallback(const sensor_msgs::ImageConstPtr &msg)
{
    //转换ROS图像消息到opencv图像
      cv::Mat depth_image;
      depth_image = cv_bridge::toCvShare(msg)->image;


      cv::Mat desired_image;
      desired_image = cv_bridge::toCvShare(msg)->image;
      bool get_actual_depth(false);
      bool get_desired(false);
      bool get_depth(false);

      int widthU = depth_image.cols;				//图片宽度
      int heightV =depth_image.rows;				//图片长度

      char key1;
      char key2;
     // namedWindow(WINDOW);
      imshow(WINDOW,desired_image);

      key1= waitKey(30);

      imshow(WINDOW1,depth_image);

        key2= waitKey(30);

      //cout<<"nihao"<<endl;

     // key=cv::waitKey(33);
      if(key1==32)           //the Ascii of "Space key" is 32
        get_desired=true;

      if (key2 ==27)
        get_depth=true;

if (get_desired)
{
  get_desired=false;
  cout<<"desired_image has stored"<<endl;
  }

if (get_depth)
{get_depth=false;
  cout<<"depth start"<<endl;}

/*
 bool get_actual_depth(false);
 bool get_desired(false);
 bool get_depth(false);
 char key;
 key=cv::waitKey(33);
 if(key==32)           //the Ascii of "Space key" is 32
 get_desired = true;
 if(get_desired)
 {
   get_desired = false;
   ofstream write;
   write.open("/home/zhsyi/vision_servo_zsy/src/v_s/src/data.txt");
   write <<desired_image;
   write.close();

   fstream file;
   file.open("/home/zhsyi/vision_servo_zsy/src/v_s/src/desired.txt",ios::in);
   if(file.get()!=EOF)
   {
     cout<<"desired depth_image has been saved"<<endl;
   }
   file.close();

   get_depth=true;
  }


 if (get_depth)
 {
    get_depth=false;
   imshow(WINDOW1,depth_image);
   waitKey(30);
   cout<<"made it"<<endl;
   char key2;
   key2 =waitKey(33);
   if (key2 == 32)
        {cout<<"haode"<<endl;
        get_actual_depth = true;}


 }
 */
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_depthdata");
  ros::NodeHandle nh;

 // cv::namedWindow("Show depth-Imagedata");
 // cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/kinect2/qhd/image_depth_rect", 1, image_depthCallback);
  ros::spin();
 // cv::destroyWindow("Show depth-imagedata");
  return 0;
}
