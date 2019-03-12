#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <cmath>
#include <vector>
#include <fstream>
#define WINDOWS1 "depth_image"
#define WINDOWS2 "depth_greyimage"

using namespace cv;
using namespace std;
using std::stringstream;
using std::string;
using std::vector;

//camera intriparam: [ 3.6200722426806516e+02, 0., 2.5309378269159924e+02, 0.,3.6188241930411635e+02, 2.0502034288339061e+02, 0., 0., 1. ]
double cam[9]={ 3.6200722426806516e+02,0,2.5309378269159924e+02, 0,3.6188241930411635e+02,2.0502034288339061e+02, 0, 0, 1};
Mat camera_matrix = Mat(3,3,CV_64FC1,cam);

//calculate A and B
void grads(const Mat &picture_grey,vector<vector<double> > &gradx,vector<vector<double> > &grady)
{
  //vector<vector<double>> gradx(picture.row-1,vector<double>(picture.col-1));
  //vector<vector<double>> grady(picture.row-1,vector<double>(picture.col-1));

  for (int i=0;i<picture_grey.rows-1;i++)
  {
    for (int j=0;j<picture_grey.cols-1;j++)
    {
      gradx[i][j]=(picture_grey.at<uchar>(i,j+1) - picture_grey.at<uchar>(i,j) + picture_grey.at<uchar>(i+1,j+1) - picture_grey.at<uchar>(i+1,j))/2;
      grady[i][j]=(picture_grey.at<uchar>(i+1,j) - picture_grey.at<uchar>(i,j) + picture_grey.at<uchar>(i+1,j+1) - picture_grey.at<uchar>(i,j+1))/2;
    }
  }
}

void calculate_XY_c( Mat &picture_depth,vector<vector<double> >& X_c,vector<vector<double> >& Y_c)
{
  double Fx = camera_matrix.at<uchar>(0,0);
  double Fy = camera_matrix.at<uchar>(1,1);
  double cx  = camera_matrix.at<uchar>(0,2);
  double cy  = camera_matrix.at<uchar>(1,2);

  for (int i=0;i<picture_depth.rows-1;i++)
  {
    for (int j=0;j<picture_depth.cols-1;j++)
    {
      if (picture_depth.at<uchar>(i,j)<=0 || picture_depth.at<uchar>(i,j)>4000)
      {
        picture_depth.at<uchar>(i,j)=0;
        X_c[i][j]=0;
        Y_c[i][j]=0;
      }
      else
      {
        X_c[i][j]=picture_depth.at<uchar>(i,j)*(i-cx)/Fx;
        Y_c[i][j]=picture_depth.at<uchar>(i,j)*(j-cy)/Fy;
      }
    }
  }
}


void calculate_xy_p(Mat &picture_depth,const vector<vector<double> >& X_c,const vector<vector<double> >& Y_c,vector<vector<double> >& x_p, vector<vector<double> >& y_p)
{
  for (int i=0;i<picture_depth.rows-1;i++)
  {
    for (int j=0;j<picture_depth.cols-1;j++)
    {
      if (picture_depth.at<uchar>(i,j)=0)
      {
        x_p[i][j]=0;
        y_p[i][j]=0;
      }
      else
      {
        x_p[i][j]=X_c[i][j]/picture_depth.at<uchar>(i,j);
        y_p[i][j]=Y_c[i][j]/picture_depth.at<uchar>(i,j);
      }
    }
  }
}

void calculate_lzt(const Mat &picture_grey,vector<vector<double> > &gradx,vector<vector<double> > &grady,vector<vector<double> > x, vector<vector<double> > y,vector<vector<double> > &lztx,vector<vector<double> > &lzty,vector<vector<double> > &lztz)
{
  for (int i=0;i<picture_grey.rows-1;i++)
  {
    for (int j=0;j<picture_grey.cols-1;j++)
    {
      lztx[i][j]=gradx[i][j]/picture_grey.at<uchar>(i,j);
      lzty[i][j]=grady[i][j]/picture_grey.at<uchar>(i,j);
      lztz[i][j]=(-1)*(picture_grey.at<uchar>(i,j) + x[i][j]*gradx[i][j] +y[i][j]*grady[i][j])/picture_grey.at<uchar>(i,j);
    }
  }
}

void calculate_lzw(const Mat &picture_grey,vector<vector<double> > &gradx,vector<vector<double> > &grady,vector<vector<double> > x, vector<vector<double> > y,vector<vector<double> > &lzwx,vector<vector<double> > &lzwy,vector<vector<double> > &lzwz)
{
  for (int i=0;i<picture_grey.rows-1;i++)
  {
    for (int j=0;j<picture_grey.cols-1;j++)
    {
      lzwx[i][j] = ((-1)*y[i][j]*picture_grey.at<uchar>(i,j)) - x[i][j]*y[i][j]*gradx[i][j] + (1+y[i][j]*y[i][j])*grady[i][j];
      lzwy[i][j] = x[i][j]*picture_grey.at<uchar>(i,j) + (1+x[i][j]*x[i][j])*gradx[i][j] + x[i][j]*y[i][j]*grady[i][j];
      lzwz[i][j] = x[i][j]*grady[i][j] - y[i][j]*gradx[i][j];
    }
  }
}



void image_Callback(const sensor_msgs::ImageConstPtr& msg)
{
  //对深度图进行直方图均衡化后的灰度图
    cv::Mat grey_img,grey_equ_img;
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr=cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::TYPE_8UC1);
    grey_img=cv_ptr->image;
    //cv::imshow("Show depth-greyImage", grey_img);
    cv::equalizeHist(grey_img,grey_equ_img);

  //获得深度图的实际数据
  //转换ROS图像消息到opencv图像
    cv::Mat depth_image;
    depth_image = cv_bridge::toCvShare(msg)->image;

    int widthU = depth_image.cols;				//图片宽度
    int heightV =depth_image.rows;				//图片长度
  //ROS_INFO("zhsyi heard: [%d],[%d]",widthU,heightV);

    cout<<"width="<<widthU<<endl;
    cout<<"height="<<heightV<<endl;

    imshow(WINDOWS1,depth_image);
    imshow(WINDOWS2,grey_equ_img);
    waitKey(30);

    vector<vector<double> > gradx(heightV-1, vector<double>(widthU-1));
    vector<vector<double> > grady(heightV-1, vector<double>(widthU-1));
    vector<vector<double> > X_c(heightV-1, vector<double>(widthU-1));
    vector<vector<double> > Y_c(heightV-1, vector<double>(widthU-1));
    vector<vector<double> > x_p(heightV-1, vector<double>(widthU-1));
    vector<vector<double> > y_p(heightV-1, vector<double>(widthU-1));
    vector<vector<double> > lztx(heightV-1, vector<double>(widthU-1));
    vector<vector<double> > lzty(heightV-1, vector<double>(widthU-1));
    vector<vector<double> > lztz(heightV-1, vector<double>(widthU-1));
    vector<vector<double> > lzwx(heightV-1, vector<double>(widthU-1));
    vector<vector<double> > lzwy(heightV-1, vector<double>(widthU-1));
    vector<vector<double> > lzwz(heightV-1, vector<double>(widthU-1));

    grads(grey_equ_img,gradx,grady);
    calculate_XY_c(depth_image,X_c,Y_c);
    calculate_xy_p(depth_image,X_c,Y_c,x_p,y_p);
    calculate_lzt(grey_equ_img,gradx,grady,x_p,y_p,lztx,lzty,lztz);
    calculate_lzw(grey_equ_img,gradx,grady,x_p,y_p,lzwx,lzwy,lzwz);
}


/*void image_depthCallback(const sensor_msgs::ImageConstPtr &image_depth)
{
      //转换ROS图像消息到opencv图像
        cv::Mat depthimage;
        depthimage = cv_bridge::toCvShare(image_depth)->image;

        int widthU = depthimage.cols;				//图片宽度
        int heightV =depthimage.rows;				//图片长度
      //ROS_INFO("zhsyi heard: [%d],[%d]",widthU,heightV);

        cout<<"width="<<widthU<<endl;
        cout<<"height="<<heightV<<endl;
        cv::Mat data(heightV,widthU,CV_32FC1);



        ofstream write;
        write.open("/home/zhsyi/vision_servo_zsy/src/v_s/src/data.txt");
        write <<depthimage;
        write.close();



        vector<vector<double> > X_c(heightV-1, vector<double>(widthU-1));
        vector<vector<double> > Y_c(heightV-1, vector<double>(widthU-1));


        imshow(WINDOWS1,depthimage);
        waitKey(30);
}*/


int main(int argc, char **argv)
{
  ros::init(argc, argv, "handle_image");
  ros::NodeHandle nh;
//cv::namedWindow("Show depth-greyImage");
//cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_image = it.subscribe("/kinect2/qhd/image_depth_rect", 1, image_Callback);
  //image_transport::Subscriber sub_depthdata = it.subscribe("/kinect2/qhd/image_depth_rect", 1, image_depthCallback);

  ros::spin();
//cv::destroyWindow("Show depth-greyImage");
  return 0;
}



