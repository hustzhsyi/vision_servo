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
//using std::endl;
using std::stringstream;
using std::string;
using std::vector;

//camera intriparam: [ 3.6200722426806516e+02, 0., 2.5309378269159924e+02, 0.,3.6188241930411635e+02, 2.0502034288339061e+02, 0., 0., 1. ]
double cam[9]={ 3.6200722426806516e+02,0,2.5309378269159924e+02, 0,3.6188241930411635e+02,2.0502034288339061e+02, 0, 0, 1};
Mat camera_matrix = Mat(3,3,CV_64FC1,cam);

//calculate A and B
void grads(const Mat &picture,vector<vector<double> > &gradx,vector<vector<double> > &grady)
{
  //vector<vector<double>> gradx(picture.row-1,vector<double>(picture.col-1));
  //vector<vector<double>> grady(picture.row-1,vector<double>(picture.col-1));

  for (int i=0;i<picture.rows-1;i++)
  {
    for (int j=0;j<picture.cols-1;j++)
    {
      gradx[i][j]=(picture.at<uchar>(i,j+1) - picture.at<uchar>(i,j) + picture.at<uchar>(i+1,j+1) - picture.at<uchar>(i+1,j))/2;
      grady[i][j]=(picture.at<uchar>(i+1,j) - picture.at<uchar>(i,j) + picture.at<uchar>(i+1,j+1) - picture.at<uchar>(i,j+1))/2;
    }
  }
}



//calculate x and y
/* void calculate_xy(const Mat &picture,vector<double> &x_,vector<double> &y_)
{
  double cu0 = camera_matrix[0][0];
  double cv0 = camera_matrix[1][1];
  double dx  = camera_matrix[0][2];
  double dy  = camera_matrix[1][2];
  for (int u=0;u<picture.cols-1;u++)
  {
    x_[u]=((double)u-cu0)*dx;
  }
  for (int v=0;v<picture.rows-1;v++)
  {
    y_[v]=((double)v-cv0)*dy;
  }
}
*/

void calcilate_XY_c( Mat &picture_depth,vector<vector<double> >& X_c,vector<vector<double> >& Y_c)
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


void calcilate_xy_p(Mat &picture_depth,const vector<vector<double> >& X_c,const vector<vector<double> >& Y_c,vector<vector<double> >& x_p, vector<vector<double> >& y_p)
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

void calculate_lzt(const Mat &picture,vector<vector<double> > &gradx,vector<vector<double> > &grady,vector<double> x,vector<double> y,vector<vector<double> > &lztx,vector<vector<double> > &lzty,vector<vector<double> > &lztz)
{
  for (int i=0;i<picture.rows-1;i++)
  {
    for (int j=0;j<picture.cols-1;j++)
    {
      lztx[i][j]=gradx[i][j]/picture.at<uchar>(i,j);
      lzty[i][j]=grady[i][j]/picture.at<uchar>(i,j);
      lztz[i][j]=(-1)*(picture.at<uchar>(i,j) + x[i]*gradx[i][j] +y[i]*grady[i][j])/picture.at<uchar>(i,j);
    }
  }
}

void calculate_lzw(const Mat &picture,vector<vector<double> > &gradx,vector<vector<double> > &grady,vector<double> x,vector<double> y,vector<vector<double> > &lzwx,vector<vector<double> > &lzwy,vector<vector<double> > &lzwz)
{
  for (int i=0;i<picture.rows-1;i++)
  {
    for (int j=0;j<picture.cols-1;j++)
    {
      lzwx[i][j] = ((-1)*y[i]*picture.at<uchar>(i,j)) - x[j]*y[i]*gradx[i][j] + (1+y[i]*y[i])*grady[i][j];
      lzwy[i][j] = x[j]*picture.at<uchar>(i,j) + (1+x[j]*x[j])*gradx[i][j] + x[j]*y[i]*grady[i][j];
      lzwz[i][j] = x[j]*grady[i][j] - y[i]*gradx[i][j];
    }
  }
}


//对深度图进行直方图均衡化后的灰度图
void image_greyCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat img,dst_img;
  cv_bridge::CvImageConstPtr cv_ptr;
  cv_ptr=cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::TYPE_8UC1);
  img=cv_ptr->image;
  //cv::imshow("Show depth-greyImage", img);
  cv::equalizeHist(img,dst_img);
  cv::imshow("WINDOWS2", dst_img);
  cv::waitKey(30);


  int widthU = dst_img.cols;				//图片宽度
  int heightV =dst_img.rows;				//图片长度
  vector<vector<double> > gradx(heightV-1, vector<double>(widthU-1));
  vector<vector<double> > grady(heightV-1, vector<double>(widthU-1));
  vector<double> x(widthU-1);
  vector<double> y(heightV-1);
  grads(dst_img,gradx,grady);
  cout<<"grey_width="<<widthU<<endl;
  cout<<"grey_height="<<heightV<<endl;
  ofstream write;
  write.open("/home/zhsyi/vision_servo_zsy/src/v_s/src/data_grey.txt");
  write <<dst_img;
  //write <<img;
  write.close();
}

//获得深度图的实际数据
void image_depthCallback(const sensor_msgs::ImageConstPtr &image_depth)
{
      //转换ROS图像消息到opencv图像
        cv::Mat depthimage;
        depthimage = cv_bridge::toCvShare(image_depth)->image;

        int widthU = depthimage.cols;				//图片宽度
        int heightV =depthimage.rows;				//图片长度
      //ROS_INFO("zhsyi heard: [%d],[%d]",widthU,heightV);

        cout<<"width="<<widthU<<endl;
        cout<<"height="<<heightV<<endl;
     /*   cv::Mat data(heightV,widthU,CV_32FC1);


        for (int i=0;i<heightV-1;i++)
        {
          for (int j=0;j<widthU-1;j++)
          {
            data[][]=depthimage[i][j];
          }
        }*/

        ofstream write;
        write.open("/home/zhsyi/vision_servo_zsy/src/v_s/src/data_depth.txt");
        write <<depthimage;
        write.close();



        vector<vector<double> > X_c(heightV-1, vector<double>(widthU-1));
        vector<vector<double> > Y_c(heightV-1, vector<double>(widthU-1));


        imshow(WINDOWS1,depthimage);
        waitKey(30);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "handle_image");
  ros::NodeHandle nh;
//cv::namedWindow("Show depth-greyImage");
//cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_greyimage = it.subscribe("/kinect2/qhd/image_depth_rect", 1, image_greyCallback);
  image_transport::Subscriber sub_depthdata = it.subscribe("/kinect2/qhd/image_depth_rect", 1, image_depthCallback);

  ros::spin();
//cv::destroyWindow("Show depth-greyImage");
  return 0;
}



