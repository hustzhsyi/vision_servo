#include <visp3/core/vpCameraParameters.h>
 #include <visp3/core/vpImageConvert.h>
 #include <visp3/io/vpImageIo.h>
 #include <iostream>
 #if VISP_HAVE_OPENCV_VERSION >= 0x020300
 #include <opencv2/core/core.hpp>
 #include <opencv2/imgproc/imgproc.hpp>
 #endif
 using namespace std;
 int main()
 {
 #if VISP_HAVE_OPENCV_VERSION >= 0x020300
   double u0 = 326.6;
   double v0 = 215.0;
   double px = 582.7;
   double py = 580.6;
   double kud = -0.3372;
   double kdu = 0.4021;
   vpCameraParameters cam(px, py, u0, v0, kud, kdu);

 cout<<"相机参数："<<cam<<endl;

   cv::Mat K = (cv::Mat_<double>(3, 3) << cam.get_px(), 0, cam.get_u0(), 0, cam.get_py(), cam.get_v0(), 0, 0, 1);
   cv::Mat D = (cv::Mat_<double>(4, 1) << cam.get_kud(), 0, 0, 0);
 
   vpImage<unsigned char> I;
   vpImageIo::read(I, "1.jpg");
 
   cv::Mat image;
   vpImageConvert::convert(I, image);
   imshow("picture",image);
   cv::Mat imageUndistorted;
   cv::undistort(image, imageUndistorted, K, D);
 
   vpImage<unsigned char> IUndistorted;
   vpImageConvert::convert(imageUndistorted, IUndistorted);
 
   vpImageIo::write(IUndistorted, "chessboard-undistorted.pgm");
   cv::waitKey();
   return 0;
 #endif
 }
 
