#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include "ncurses.h"
#include <math.h> 
#include <iostream>
// #include <transport_hints/transport_hints.h>

static const std::string OPENCV_WINDOW1 = "Blurred Image Window";
static const std::string OPENCV_WINDOW2 = "Thresholded Image";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher pub_vel;
  geometry_msgs::Twist cmd_stored;

  int iLastX, posX; 
  int iLastY, posY;
  double dArea, dLastArea;
  int go;

  int iLowH,iHighH,iLowS,iHighS,iLowV,iHighV;


public:
  ImageConverter()
    : it_(nh_)
  {
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::imageCb, this, image_transport::TransportHints("compressed"));
    pub_vel = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    cv::namedWindow(OPENCV_WINDOW1, CV_WINDOW_NORMAL);
    cv::namedWindow(OPENCV_WINDOW2, CV_WINDOW_NORMAL);

    iLowH =0;
    iHighH = 0;
    iLowS = 0; 
    iHighS = 0;
    iLowV = 0;
    iHighV = 0;

    go = false;

    cv::createTrackbar("LowH", OPENCV_WINDOW2, &iLowH, 179); //Hue (0 - 179)
    cv::createTrackbar("HighH", OPENCV_WINDOW2, &iHighH, 179);

    cv::createTrackbar("LowS", OPENCV_WINDOW2, &iLowS, 255); //Saturation (0 - 255)
    cv::createTrackbar("HighS", OPENCV_WINDOW2, &iHighS, 255);

    cv::createTrackbar("LowV", OPENCV_WINDOW2, &iLowV, 255);//Value (0 - 255)
    cv::createTrackbar("HighV", OPENCV_WINDOW2, &iHighV, 255);

    cv::createTrackbar("Go", OPENCV_WINDOW2, &go, 1);

  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW1);
    cv::destroyWindow(OPENCV_WINDOW2);
  }

float divisor(float number, int power)
{
  for (int i = 0; i < power; ++i)
  {
    number = number*0.1;
  }

  return number;

}

void movement()
{
  cmd_stored.linear.x = cmd_stored.angular.x = cmd_stored.linear.z = cmd_stored.angular.y = cmd_stored.angular.z = cmd_stored.linear.y = 0;

  int vector = posX - iLastX;
  double depth = dArea - dLastArea;
 // std::cout << "Value (vector, dArea): \n" << "("<< vector << ", "<< dArea << ")\n";

  const float constant_right = 0.0020089377;
  const float intercept_right = 0.1951411556;
  const float constant_left = -0.001715662;
  const float intercept_left = 0.2168756237;
  float angular_right = constant_right*vector + intercept_right;
  float angular_left = constant_left*vector + intercept_left;

  float linear_backward = divisor(-1.6331969,15)*dArea*dArea + divisor(5.085534,8)*dArea + 0.0975220864;
  float linear_forward = 0.070586926*log (dArea) - 0.5313718272;
  if(vector<-50)
  {
   cmd_stored.angular.z = 1.3*angular_left; //TURN LEFT
   std::cout << "angular left:  " << cmd_stored.angular.z << "\n"; 
  }

  else if(vector>50){
    cmd_stored.angular.z = -1.3*angular_right; //TURN RIGHT
    std::cout << "angular right:  " << cmd_stored.angular.z << "\n"; 
  }

  if(200000<dArea && dArea<1000000)
    {
      cmd_stored.linear.x = linear_forward;
      // std::cout << "test forward";
    }
  else if(dArea>2000000)
    {
      cmd_stored.linear.x = -linear_backward;
      // std::cout << "lin back:  " << -linear_backward;
    }

  // BUG IS HERE, YOU NEED ANOTHER METHOD TO SET SPEED TO 0
  else
    {
      cmd_stored.linear.x = 0;
      std::cout << "it fucked up";
    }

  pub_vel.publish(cmd_stored);
}
  
void imageCb(const sensor_msgs::ImageConstPtr& msg)
{

  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat imgHSV, threshold_Image;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
      
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

    cv::cvtColor(cv_ptr->image, imgHSV, cv::COLOR_BGR2HSV);
    cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), threshold_Image);
    cv::erode(threshold_Image, threshold_Image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(threshold_Image, threshold_Image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
    cv::dilate(threshold_Image, threshold_Image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
    cv::erode(threshold_Image, threshold_Image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

    cv::Moments oMoments = cv::moments(threshold_Image);

    double dM01 = oMoments.m01;
    double dM10 = oMoments.m10;
    dArea = oMoments.m00;

    if (dArea > 10)
      {
        posX = dM10 / dArea;
        posY = dM01 / dArea;
        int vector = posX - iLastX;
       
        if(go == 1)
          {
            movement();
          }
         // std::cout << "Value (vector, dArea): \n" << "("<< vector << ", "<< dArea << ")\n";
        iLastX = 331;;
        iLastY = 205;;
      }

    cv::circle(cv_ptr->image, cv::Point(posX, posY), 10, CV_RGB(255,0,0));
    cv::imshow(OPENCV_WINDOW2, threshold_Image);
    cv::imshow(OPENCV_WINDOW1, cv_ptr->image);
      
    cv::waitKey(3);
  }
};

int main(int argc, char** argv)
{
  // initscr();
  // cbreak();
  // noecho();
  // int ch = getch();
  while(ros::ok)
  {
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    // if(ch == 112)
      // endwin();
  }
  // exit(0);
  return 0;
}