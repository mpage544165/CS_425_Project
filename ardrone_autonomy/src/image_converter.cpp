#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/tracking.hpp>

static const std::string OPENCV_WINDOW = "Image window";

static bool initialized = false;

//known measurements in inches
static int const KNOWN_DISTANCE = 157;
static int const KNOWN_WIDTH = 39;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  cv::Rect2d roi;
  cv::Ptr<cv::Tracker> tracker = cv::TrackerKCF::create();

  bool imageCaptured;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/ardrone/front/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);

  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  cv::RotatedRect findMarker(cv::Mat image){
    cv::Mat gray;
    cv::cvtColor(image, gray, CV_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
    cv::Mat edges;
    cv::Canny(gray, edges, 35, 125);

    cv::imshow("edges", edges);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(edges, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

    return cv::minAreaRect(contours[0]);
  }

  int distanceToCamera(int knownWidth, int focalLength, int perWidth){
    return (knownWidth * focalLength) / perWidth;
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

    if(!initialized){
        //cap >> frame;
        roi = selectROI("tracker", cv_ptr->image);

        tracker->init(cv_ptr->image, roi);

        initialized = true;
    }

    if(!imageCaptured){
        cv::imwrite("ref_image.png", cv_ptr->image);
        imageCaptured = true;

    }

    //calibrate camera
    cv::Mat im = cv::imread("ref_image.png");
    cv::RotatedRect rect = findMarker(cv_ptr->image);
    int focalLength = (rect.size.width * KNOWN_DISTANCE) / KNOWN_WIDTH;

    cv::imshow("image", cv_ptr->image);

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));


    // update the tracking result
    tracker->update(cv_ptr->image, roi);

    // draw the tracked object
    cv::rectangle(cv_ptr->image, roi, cv::Scalar( 255, 0, 0 ), 2, 1 );

    // show image with the tracked object
    //imshow("tracker", cv_ptr->image);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
