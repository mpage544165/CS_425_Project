
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <gtk/gtk.h>


//static const std::string OPENCV_WINDOW = "Image window";

static bool initialized = false;

class MainWindow
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

public:
  MainWindow()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/image_converter/output_video", 1,
      &MainWindow::showFrame, this);

    //cv::namedWindow(OPENCV_WINDOW);

  }

  ~MainWindow()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void showFrame(const cv_bridge::CvImagePtr& msg)
  {
    /*cv_bridge::CvImagePtr cv_ptr;
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

        initialized = true;
    }*/

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // draw the tracked object
    cv::rectangle(cv_ptr->image, roi, cv::Scalar( 255, 0, 0 ), 2, 1 );

    // show image with the tracked object
    //imshow("tracker", cv_ptr->image);

    // Update GUI Window
    GtkWidget *video;
    video = 

  }
};

static void activate (GtkApplication *app, gpointer user_data)
{
  GtkWidget *window;

  window = gtk_application_window_new (app);
  gtk_window_set_title (GTK_WINDOW (window), "Window");
  gtk_window_set_default_size (GTK_WINDOW (window), 1200, 720);

  gtk_widget_show_all (window);
}

int main(int argc, char** argv)
{
  GtkApplication *app;
  int status;

  app = gtk_application_new ("org.gtk.example", G_APPLICATION_FLAGS_NONE);
  g_signal_connect (app, "activate", G_CALLBACK (activate), NULL);
  status = g_application_run (G_APPLICATION (app), argc, argv);
  g_object_unref (app);

  ros::init(argc, argv, "main_window");
  MainWindow win;
  ros::spin();
  return 0;
}
