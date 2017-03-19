#include <sstream>
#include <string>
#include <iostream>
//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 179;
int S_MIN = 0;
int S_MAX = 255;
int V_MIN = 0;
int V_MAX = 255;
const string windowName = "Original Image";
const string windowName1 = "HSV Image";

const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
static const std::string OPENCV_WINDOW = "Camera Feed";


//drawObject
//Written by  Kyle Hounslow 2013
void drawObject(int x, int y,Mat &frame, const Scalar &colour){

	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!

    //UPDATE:JUNE 18TH, 2013
    //added 'if' and 'else' statements to prevent
    //memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

	circle(frame,Point(x,y),20,colour,2);
    if(y-25>0)
    line(frame,Point(x,y),Point(x,y-25),colour,2);
    else line(frame,Point(x,y),Point(x,0),colour,2);
    if(y+25<FRAME_HEIGHT)
    line(frame,Point(x,y),Point(x,y+25),colour,2);
    else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),colour,2);
    if(x-25>0)
    line(frame,Point(x,y),Point(x-25,y),colour,2);
    else line(frame,Point(x,y),Point(0,y),colour,2);
    if(x+25<FRAME_WIDTH)
    line(frame,Point(x,y),Point(x+25,y),colour,2);
    else line(frame,Point(x,y),Point(FRAME_WIDTH,y),colour,2);

	std::ostringstream temp;
	temp<<x<<","<<y;	
	putText(frame,temp.str(),Point(x,y+30),1,1,Scalar(0,255,0),2);

}

//trackFilteredObject
//Written by  Kyle Hounslow 2013
void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed, const Scalar &colour){

	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
					x = moment.m10/area;
					y = moment.m01/area;
					objectFound = true;
					refArea = area;
				}else objectFound = false;


			}
			//let user know you found an object
			if(objectFound ==true){
				putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
				//draw object location on screen
				drawObject(x,y,cameraFeed, colour);}

		}else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
}

class Img_conv
{
	ros::NodeHandle nh;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub;
	image_transport::Publisher image_pub;

public:
	Img_conv()
		: it_(nh)
	{
		image_sub = it_.subscribe("/cv_camera/image_raw", 1, &Img_conv::imageCb, this);
		image_pub = it_.advertise("/image_converter/output_video", 1);
		cv::namedWindow(OPENCV_WINDOW);
	}

	~Img_conv()
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

       Mat camera;
       Mat HSV;
       camera = cv_ptr->image;
       Mat Yellow;
       Mat Red;
       int x=0,y=0;
       cvtColor(camera,HSV,COLOR_BGR2HSV);
       //cvtColor(camera,grey,COLOR_BGR2GRAY);
       inRange(HSV,Scalar(23,170,170),Scalar(38,S_MAX,V_MAX),Yellow);
       inRange(HSV,Scalar(160,170,170),Scalar(180,S_MAX,V_MAX),Red);
       trackFilteredObject(x,y,Yellow,camera,Scalar(0,255,255));
       trackFilteredObject(x,y,Red,camera,Scalar(0,0,255));
       imshow(windowName,camera);
       waitKey(30);

       image_pub.publish(cv_ptr->toImageMsg());
   }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "image_converter");
    Img_conv ic;
    ros::spin();
    return 0;
}