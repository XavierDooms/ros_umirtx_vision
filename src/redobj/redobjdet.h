// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
#include "ros_umirtx_vision/VisPMsg.h"
// %EndTag(MSG_HEADER)%

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <sstream>
#include <iostream>
#include <stdio.h>

#include <string>


using namespace std;
using namespace cv;





int createControlWindow(String nameWindow, int minmaxhsv[][2], int *status);
void selectRedObj(Mat &frameCopy, Mat &imgHSV, Mat &imgThresholded, int minmaxhsv[][2]);
void getCenterOfObj(Mat &imgIn, Mat &imgLines, int iLastXY[2], double *dArea);
