//#include "ros/ros.h"
//#include "ros_umirtx_vision/VisPMsg.h"

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <sstream>
#include <stdio.h>

#include <string>


using namespace std;
using namespace cv;





int createControlWindow(String nameWindow, int minmaxhsv[][2]);
void selectRedObj(Mat &frameCopy, Mat &imgHSV, Mat &imgThresholded, int minmaxhsv[][2]);
void getCenterOfObj(Mat &imgIn, Mat &imgLines, int *iLastX, int *iLastY);
