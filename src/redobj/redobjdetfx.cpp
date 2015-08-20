#include "redobjdet.h"


int createControlWindow(String nameWindow, int minmaxhsv[][2], int *status){
	
	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	
	//Create trackbars in "Control" window
	cvCreateTrackbar("LowH", "Control", &minmaxhsv[0][0], 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &minmaxhsv[0][1], 179);

	cvCreateTrackbar("LowS", "Control", &minmaxhsv[1][0], 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &minmaxhsv[1][1], 255);

	cvCreateTrackbar("LowV", "Control", &minmaxhsv[2][0], 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &minmaxhsv[2][1], 255);
	
	cvCreateTrackbar("Open", "Control", status, 1);
	
	return 1;
}



void selectRedObj(Mat &frameCopy, Mat &imgHSV, Mat &imgThresholded, int minmaxhsv[][2]){
	
	cvtColor(frameCopy, imgHSV, COLOR_RGB2HSV); //Convert the captured frame from BGR to HSV  (RGB -> Hue offset)
	inRange(imgHSV, Scalar(minmaxhsv[0][0], minmaxhsv[1][0], minmaxhsv[2][0]), Scalar(minmaxhsv[0][1], minmaxhsv[1][1], minmaxhsv[2][1]), imgThresholded); //Threshold the image
	
	Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
	//morphological opening (remove small objects from the foreground)
	erode(imgThresholded, imgThresholded,  kernel );
	dilate( imgThresholded, imgThresholded, kernel ); 
	
	kernel = getStructuringElement(MORPH_ELLIPSE, Size(7, 7));
	//morphological closing (fill small holes in the foreground)
	dilate( imgThresholded, imgThresholded, kernel ); 
	erode(imgThresholded, imgThresholded, kernel );
            
	
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Mat imgTemp = Mat::zeros( imgThresholded.size(), CV_8UC1 );;
	Mat imgTemp2;
	imgThresholded.copyTo( imgTemp2 );
	
	findContours( imgTemp2, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	
	int largestIndex = 0;
	int largestContour = 0;
	for(int i = 0; i<contours.size(); i++)
	{
		//cout << " s: "<<contours[i].size();
		if(contours[i].size() > largestContour){
			largestContour = contours[i].size();
			largestIndex = i;
		}
	}
	//cout<<endl;
	//cout <<"LC: "<<largestContour<<" Numb: "<<contours.size()<<endl;
	
	if(largestContour > 0)
	{
		drawContours( imgTemp, contours, largestIndex, Scalar(255), CV_FILLED, 8, hierarchy, 0, Point() );
	}
	imgTemp.copyTo( imgThresholded );
	
	//split(frameCopy,rgbimg);
		
	//subtract(255,rgbimg[0],tempimg);
	//bitwise_not(rgbimg[0],tempimg);
	//multiply(rgbimg[2], tempimg, tempimg);
	//subtract(1,rgbimg[1],prodimg);
	//multiply(tempimg  , prodimg, prodimg);
	
	//divide(rgbimg[2], rgbimg[0], prodimg);
	//divide(prodimg  , rgbimg[1], prodimg);
	
	//equalizeHist( prodimg, prodimg );
	
	//imshow( "Control", imgThresholded);
	
	
}


void getCenterOfObj(Mat &imgIn, Mat &imgLines, int iLastXY[2], double *dArea){
	
	//int iLastX = iLastXY[0];
	
	
	//Calculate the moments of the thresholded image
	Moments oMoments = moments(imgIn);
	
	double dM01 = oMoments.m01;
	double dM10 = oMoments.m10;
	*dArea = oMoments.m00;
	
	
	// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
	if (*dArea > 10000)
	{
		//calculate the position of the ball
		int posX = dM10 / *dArea;
		int posY = dM01 / *dArea;        

		if (iLastXY[0] >= 0 && iLastXY[1] >= 0 && posX >= 0 && posY >= 0)
		{
			//Draw a red line from the previous point to the current point
			line(imgLines, Point(posX, posY), Point(iLastXY[0], iLastXY[1]), Scalar(0,255,0), 2);
		}

		iLastXY[0] = posX;
		iLastXY[1] = posY;
	}
	
	
}
