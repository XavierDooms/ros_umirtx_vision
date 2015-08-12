#include "redobjdet.h"


int createControlWindow(String nameWindow, int minmaxhsv[][2]){
	
	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	
	//Create trackbars in "Control" window
	cvCreateTrackbar("LowH", "Control", &minmaxhsv[0][0], 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &minmaxhsv[0][1], 179);

	cvCreateTrackbar("LowS", "Control", &minmaxhsv[1][0], 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &minmaxhsv[1][1], 255);

	cvCreateTrackbar("LowV", "Control", &minmaxhsv[2][0], 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &minmaxhsv[2][1], 255);
	
	return 1;
}



void selectRedObj(Mat &frameCopy, Mat &imgHSV, Mat &imgThresholded, int minmaxhsv[][2]){
	
	cvtColor(frameCopy, imgHSV, COLOR_RGB2HSV); //Convert the captured frame from BGR to HSV  (RGB -> Hue offset)
	inRange(imgHSV, Scalar(minmaxhsv[0][0], minmaxhsv[1][0], minmaxhsv[2][0]), Scalar(minmaxhsv[0][1], minmaxhsv[1][1], minmaxhsv[2][1]), imgThresholded); //Threshold the image
	
	//morphological opening (remove small objects from the foreground)
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

	//morphological closing (fill small holes in the foreground)
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
            
	
	
	
	
	
	
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
	
	int iLastX = iLastXY[0];
	
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
			line(imgLines, Point(posX, posY), Point(iLastXY[0], iLastXY[1]), Scalar(0,0,255), 2);
		}

		iLastXY[0] = posX;
		iLastXY[1] = posY;
	}
	
	
}
