#include "redobjdet.h"






int main( int argc, const char** argv )
{
	//ros::init(argc, argv, "talker");
	try{
    //CvCapture* capture = 0;
    VideoCapture cap(0); // open the default camera
    
    Mat frame, frameCopy, image;
    Mat rgbimg[3], tempimg, prodimg;
    
	Mat imgThresholded, imgHSV;
	
	int minmaxhsv[3][2] = {{103,133},{108,254},{128,248}};
	
	int iLastX = -1; 
	int iLastY = -1;
    
    //capture = cvCaptureFromCAM( 0 );
    int c = 0 ;
    if(!cap.isOpened()) {
		cout << "Capture from CAM " <<  c << " didn't work" << endl;
		return -1;
	}
	
	createControlWindow("Control", minmaxhsv);
	
	
    if(cap.isOpened())
    {
		cap >> frame;
		
		if( frame.empty() )
			exit(0);
		
		//frame.copyTo( frameCopy );
		flip( frame, frameCopy, -1 );
			
		Mat imgLines = Mat::zeros( frameCopy.size(), CV_8UC3 );
		
        cout << "In capture ..." << endl;
        for(;;)
        {
			try{
            cap >> frame;
            if( frame.empty() )
                break;
            
            //frame.copyTo( frameCopy );
            flip( frame, frameCopy, -1 );
            //std::cout << "H:" << frameCopy.rows << " W:" << frameCopy.cols << std::endl;
            
            
			selectRedObj(frameCopy, imgHSV, imgThresholded, minmaxhsv);
            
			getCenterOfObj(imgThresholded, imgLines, &iLastX, &iLastY);
			
            imshow("Thresholded Image", imgThresholded); //show the thresholded image
            frameCopy = frameCopy + imgLines;
			imshow("Original", frameCopy); //show the original image
			
			}
			catch(int e){
				cout << "Oopsie!" << endl;
			}
			
            if( waitKey( 10 ) >= 0 )
                //goto _cleanup_;
                break;
        }

        waitKey(0);


//_cleanup_:
        //cvReleaseCapture( &capture );
    }
}
catch(int e){
	cout << "Oopsie2!" << endl;
}
    
    cvDestroyWindow("result");

    return 0;
}

