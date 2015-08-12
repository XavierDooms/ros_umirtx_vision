#include "redobjdet.h"






int main( int argc, char **argv )
{
	
// %Tag(INIT)%
	ros::init(argc, argv, "visiontracker");
// %EndTag(INIT)%

// %Tag(NODEHANDLE)%
	ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

// %Tag(PUBLISHER)%
	ros::Publisher chatter_pub = n.advertise<ros_umirtx_vision::VisPMsg>("visionposition", 1000);
// %EndTag(PUBLISHER)%

	try{
		
	int c = 0 ;
    VideoCapture cap(c); // open the default camera
    Mat frame, frameCopy, image;
    Mat rgbimg[3], tempimg, prodimg;
	Mat imgThresholded, imgHSV;
	
	int minmaxhsv[3][2] = {{103,133},{108,254},{128,248}};
	int iLastXY[2] = {-1,-1};
	double dArea = 0;
	int frameHeight = 480, frameWidth = 640;
	double xpos = 0.5;
	double ypos = 0.5;
    
    ros_umirtx_vision::VisPMsg msg;
    
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
		frameHeight = frame.rows;
		frameWidth = frame.cols;
		
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
            
			getCenterOfObj(imgThresholded, imgLines, iLastXY, &dArea);
			
			//std::cout<<"X: "<<iLastXY[0]<<" Y: "<<iLastXY[1]<<" Area: "<<dArea<<std::endl;
			msg.x = 100*((double)iLastXY[0])/frameWidth;
			msg.y = 100*(double)iLastXY[1]/frameHeight;
			msg.area = 100*dArea/frameWidth/frameHeight;
			
			chatter_pub.publish(msg);
			
			
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
            
            ros::spinOnce();
        }

        waitKey(0);


//_cleanup_:
    }
}
catch(int e){
	cout << "Oopsie2!" << endl;
}
    
    cvDestroyWindow("result");

    return 0;
}

