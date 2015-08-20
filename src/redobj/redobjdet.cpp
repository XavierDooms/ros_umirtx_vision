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
	Mat imgResult;
	
	int minmaxhsv[3][2] = {{100,140},{85,254},{128,264}};
	int status = 1;
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
	
	createControlWindow("Control", minmaxhsv, &status);
	
	
    if(cap.isOpened())
    {
		cap >> frame;
		
		if( frame.empty() )
			exit(0);
		
		//frame.copyTo( frameCopy );
		flip( frame, frameCopy, -1 );
			
		Mat imgLines = Mat::zeros( frameCopy.size(), CV_8UC3 );
		Mat imgResult= Mat::zeros( frameCopy.size(), CV_8UC3 );
		frameHeight = frame.rows;
		frameWidth = frame.cols;
		
        cout << "In capture ..." << endl;
        while(status>0)
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
			
			msg.x = 100*((double)iLastXY[0])/frameWidth;
			msg.y = 100*(double)iLastXY[1]/frameHeight;
			msg.area = 100*dArea/frameWidth/frameHeight;
			chatter_pub.publish(msg);
			
			cvtColor(imgThresholded,imgThresholded, CV_GRAY2RGB);
			addWeighted( frameCopy, 1, imgThresholded, 0.4, 0.0, imgResult);
			circle(imgResult,Point(iLastXY[0],iLastXY[1]),5,Scalar( 0, 0, 255 ),-1);
            imgResult = imgResult + imgLines;
            
			imshow("Result",imgResult);
			
			}
			catch(int e){
				cout << "Oopsie!" << endl;
			}
			
            if( waitKey( 10 ) >= 0 )
                //goto _cleanup_;
                cout << "Button pressed" << endl;
                //break;
            
            ros::spinOnce();
        }

        //waitKey(0);


//_cleanup_:
    }
}
catch(int e){
	cout << "Oopsie2!" << endl;
}
    
    cvDestroyWindow("Result");

    return 0;
}

