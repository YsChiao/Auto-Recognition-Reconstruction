#include <iostream>

#include <NiTE.h>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <openbr\openbr_plugin.h>
#include "atre_user.h"


using namespace std;
using namespace cv;


int main (int argc, char** argv)
{
	// OpenNI2 image
	openni::VideoFrameRef oniDepthImg;
	openni::VideoFrameRef oniColorImg;

	// OpenCV image
	Mat cvDepthImg;
	Mat cvBGRImg;

	// initialize OpenNI2
	openni::OpenNI::initialize();

	// open device
	openni::Device device;
	device.open( openni::ANY_DEVICE);

	// create depth stream
	openni::VideoStream oniDepthStream;
	oniDepthStream.create( device, openni::SENSOR_DEPTH );
	// set depth video mode
	openni::VideoMode modeDepth;
	modeDepth.setResolution( 640, 480);
	modeDepth.setFps( 30 );
	modeDepth.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
	oniDepthStream.setVideoMode( modeDepth );
	// start depth stream
	oniDepthStream.start();

	// create color stream  
	openni::VideoStream oniColorStream;  
	oniColorStream.create( device, openni::SENSOR_COLOR );  
	// set color video mode  
	openni::VideoMode modeColor;  
	modeColor.setResolution( 640, 480 );  
	modeColor.setFps( 30 );  
	modeColor.setPixelFormat( openni::PIXEL_FORMAT_RGB888 );  
	oniColorStream.setVideoMode( modeColor);  
	// set detph and color image registration mode
	if( device.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR) )
	{
		device.setImageRegistrationMode( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );
	}
	// start color stream
	oniColorStream.start();

	// NITE initialize 
	nite::NiTE::initialize();

	nite::UserTracker mUserTracker;
	mUserTracker.create();

	nite::UserTrackerFrameRef mUserFrame;
	int count = 0; 

	while(1)
	{
		mUserTracker.readFrame( &mUserFrame );
		oniDepthImg = mUserFrame.getDepthFrame();

		// Color Image
		oniColorStream.readFrame(&oniColorImg);
		Mat mImageRGB( oniColorImg.getHeight(), oniColorImg.getWidth(), CV_8UC3, (void*)oniColorImg.getData() );
		cvtColor(mImageRGB, mImageRGB, CV_RGB2BGR);

		// get user map and user list
		const nite::UserMap& rUserMap = mUserFrame.getUserMap();
		const nite::Array<nite::UserData>& rUsers = mUserFrame.getUsers();
		int size = rUsers.getSize();
		cv::vector<atre_user> Users(size);

		for( int i = 0; i < size; ++ i )
		{
			Users[i].loadUser(rUsers[i]);
		}

		// user detection
		for( int i = 0; i < size; ++ i )
		{
			if( Users[i].isNew() == 0 )
			{
				printf("New User [%d] found.\r\n", Users[i].getId());
				mUserTracker.startSkeletonTracking( Users[i].getId());
				mUserTracker.startPoseDetection( Users[i].getId(), nite::POSE_PSI);
			}

			if( Users[i].isVisible() == 0 )
			{
				const nite::Skeleton& rSkeleton = Users[i].getSkeleton();
				if( rSkeleton.getState() == nite::SKELETON_TRACKED )
				{

					//Users[i].skeletonTracking(mUserTracker, mImageRGB);
					time_t now = time(0);
					stringstream ss;
					ss << now;
					string ts = "REME_" + ss.str() + ".PLY"; 
					Users[i].reconstructMe(oniDepthStream, oniDepthImg, ts);

					double hip = 0;
					double torso = 0;
					double neck = 0;
					// detect the circumference of bust, waist, hip
					double hip_cir = 0;
					double waist_cir = 0;
					double bust_cir = 0;

					Users[i].getGirth(ts, hip, hip_cir);
					Users[i].getGirth(ts, torso+30, waist_cir);
					Users[i].getGirth(ts, torso, bust_cir);

					// detect the shape of user
					double shape = waist_cir/hip_cir;

					// detect the volume of user
					double weight = 0;
					Users[i].reme_filter(ts, weight);

					system("cls");
					printf("HIP    = %d\r\n", hip_cir*3); // 1 unit equals 3 mm;
					printf("WAIST  = %d\r\n", waist_cir*3);
					printf("BUST   = %d\r\n", bust_cir*3);
					printf("WEIGHT = %d Kg\r\n", weight);
					printf("WHR value of the user = %d\r\n", shape);
				}
			}

			if( Users[i].isLost() == 0 )
			{
				printf("User [%d] lost.\r\n\n", Users[i].getId());
				count=0;
			}
		}
		cvBGRImg = mImageRGB;
		imshow("Main Window", cvBGRImg);
		waitKey(10);

	}

	//destroyWindow("depth");
	destroyWindow("Main Window");
	mUserFrame.release();      
	mUserTracker.destroy();
	device.close();
	openni::OpenNI::shutdown();
	nite::NiTE::shutdown();

	return 0;
}