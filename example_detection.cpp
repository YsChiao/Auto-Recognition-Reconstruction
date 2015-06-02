//#include <iostream>
//#include <OpenNI.h>
//#include <NiTE.h>
//#include <opencv2\core\core.hpp>
//#include <opencv2\highgui\highgui.hpp>
//#include <opencv2\imgproc\imgproc.hpp>
//#include <openbr\openbr_plugin.h>
//#include "atre_user.h"
//
//
//
//using namespace std;
//using namespace cv;
//
//int main( int argc, char** argv )
//{
//	// OpenBR initialize
//	br::Context::initialize(argc, argv);
//	// Retrieve class for enrolling templates using the GenderEstimation algorithm
//	QSharedPointer<br::Transform> transformGender = br::Transform::fromAlgorithm("GenderEstimation");
//	QSharedPointer<br::Transform> transformAge = br::Transform::fromAlgorithm("AgeEstimation");
//	// Gender and Age
//	QString UserGender;
//	int UserAge = 0;
//
//	// OpenNI2 image
//	openni::VideoFrameRef oniDepthImg;
//	openni::VideoFrameRef oniColorImg;
//
//	// OpenCV image
//	Mat cvDepthImg;
//	Mat cvBGRImg;
//
//	// initialize OpenNI2
//	openni::OpenNI::initialize();
//
//	// open device
//	openni::Device device;
//	device.open( openni::ANY_DEVICE);
//
//	// create depth stream
//	openni::VideoStream oniDepthStream;
//	oniDepthStream.create( device, openni::SENSOR_DEPTH );
//	// set depth video mode
//	openni::VideoMode modeDepth;
//	modeDepth.setResolution( 640, 480);
//	modeDepth.setFps( 30 );
//	modeDepth.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
//	oniDepthStream.setVideoMode( modeDepth );
//	// start depth stream
//	oniDepthStream.start();
//
//	// create color stream  
//	openni::VideoStream oniColorStream;  
//	oniColorStream.create( device, openni::SENSOR_COLOR );  
//	// set color video mode  
//	openni::VideoMode modeColor;  
//	modeColor.setResolution( 640, 480 );  
//	modeColor.setFps( 30 );  
//	modeColor.setPixelFormat( openni::PIXEL_FORMAT_RGB888 );  
//	oniColorStream.setVideoMode( modeColor);  
//	// set detph and color image registration mode
//	if( device.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR) )
//	{
//		device.setImageRegistrationMode( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );
//	}
//	// start color stream
//	oniColorStream.start();
//
//	// NITE initialize 
//	nite::NiTE::initialize();
//
//	nite::UserTracker mUserTracker;
//	mUserTracker.create();
//
//	nite::UserTrackerFrameRef mUserFrame;
//	int count = 0; 
//
//	while(1)
//	{
//		mUserTracker.readFrame( &mUserFrame );
//		oniDepthImg = mUserFrame.getDepthFrame();
//
//		// Color Image
//		oniColorStream.readFrame(&oniColorImg);
//		Mat mImageRGB( oniColorImg.getHeight(), oniColorImg.getWidth(), CV_8UC3, (void*)oniColorImg.getData() );
//		cvtColor(mImageRGB, mImageRGB, CV_RGB2BGR);
//
//		// get user map and user list
//		const nite::UserMap& rUserMap = mUserFrame.getUserMap();
//		const nite::Array<nite::UserData>& rUsers = mUserFrame.getUsers();
//		int size = rUsers.getSize();
//		cv::vector<atre_user> Users(size);
//
//		for( int i = 0; i < size; ++ i )
//		{
//			Users[i].loadUser(rUsers[i]);
//		}
//
//		// user detection
//		for( int i = 0; i < size; ++ i )
//		{
//			if( Users[i].isNew() == 0 )
//			{
//				printf("New User [%d] found.\r\n", Users[i].getId());
//				mUserTracker.startSkeletonTracking( Users[i].getId());
//				mUserTracker.startPoseDetection( Users[i].getId(), nite::POSE_PSI);
//			}
//
//			if( Users[i].isVisible() == 0 )
//			{
//				// detect pose of user
//				const nite::PoseData& uPosePSI = Users[i].getPose(nite::POSE_PSI);
//				if (uPosePSI.isHeld())
//				{
//					if (count == 0)
//					{
//						// detect gender and age
//						Users[i].loadImage(oniColorImg);
//						Users[i].getGenderAge(argc, argv, transformGender, transformAge, UserGender, UserAge);
//						printf("Gender =  %s\n", qPrintable(UserGender));
//						printf("Age = %d\n", UserAge);
//
//						// detect arm
//						double leftArmLength, rightArmLength;
//						Users[i].getArmLength(leftArmLength,rightArmLength);
//						printf("LeftArmLength = %f\r\n", leftArmLength);
//						printf("RightArmLength = %f\r\n", rightArmLength);
//
//						// detect floor
//						double entropy = -120;
//						nite::Plane floorCoords;
//						Users[i].getFloor(mUserFrame, floorCoords);
//		
//						// get the length of leg
//						double leftLegLength, rightLegLength;
//						Users[i].getLegLength(floorCoords, leftLegLength, rightLegLength, entropy);
//						printf("LeftLegLength = %f\r\n", leftLegLength);
//						printf("RightLegLength = %f\r\n", rightLegLength);
//
//						// get height 
//						double height = 0;
//						Users[i].getHeight(oniDepthStream, oniDepthImg, rUserMap, floorCoords, height, entropy);
//						printf("Height = %f\r\n", height);
//					}
//					count ++;
//				}
//
//				const nite::Skeleton& rSkeleton = Users[i].getSkeleton();
//				if( rSkeleton.getState() == nite::SKELETON_TRACKED )
//				{
//
//					Users[i].skeletonTracking(mUserTracker, mImageRGB);
//				}
//			}
//
//			if( Users[i].isLost() == 0 )
//			{
//				printf("User [%d] lost.\r\n\n", Users[i].getId());
//				count=0;
//			}
//		}
//		cvBGRImg = mImageRGB;
//		imshow("Main Window", cvBGRImg);
//		waitKey(10);
//
//	}
//
//	//// display depth image
//	//if (oniDepthImg.isValid())
//	//{
//	//	cv::Mat cvRawImg16U( oniDepthImg.getHeight(), oniDepthImg.getWidth(), CV_16UC1, (void*)oniDepthImg.getData() );  
//	//	cvRawImg16U.convertTo( cvDepthImg, CV_8U, 255.0/(oniDepthStream.getMaxPixelValue()));  
//	//	// convert depth image GRAY to BGR  
//	//	//cv::cvtColor(cvDepthImg,cvFusionImg,CV_GRAY2BGR);  
//	//	cv::imshow( "depth", cvDepthImg ); 
//	//	waitKey(20);
//	//}
//
//	//destroyWindow("depth");
//	destroyWindow("Main Window");
//	mUserFrame.release();      
//	mUserTracker.destroy();
//	device.close();
//	openni::OpenNI::shutdown();
//	nite::NiTE::shutdown();
//
//	return 0;
//}
