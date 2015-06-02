#include "atre_user.h"
#include <string>
#include <algorithm> // std::min_element, std::max_element
#include <stdlib.h>
#include <NiTE.h>

#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <reconstructmesdk\reme.h>

// reme_filter.h
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <math.h>

// girth.h



using namespace std;
using namespace cv;

// internal function 
double Length(nite::Point3f p1, nite::Point3f p2)
{
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

// A simple macro that checks for any errors and exits if a method invocation failed.
#define OK(s)                                       \
	if (s != REME_ERROR_SUCCESS) {                  \
	reme_context_print_errors(c);                   \
	reme_context_destroy(&c);                       \
	exit(-1);                                       \
	}                                               \


atre_user::atre_user(void)
{
}

atre_user::~atre_user(void)
{
}



int atre_user::loadUser(nite::UserData rUser)
{
	aUser = rUser;
	aUserId = aUser.getId();
	aUserSkeleton = aUser.getSkeleton();
	return 0;
}



nite::UserId atre_user::getId()
{
	return aUserId;
}

nite::Skeleton atre_user::getSkeleton()
{
	return aUserSkeleton;
}

int atre_user::isNew()
{
	if(aUser.isNew())
	{
		return 0;
	}
	else
		return 1;
}

int atre_user::isLost()
{
	if(aUser.isLost())
	{
		return 0;
	}
	else
		return 1;
}

int atre_user::isVisible()
{
	if(aUser.isVisible())
	{
		return 0;
	}
	else
		return 1;
}


int atre_user::loadImage(openni::VideoFrameRef oniColorImg)
{
	cv::Mat cvBGRImg;
	cv::Mat cvRGBImg( oniColorImg.getHeight(), oniColorImg.getWidth(), CV_8UC3, (void*)oniColorImg.getData() );
	cv::cvtColor( cvRGBImg, cvBGRImg, CV_RGB2BGR );
	this->aUserImage = cvBGRImg;
	return 0;
}

int atre_user::showImage()
{
	cv::namedWindow("User's color image");
	cv::imshow("User's color image", this->aUserImage);
	cv::waitKey(10);
	cv::destroyWindow("User's color image");
	return 0;
}

int atre_user::getGenderAge(int argc, char *argv[], QSharedPointer<br::Transform> transformGender, QSharedPointer<br::Transform> transformAge, QString &Gender, int &Age)
{
	// initialize templates
	br::Template queryGender(aUserImage);
	br::Template queryAge(aUserImage);

	// enroll templates
	queryGender >> *transformGender;
	queryAge >> *transformAge;

	//printf("%s gender: %s\n", qPrintable(queryGender.file.fileName()), qPrintable(queryGender.file.get<QString>("Gender")));
	aUserGender = qPrintable(queryGender.file.get<QString>("Gender"));
	Gender = qPrintable(queryGender.file.get<QString>("Gender"));

	//printf("%s age: %d\n", qPrintable(queryAge.file.fileName()), int(queryAge.file.get<float>("Age")));
	aUserAge =  int(queryAge.file.get<float>("Age"));
	Age = int(queryAge.file.get<float>("Age"));

	return 0;

}

int atre_user::getArmLength(double& leftArmLength, double& rightArmLength)
{
	double leftArmLength1, leftArmLength2;

	leftArmLength1 = Length(aUserSkeleton.getJoint( nite::JOINT_LEFT_SHOULDER).getPosition(), aUserSkeleton.getJoint( nite::JOINT_LEFT_ELBOW).getPosition() ); 
	leftArmLength2 = Length(aUserSkeleton.getJoint( nite::JOINT_LEFT_ELBOW).getPosition(), aUserSkeleton.getJoint( nite::JOINT_LEFT_HAND).getPosition() );
	leftArmLength = leftArmLength1 + leftArmLength2;

	double rightArmLength1, rightArmLength2;
	rightArmLength1 = Length( aUserSkeleton.getJoint( nite::JOINT_RIGHT_SHOULDER).getPosition(), aUserSkeleton.getJoint( nite::JOINT_RIGHT_ELBOW).getPosition() );
	rightArmLength2 = Length( aUserSkeleton.getJoint( nite::JOINT_RIGHT_ELBOW).getPosition(), aUserSkeleton.getJoint( nite::JOINT_RIGHT_HAND).getPosition() );
	rightArmLength = rightArmLength1 + rightArmLength2;

	return 0;
}

int atre_user::getLegLength(nite::Plane& floor, double& leftLegLength, double& rightLegLength, double entropy)
{
	leftLegLength  = abs(aUserSkeleton.getJoint(nite::JOINT_LEFT_HIP).getPosition().y -  floor.point.y) + entropy;
	rightLegLength = abs(aUserSkeleton.getJoint(nite::JOINT_RIGHT_HIP).getPosition().y - floor.point.y) + entropy;
	//printf("%f %f\r\n",aUserSkeleton.getJoint(nite::JOINT_LEFT_HIP).getPosition().y, aUserSkeleton.getJoint(nite::JOINT_RIGHT_HIP).getPosition().y);

	return 0;
}


int atre_user::getPoition(string STRING, double& POS)
{
	nite::SkeletonJoint aJoints[15];
	aJoints[ 0] = aUserSkeleton.getJoint( nite::JOINT_HEAD );
	aJoints[ 1] = aUserSkeleton.getJoint( nite::JOINT_NECK );
	aJoints[ 2] = aUserSkeleton.getJoint( nite::JOINT_LEFT_SHOULDER );
	aJoints[ 3] = aUserSkeleton.getJoint( nite::JOINT_RIGHT_SHOULDER );
	aJoints[ 4] = aUserSkeleton.getJoint( nite::JOINT_LEFT_ELBOW );
	aJoints[ 5] = aUserSkeleton.getJoint( nite::JOINT_RIGHT_ELBOW );
	aJoints[ 6] = aUserSkeleton.getJoint( nite::JOINT_LEFT_HAND );
	aJoints[ 7] = aUserSkeleton.getJoint( nite::JOINT_RIGHT_HAND );
	aJoints[ 8] = aUserSkeleton.getJoint( nite::JOINT_TORSO );
	aJoints[ 9] = aUserSkeleton.getJoint( nite::JOINT_LEFT_HIP );
	aJoints[10] = aUserSkeleton.getJoint( nite::JOINT_RIGHT_HIP );
	aJoints[11] = aUserSkeleton.getJoint( nite::JOINT_LEFT_KNEE );
	aJoints[12] = aUserSkeleton.getJoint( nite::JOINT_RIGHT_KNEE );
	aJoints[13] = aUserSkeleton.getJoint( nite::JOINT_LEFT_FOOT );
	aJoints[14] = aUserSkeleton.getJoint( nite::JOINT_RIGHT_FOOT );

	// get position
	if (STRING.compare("HEAD")== 0) 
	{
		const nite::Point3f& position = aJoints[ 0].getPosition();
		POS = position.y;
		return 0;
	} 
	else if ( STRING.compare("NECK")== 0)
	{
		const nite::Point3f& position = aJoints[ 1].getPosition();
		POS = position.y;
		return 0;
	}
	else if ( STRING.compare("LEFT_SHOULDER")== 0)
	{
		const nite::Point3f& position = aJoints[ 2].getPosition();
		POS = position.y;
		return 0;
	}
	else if ( STRING.compare("RIGHT_SHOULDER")== 0)
	{
		const nite::Point3f& position = aJoints[ 3].getPosition();
		POS = position.y;
		return 0;
	}
	else if ( STRING.compare("TORSO")== 0)
	{
		const nite::Point3f& position = aJoints[ 8].getPosition();
		POS = position.y;
		return 0;
	}
	else if ( STRING.compare("LEFT_HIP")== 0)
	{
		const nite::Point3f& position = aJoints[ 9].getPosition();
		POS = position.y;
		return 0;
	}
	else if ( STRING.compare("RIGHT_HIP")== 0)
	{
		const nite::Point3f& position = aJoints[10].getPosition();
		POS = position.y;
		return 0;
	}
		else if ( STRING.compare("LEFT_KNEE")== 0)
	{
		const nite::Point3f& position = aJoints[11].getPosition();
		POS = position.y;
		return 0;
	}
		else if ( STRING.compare("RIGHT_KNEE")== 0)
	{
		const nite::Point3f& position = aJoints[12].getPosition();
		POS = position.y;
		return 0;
	}
	else /* default: */
	{
		POS = 0;
		return 0;
	}
}

int atre_user::getHeight(openni::VideoStream& oniDepthStream, openni::VideoFrameRef& oniDepthImg, nite::UserMap rUserMap, nite::Plane& floor, double& height, double entropy)
{

	const nite::UserId* pUserMapData = rUserMap.getPixels(); 
	// draw user pixles
	int count = 0;

	for ( int y = 0; y < rUserMap.getHeight(); ++y )
	{
		for (int x = 0; x < rUserMap.getWidth(); ++x )
		{
			const nite::UserId& rUserID = pUserMapData[ x + y * rUserMap.getWidth() ];
			if( rUserID !=0 )
			{
				if ( count == 0)
				{
					const openni::DepthPixel* pDepthArray = (const openni::DepthPixel*)oniDepthImg.getData();
					int idx = x + y * oniDepthImg.getWidth();
					const openni::DepthPixel& rDepth = pDepthArray[idx];
					float fX, fY, fZ;
					openni::CoordinateConverter::convertDepthToWorld( oniDepthStream, x, y, rDepth, &fX, &fY, &fZ );
					height = abs( fY - floor.point.y ) + entropy;
					count ++;
				}
				else
				{
					return 0;
				}
			}
		}
	}
	return 0;
}

nite::PoseData atre_user::getPose(nite::PoseType )
{
	const nite::PoseData& rPosePSI = aUser.getPose(nite::POSE_PSI);
	return rPosePSI;
}

int atre_user::getFloor(nite::UserTrackerFrameRef& mUserFrame, nite::Plane& floorCoords)
{
	floorCoords = mUserFrame.getFloor();
	return 0;
}


int atre_user::skeletonTracking(nite::UserTracker& mUserTracker, cv::Mat& mImageRGB)
{
	// draw BoundingBox line
	nite::BoundingBox UserBB = aUser.getBoundingBox();
	float minPosX = UserBB.min.x;
	float maxPosX = UserBB.max.x;
	float minPosY = UserBB.min.y;
	float maxPosY = UserBB.max.y;

	cv::Point2f a(minPosX, minPosY), b(maxPosX, minPosY), c(minPosX, maxPosY), d(maxPosX, maxPosY);
	line( mImageRGB, a, b, cv::Scalar(255,0,0), 3 );
	line( mImageRGB, b, d, cv::Scalar(255,0,0), 3 );
	line( mImageRGB, d, c, cv::Scalar(255,0,0), 3 );
	line( mImageRGB, c, a, cv::Scalar(255,0,0), 3 );


	// build joints array
	nite::SkeletonJoint aJoints[15];
	aJoints[ 0] = aUserSkeleton.getJoint( nite::JOINT_HEAD);
	aJoints[ 1] = aUserSkeleton.getJoint( nite::JOINT_NECK);
	aJoints[ 2] = aUserSkeleton.getJoint( nite::JOINT_LEFT_SHOULDER);
	aJoints[ 3] = aUserSkeleton.getJoint( nite::JOINT_RIGHT_SHOULDER );
	aJoints[ 4] = aUserSkeleton.getJoint( nite::JOINT_LEFT_ELBOW );
	aJoints[ 5] = aUserSkeleton.getJoint( nite::JOINT_RIGHT_ELBOW );
	aJoints[ 6] = aUserSkeleton.getJoint( nite::JOINT_LEFT_HAND );
	aJoints[ 7] = aUserSkeleton.getJoint( nite::JOINT_RIGHT_HAND );
	aJoints[ 8] = aUserSkeleton.getJoint( nite::JOINT_TORSO );
	aJoints[ 9] = aUserSkeleton.getJoint( nite::JOINT_LEFT_HIP );
	aJoints[10] = aUserSkeleton.getJoint( nite::JOINT_RIGHT_HIP );
	aJoints[11] = aUserSkeleton.getJoint( nite::JOINT_LEFT_KNEE );
	aJoints[12] = aUserSkeleton.getJoint( nite::JOINT_RIGHT_KNEE );
	aJoints[13] = aUserSkeleton.getJoint( nite::JOINT_LEFT_FOOT );
	aJoints[14] = aUserSkeleton.getJoint( nite::JOINT_RIGHT_FOOT );

	// convet joint position to image
	cv::Point2f aPoints[15];
	for ( int s = 0; s < 15; ++s )
	{
		const nite::Point3f rPos = aJoints[s].getPosition();
		mUserTracker.convertJointCoordinatesToDepth( rPos.x, rPos.y, rPos.z, &(aPoints[s].x), &(aPoints[s].y) );
	}
	// draw line
	cv::line( mImageRGB, aPoints[ 0], aPoints[ 1], cv::Scalar( 255, 0, 0 ), 3 );
	cv::line( mImageRGB, aPoints[ 1], aPoints[ 2], cv::Scalar( 255, 0, 0 ), 3 );
	cv::line( mImageRGB, aPoints[ 1], aPoints[ 3], cv::Scalar( 255, 0, 0 ), 3 );
	cv::line( mImageRGB, aPoints[ 2], aPoints[ 4], cv::Scalar( 255, 0, 0 ), 3 );
	cv::line( mImageRGB, aPoints[ 3], aPoints[ 5], cv::Scalar( 255, 0, 0 ), 3 );
	cv::line( mImageRGB, aPoints[ 4], aPoints[ 6], cv::Scalar( 255, 0, 0 ), 3 );
	cv::line( mImageRGB, aPoints[ 5], aPoints[ 7], cv::Scalar( 255, 0, 0 ), 3 );
	cv::line( mImageRGB, aPoints[ 1], aPoints[ 8], cv::Scalar( 255, 0, 0 ), 3 );
	cv::line( mImageRGB, aPoints[ 8], aPoints[ 9], cv::Scalar( 255, 0, 0 ), 3 );
	cv::line( mImageRGB, aPoints[ 8], aPoints[10], cv::Scalar( 255, 0, 0 ), 3 );
	cv::line( mImageRGB, aPoints[ 9], aPoints[11], cv::Scalar( 255, 0, 0 ), 3 );
	cv::line( mImageRGB, aPoints[10], aPoints[12], cv::Scalar( 255, 0, 0 ), 3 );
	cv::line( mImageRGB, aPoints[11], aPoints[13], cv::Scalar( 255, 0, 0 ), 3 );
	cv::line( mImageRGB, aPoints[12], aPoints[14], cv::Scalar( 255, 0, 0 ), 3 );

	// draw joint
	for( int  s = 0; s < 15; ++ s )
	{
		if( aJoints[s].getPositionConfidence() > 0.5 )
			cv::circle( mImageRGB, aPoints[s], 3, cv::Scalar( 0, 0, 255 ), 2 );
		else
			cv::circle( mImageRGB, aPoints[s], 3, cv::Scalar( 0, 255, 0 ), 2 );
	}
	return 0;
}


int atre_user::reconstructMe(openni::VideoStream& oniDepthStream, openni::VideoFrameRef& oniDepthImg, string path)
{
	// initialize ReconstructMe
	// Create ReconstructMe context
	reme_context_t c;
	reme_context_create(&c);

	reme_options_t o;
	reme_options_create(c, &o);

	//// Create empty options biding
	//reme_context_set_log_callback(c, reme_default_log_callback, 0);

	// Choose the OpenCL device
	reme_context_bind_reconstruction_options(c, o);
	reme_options_set_int(c, o, "device_id", 0);

	// Compile for OpenCL device using defaults
	reme_options_set_int(c, o, "volume.minimum_corner.x", -1000);
	reme_options_set_int(c, o, "volume.minimum_corner.y", -1000);
	reme_options_set_int(c, o, "volume.minimum_corner.z", -1000);
	reme_options_set_int(c, o, "volume.maximum_corner.x", 1000);
	reme_options_set_int(c, o, "volume.maximum_corner.y", 1000);
	reme_options_set_int(c, o, "volume.maximum_corner.z", 1000);
	reme_options_set_int(c, o, "volume.resolution.x", 256);
	reme_options_set_int(c, o, "volume.resolution.y", 256);
	reme_options_set_int(c, o, "volume.resolution.z", 256);

	// Compile for OpenCL device using defaults
	reme_context_tune_reconstruction_options(c, REME_TUNE_PROFILE_LOW_QUALITY);
	reme_context_compile(c);

	// Create external sensor.
	reme_sensor_t s;
	reme_sensor_create(c, "external", false, &s);

	// Retrieve num_slices
	int num_slices;
	//OK(reme_context_bind_reconstruction_options(c, o));
	OK(reme_options_get_int(c, o, "volume.resolution.z", &num_slices));
	const void *bytes;
	int length;


	// Since we are using an external sensor,  we need to tell ReMe about its image size filed of view.
	reme_sensor_bind_camera_options(c, s, o);
	int image_width = oniDepthStream.getVideoMode().getResolutionX();
	int image_height = oniDepthStream.getVideoMode().getResolutionY();
	reme_options_set_int(c, o, "depth_stream.image_size.width", image_width);
	reme_options_set_int(c, o, "depth_stream.image_size.height", image_height);

	// The following intrinsics are based on a Kinect like device. By providing
	// intrinsics.width and intrinsics.height ReMe is able to automatically derive intrinsics
	// for different stream sizes.
	reme_options_set_int(c, o, "depth_stream.intrinsics.width", 640);
	reme_options_set_int(c, o, "depth_stream.intrinsics.height", 480);
	reme_options_set_real(c, o, "depth_stream.intrinsics.fx", 530);
	reme_options_set_real(c, o, "depth_stream.intrinsics.fy", 530);
	reme_options_set_real(c, o, "depth_stream.intrinsics.cx", 320);
	reme_options_set_real(c, o, "depth_stream.intrinsics.cy", 240);

	printf("ReconstructMe works OK\r\n");

	// Open the sensor like any other sensor.
	reme_sensor_open(c, s);
	reme_sensor_set_prescan_position(c, s, REME_SENSOR_POSITION_INFRONT);

	//// Specify eye, ref and up with respect to world
	//const float eye[3] = {0.f, -2000.f,  500.f};
	//const float ref[3] = {0.f,     0.f,  500.f};
	//const float up[3]  = {0.f,     0.f,    1.f};
	//// Create sensor coordinate system with respect to world
	//float mat[16];
	//reme_transform_look_at(c, eye, ref, up, mat);
	//// Set initial sensor position
	//reme_sensor_set_position(c, s, mat);
	//// We set the same position first recovery position
	//reme_sensor_set_recovery_position(c, s, mat);

	// In order inform ReMe about external sensor data
	//reme_image_t raw_depth;
	reme_image_t raw_depth;
	reme_image_create(c, &raw_depth);

	// Create a new volume
	reme_volume_t v;
	OK(reme_volume_create(c, &v));


	// For debugging purposes open a viewer for tracking the reconstruction process.
	reme_viewer_t viewer;
	reme_viewer_create_image(c, "This is ReconstructMe SDK", &viewer);

	reme_image_t volume;
	reme_image_create(c, &volume);
	reme_viewer_add_image(c, viewer, volume);

	// Perform reconstruction until one complete rotation is performed
	float prev_pos[16], cur_pos[16];
	float rotation_axis[4] = {0,0,1,0};

	reme_sensor_get_position(c, s, prev_pos);


	float angle;
	float sum_turn_angles = 0.f;

	while(fabs(sum_turn_angles) < 2.f * 3.1415f) {

		const unsigned short *sensor_pixel = (const unsigned short *)oniDepthImg.getData();

		if (!REME_SUCCESS(reme_sensor_grab(c,s))) {
			continue;
		}

		reme_sensor_prepare_image(c, s, REME_IMAGE_RAW_DEPTH);
		reme_sensor_get_image(c, s, REME_IMAGE_RAW_DEPTH, raw_depth);

		void *virtual_pixel;
		int nbytes;
		reme_image_get_mutable_bytes(c, raw_depth, &virtual_pixel, &nbytes );

		//Copy content
		memcpy(virtual_pixel, sensor_pixel, image_width * image_height * sizeof(unsigned short));
		// Only need to prepare volume data
		reme_sensor_prepare_image(c, s, REME_IMAGE_VOLUME);

		if (REME_SUCCESS(reme_sensor_track_position(c, s))) {
			reme_sensor_update_volume(c, s);

			reme_sensor_get_position(c, s, cur_pos);
			reme_transform_get_projected_angle(c, rotation_axis, prev_pos, cur_pos, &angle);
			sum_turn_angles += angle;
			memcpy(prev_pos, cur_pos, 16 * sizeof(float));
		}
		else
		{
			cout << "Tracking failed !!!" << endl;
			//cout << "Reconstruction is done. " << endl;

			// Close and destroy the sensor, it is not needed anymore
			reme_sensor_close(c, s);
			reme_sensor_destroy(c, &s);
			reme_context_destroy(&c);
			return(-1);

		}
		//reme_sensor_get_image(c, s, REME_IMAGE_AUX, aux);
		reme_sensor_get_image(c, s, REME_IMAGE_VOLUME, volume);
		reme_viewer_update(c, viewer);

	}

	// Close and destroy the sensor, it is not needed anymore
	reme_sensor_close(c, s);
	reme_sensor_destroy(c, &s);

	// Create a new surface
	reme_surface_t m;
	reme_surface_create(c, &m);
	reme_surface_generate(c, m, v);
	const char* fpath = path.c_str();
	reme_surface_save_to_file(c, m, fpath);
	this->aUserSurface = m;
	Sleep(1000);

	// Visualize resulting surface
	reme_viewer_t viewer_surface;
	reme_viewer_create_surface(c, m, "This is ReconstructMeSDK", &viewer_surface);
	reme_viewer_wait(c, viewer_surface);

	// finish of the reconstruct
	printf("Reconstruction is done\r\n");
	reme_surface_destroy(c, &m);

	// Print pending errors
	//reme_context_print_errors(c);
	// Make sure to release all memory acquired
	reme_context_destroy(&c);
	return(0);

}

// internal function girth

int thresh_output(int thresh,  Mat src, double& circumference)
{
	Mat canny_output;
	RNG rng(12345);
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	// use Canndy detect contour
	Canny(src, canny_output, thresh, thresh*2, 3);

	// find contour
	findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	/// Find the convex hull object for each contour
	vector<vector<Point> >hull( contours.size() );
	for( int i = 0; i < contours.size(); i++ )
	{  convexHull( Mat(contours[i]), hull[i], false ); }

	int *length = new int[contours.size()]; 

	for( int i = 0; i< contours.size(); i++ )
	{
		length[i] = arcLength( contours[i], true );
		//cout << " Length: " <<  length[i] << " ";
	}

	//cout << "The length is "  << *max_element(length, length + contours.size()) << '\n
	circumference = double(*max_element(length, length + contours.size())) ;
	int max_position = distance(length, max_element(length, length + contours.size()));

	Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
	for( int i = 0; i< contours.size(); i++ )
	{
		if (i ==max_position)
		{
			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
			//drawContours( drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
		}
	}

	//// Show in a window
	//namedWindow( "Contours", 1);
	//imshow( "Contours", drawing );
	//waitKey();
	return(0);

}

int Slice (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter, double layer, double& circumference)
{
	// Build a filter to remove spurious NaNs
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (layer, layer+7.8);
	pass.filter (*cloud_filter);


	// Testing upsampling
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls_upsampling;
	// Set parameters
	mls_upsampling.setInputCloud (cloud_filter);
	mls_upsampling.setSearchRadius (200);
	mls_upsampling.setPolynomialFit (true);
	mls_upsampling.setPolynomialOrder (2);

	mls_upsampling.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
	mls_upsampling.setUpsamplingRadius (10);
	mls_upsampling.setUpsamplingStepSize (1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZ> ());
	mls_upsampling.process (*cloud_smoothed);

	//pcl::visualization::CloudViewer viewerU("Upsampling Cloud Viewer");
	//viewerU.showCloud(cloud_smoothed);
	//while (!viewerU.wasStopped()){}


	//std::cerr << "PointCloud after filtering has: " << cloud_smoothed->points.size () << " data points." << std::endl;

	//pcl::visualization::CloudViewer viewer("Simple Cloud Viewer1");
	//viewer.showCloud(cloud);
	//while (!viewer.wasStopped()){}
	//pcl::visualization::CloudViewer viewer2("Simple Cloud Viewer2");
	//viewer2.showCloud(cloud_smoothed);
	//while (!viewer2.wasStopped()){}

	//for (int i = 0; i < cloud_smoothed->points.size(); i++) 
	//{ 
	//	cout << cloud_smoothed->points[i].x <<" "<<
	//		cloud_smoothed->points[i].y <<" "<<
	//		cloud_smoothed->points[i].z <<endl;
	//}


	Mat img = Mat::zeros(2000, 2000, CV_8U);

	for (int i = 0; i < cloud_smoothed->points.size(); i++) 
	{ 
		int x = (int)cloud_smoothed->points[i].x+1000;
		int y = -(int)cloud_smoothed->points[i].y+1000;

		img.at<uchar>(y, x) = 255;
	}

	cv::resize(img,img,Size(512,512),0,0,CV_INTER_LINEAR);

	//namedWindow( "image", 1);
	//imshow("image", img);
	////waitKey();

	int thresh = 1;
	// contour
	blur( img, img, Size(3,3) );
	thresh_output(thresh, img, circumference);

	return (0);

}

int atre_user::getGirth(string path, double layer, double& circumference)
{    

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter (new pcl::PointCloud<pcl::PointXYZ>); 

	//pcl::PLYReader reader;
	//int sucess = reader.read(path, *cloud); 
	//if ( sucess < 0 )
	//{
	//	cout << "read file error ... " << endl;
	//	return (-1);
	//}

	if (pcl::io::loadPLYFile<pcl::PointXYZ> (path, *cloud) != 0)
	{
		cout << "read file error ... " << endl;
		return (-1);
	}

	Slice (cloud,cloud_filter, layer, circumference);
	return(0);

}


// internal function for reme_filter. 
float signedVolumeOfTriangle(pcl::PointXYZ &p1, pcl::PointXYZ &p2, pcl::PointXYZ &p3) 
{
	float v321 = p3.x*p2.y*p1.z;
	float v231 = p2.x*p3.y*p1.z;
	float v312 = p3.x*p1.y*p2.z;
	float v132 = p1.x*p3.y*p2.z;
	float v213 = p2.x*p1.y*p3.z;
	float v123 = p1.x*p2.y*p3.z;
	return (1.0f/6.0f)*(-v321 + v231 + v312 - v132 - v213 + v123);
}

float volumeOfMesh(pcl::PolygonMesh mesh) 
{
	float vols = 0.0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);


	pcl::fromROSMsg(mesh.cloud, *cloud);
	//pcl::fromPointCloud2(mesh.cloud,*cloud);
	for(int triangle=0;triangle<mesh.polygons.size();triangle++)
	{
		pcl::PointXYZ pt1 = cloud->points[mesh.polygons[triangle].vertices[0]];
		pcl::PointXYZ pt2 = cloud->points[mesh.polygons[triangle].vertices[1]];
		pcl::PointXYZ pt3 = cloud->points[mesh.polygons[triangle].vertices[2]];
		vols += signedVolumeOfTriangle(pt1, pt2, pt3);
	}
	return abs(vols);
	//return Math.Abs(vols.Sum());
}

int atre_user::reme_filter(string path, double& weight)
{
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered  (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredX (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredXY (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredXYZ (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPLYFile (path, *cloud);

	//pcl::visualization::CloudViewer viewerPlane("Original");
	//viewerPlane.showCloud(cloud);
	//while (!viewerPlane.wasStopped())
	//{
	//	// Do nothing but wait.
	//}

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (200);
	sor.setStddevMulThresh (1.0);
	sor.filter (*cloud_filtered);

	//pcl::visualization::CloudViewer viewerPlane2("cloud_filtered");
	//viewerPlane2.showCloud(cloud_filtered);
	//while (!viewerPlane2.wasStopped())
	//{
	//	// Do nothing but wait.
	//}


	// Create the filtering object for x axis
	pcl::PassThrough<pcl::PointXYZ> passX;
	passX.setInputCloud (cloud_filtered);
	passX.setFilterFieldName ("x");
	passX.setFilterLimits (-800, 800);
	//pass.setFilterLimitsNegative (true);
	passX.filter (*cloud_filteredX);


	pcl::PassThrough<pcl::PointXYZ> passXY;
	passXY.setInputCloud (cloud_filteredX);
	passXY.setFilterFieldName ("y");
	passXY.setFilterLimits (-800, 800);
	//pass.setFilterLimitsNegative (true);
	passXY.filter (*cloud_filteredXY);

	pcl::PassThrough<pcl::PointXYZ> passXYZ;
	passXYZ.setInputCloud (cloud_filteredXY);
	passXYZ.setFilterFieldName ("z");
	passXYZ.setFilterLimits (0, 1);
	//pass.setFilterLimitsNegative (true);
	passXYZ.filter (*cloud_filteredXYZ);

	//pcl::visualization::CloudViewer viewerPlane3("cloud_filteredXYZ");
	//viewerPlane3.showCloud(cloud_filteredXYZ);
	//while (!viewerPlane3.wasStopped())
	//{
	//	// Do nothing but wait.
	//}

	pcl::io::savePLYFileASCII ("test_remove2.ply", *cloud_filteredXY);

	//// upsampling 
	//pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
	//   mls.setInputCloud (cloud_filteredXY);
	//   mls.setSearchRadius (4);
	//   mls.setPolynomialFit (true);
	//   mls.setPolynomialOrder (1);
	//   mls.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
	//   mls.setUpsamplingRadius (1);
	//   mls.setUpsamplingStepSize (0.3);

	//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZ> ());
	//   mls.process (*cloud_smoothed);

	//pcl::visualization::CloudViewer viewerPlane4("cloud_smoothed");
	//viewerPlane4.showCloud(cloud_smoothed);
	//while (!viewerPlane4.wasStopped())
	//{
	//	// Do nothing but wait.
	//}

	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setNumberOfThreads (8);
	ne.setInputCloud (cloud_filteredXY);
	//ne.setRadiusSearch (0.8);
	ne.setKSearch(1500);
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid (*cloud_filteredXY, centroid);
	ne.setViewPoint (centroid[0], centroid[1], centroid[2]);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
	ne.compute (*cloud_normals);
	for (size_t i = 0; i < cloud_normals->size (); ++i)
	{
		cloud_normals->points[i].normal_x *= -1;
		cloud_normals->points[i].normal_y *= -1;
		cloud_normals->points[i].normal_z *= -1;
	}
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals (new pcl::PointCloud<pcl::PointNormal> ());
	pcl::concatenateFields (*cloud_filteredXY, *cloud_normals, *cloud_smoothed_normals);

	pcl::Poisson<pcl::PointNormal> poisson;
	poisson.setDepth (9);
	poisson.setInputCloud  (cloud_smoothed_normals);
	pcl::PolygonMesh mesh_poisson;
	poisson.reconstruct (mesh_poisson);


	// save poisson mesh file
	string result = path + "_RESULT.PLY";
	pcl::io::savePLYFile(result, mesh_poisson);

	// calculate volume
	float volume;
	volume = volumeOfMesh(mesh_poisson); 
	weight = volume/1000/1000;
	this->aUserWeight = weight;
	return (0);

}

int atre_user::getSurface(reme_surface_t& surface)
{
	surface = this->aUserSurface;
	return 0;
}

int atre_user::getWeight(double& weight)
{
	weight = this->aUserWeight;
	return 0;
}




