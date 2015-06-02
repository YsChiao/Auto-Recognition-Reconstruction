#pragma once
#include <NiTE.h>
#include <OpenNI.h>
#include <iostream>
#include <string>
#include <opencv2\core\core.hpp>
#include <openbr/openbr_plugin.h>
#include <reconstructmesdk\reme.h>


using namespace std;


class atre_user 
{
public:
	atre_user(void);
	~atre_user(void);

	nite::UserId     getId();
	nite::Skeleton   getSkeleton();
	nite::PoseData   getPose(nite::PoseType);

	int  isNew();
	int  isLost();
	int  isVisible();
	int  loadUser(nite::UserData);
	int  loadImage(openni::VideoFrameRef);
	int  showImage();
	int  getGenderAge (int, char**, QSharedPointer<br::Transform>, QSharedPointer<br::Transform>, QString&, int&);
	int  getArmLength(double&, double&);
	int  getLegLength(nite::Plane&, double&, double&, double);
	int  getHeight(openni::VideoStream&, openni::VideoFrameRef&, nite::UserMap, nite::Plane&, double&, double entropy);
	int  getPoition(string, double&); 
	int  getFloor(nite::UserTrackerFrameRef&, nite::Plane&);
	int  skeletonTracking(nite::UserTracker&, cv::Mat&);

	int  reconstructMe(openni::VideoStream&, openni::VideoFrameRef&, string );

	int  reme_filter(string, double& );
	int  getGirth(string, double, double&);

	int  getSurface(reme_surface_t& surface);
	int  getWeight(double&);



private:
	nite::UserData  aUser;
	nite::UserId    aUserId;
	nite::Skeleton  aUserSkeleton;
	cv::Mat         aUserImage;
	string          aUserGender;
	int             aUserAge;
	

	reme_surface_t  aUserSurface; 
	double          aUserWeight;
};

