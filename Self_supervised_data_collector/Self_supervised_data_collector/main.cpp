#include <stdio.h>
#include <afx.h>

#include "ARMSDK\include\ARMSDK.h"
#include "KinectConnecter.h"
#include "Robot\RobotArm.h"

using namespace armsdk;

#ifdef _DEBUG
#pragma comment(lib, "ARMSDKd.lib")
#endif
#ifdef NDEBUG
#pragma comment(lib, "ARMSDK.lib") 
#endif

#define RIGHT_ARM_USE /*LEFT_ARM_USE*/
#define NUM_XEL			9

int main(){
	//class 
	KinectConnecter kinect;
	RobotArm arm;
	armsdk::RobotInfo robot;

	//variable
	cv::Mat KinectMappingImage;
	cv::Mat KinectDepthimage;
	cv::Mat KinectXYZImage;
	cv::Mat KinectColorImage;
	cv::Rect RobotROI((KINECT_DEPTH_WIDTH - 160) / 2, (KINECT_DEPTH_HEIGHT- 160) / 2, 160, 160);
	bool saveCheck = false;

	//initialize
	kinect.KinectInitialize();
	KinectColorImage.create(KINECT_COLOR_HEIGHT, KINECT_COLOR_WIDTH, CV_8UC4);			//Kinect Color Image format BGRA 4 channel image

#ifdef RIGHT_ARM_USE
	//RightArm
	robot.AddJoint(  0.0,  ML_PI_2,    0.0,      0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 1);
	robot.AddJoint(  0.0, -ML_PI_2,    0.0,      0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 3);
	robot.AddJoint( 30.0, -ML_PI_2,  246.0,      0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 5);
	robot.AddJoint(-30.0,  ML_PI_2,    0.0,  ML_PI_2, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 7);
	robot.AddJoint(  0.0, -ML_PI_2,  216.0,      0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 9);
	robot.AddJoint(  0.0,  ML_PI_2,    0.0,      0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 11);
#elif defined LEFT_ARM_USE
	//Leftarm
	robot.AddJoint(  0.0, -ML_PI_2,    0.0,      0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 2);
	robot.AddJoint(  0.0,  ML_PI_2,    0.0,      0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 4);
	robot.AddJoint( 30.0,  ML_PI_2,  246.0,      0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 6);
	robot.AddJoint(-30.0, -ML_PI_2,    0.0, -ML_PI_2, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 8);
	robot.AddJoint(  0.0,  ML_PI_2,  216.0,      0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 10);
	robot.AddJoint(  0.0, -ML_PI_2,    0.0,      0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 12);
#endif

	while(1){
		char key = cv::waitKey(10);

		kinect.GetColorImage(&KinectColorImage);
		kinect.GetRGBDnDepthnXYZ(&KinectMappingImage, &KinectDepthimage, &KinectXYZImage);
		if(KinectMappingImage.cols != 0){
			cv::Mat tempMapImg = KinectMappingImage.clone();
			cv::rectangle(tempMapImg, cv::Rect((KINECT_DEPTH_WIDTH-160)/2, (KINECT_DEPTH_HEIGHT-160)/2, 160,160), cv::Scalar(0,255,0));
			imshow("KinectMapFrame", tempMapImg);
		}

		if(key == 27)
			break;
		else if(key == 's')		saveCheck = !saveCheck;

		//실제 저장부
		if(saveCheck){

		}
	}

	//Deallocation
	kinect.KinectDestroy();
	cv::destroyAllWindows();

	return 0;
}