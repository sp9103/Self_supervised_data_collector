#include <process.h>

#include "ARMSDK\include\ARMSDK.h"
#include "stdafx.h"
#include "RobotVisServer.h"
#include "Robot\RobotArm.h"

#define USING_SIMULATOR
#define PORT 2252

class InvalidMotionHandler
{
public:
	InvalidMotionHandler(void);
	~InvalidMotionHandler(void);

	//해당 앵글로 움직일 수 있으면 return true, else return false
	void Initialize();
	void Deinitialize();
	bool InvalidCheck(int *angle, int *prevAngle);
	bool robotConnect(RobotArm *robot);
	armsdk::Pose3D ForwardEnd(RobotArm *robotArm);

private:
	armsdk::RobotInfo robot;
	armsdk::Kinematics kin;
	RobotVisServer robotvisServer;
	std::pair<cv::Point3f, cv::Point3f> ROI3D;
	RobotInfoData robotDataFormat;
	bool DeinitCheck;

	static UINT WINAPI simulateThread(LPVOID param); // 쓰레드 함수.
	void fingerTransform(RobotInfoData *src);
	void rot(cv::Mat rotMat, FingerInfo *fin);
	bool inROI(armsdk::Pose3D end);
};

