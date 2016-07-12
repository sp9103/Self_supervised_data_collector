#include "InvalidMotionHandler.h"


InvalidMotionHandler::InvalidMotionHandler(void)
{
	DeinitCheck = true;

	ROI3D.first = cv::Point3f(259.f, -300.f, -100.f);
	ROI3D.second = cv::Point3f(500.f, 55.f, 0.f);

	robotDataFormat.Thumb.x = 130.0f;
	robotDataFormat.Thumb.y = 0.0f;
	robotDataFormat.Thumb.z = 140.0f;
	robotDataFormat.upperLeft.x = -60.0f;
	robotDataFormat.upperLeft.y = 45.0f;
	robotDataFormat.upperLeft.z = 200.0f;
	robotDataFormat.upperRight.x = -60.0f;
	robotDataFormat.upperRight.y = -45.0f;
	robotDataFormat.upperRight.z = 200.0f;
}


InvalidMotionHandler::~InvalidMotionHandler(void)
{
	if(!DeinitCheck)
		Deinitialize();
}

//true == colision detected. false == safe
bool InvalidMotionHandler::InvalidCheck(int *angle, int *prevAngle){
	bool retVal = false;
	const int FingerMaxLimit[3] = {2940, 1600, 2100};
	const int FingerMinLimit[3] = {2480, 1150, 1800};
	const int JointMaxLimit[6] = {250950, 148000, 250950, 230000, 145000, 50000};
	const int JointMinLimit[6] = {-250950,-148000, -250950, -56600, -145000, -50000};

	for(int i = 0; i < NUM_JOINT; i++){
		int angleOrigin = angle[i];
		if(angleOrigin < JointMinLimit[i] || angleOrigin > JointMaxLimit[i])
			return false;
	}

	for(int i = 0; i < 3; i++){
		int angleOrigin = angle[NUM_JOINT + i];
		if(angleOrigin < FingerMinLimit[i] || angleOrigin > FingerMaxLimit[i])
			return false;
	}


	//관심영역 안인지 밖인지를 체크
	veci angi(6);
	vecd angd;
	armsdk::Pose3D endEffector;
	angi.resize(6);
	for(int i = 0; i < 6; i++)		angi[i] = angle[i];
	angd = kin.Value2Rad(angi);
	kin.Forward(angd, &endEffector);

	if(!inROI(endEffector))	return false;

#ifdef USING_SIMULATOR
	//시뮬레이터 체크
	RobotInfoData sendData = robotDataFormat;
	armsdk::Pose3D xaxis;
	for(int i = 0; i < 6; i++)
		sendData.Angle[i] = angle[i];
	fingerTransform(&sendData, &xaxis);

	//손목 각도 체크
	float magx = sqrt(xaxis.x * xaxis.x + xaxis.y * xaxis.y + xaxis.z * xaxis.z);
	float dotp = (xaxis.z * 1.f) / magx;
	float ang = acos(dotp) * 180 / PI;
	if(ang < 45.f)
		return false;

	retVal = robotvisServer.SendAndCheck(sendData);

	//시뮬레이터 되돌리기
	if(!retVal){
		sendData = robotDataFormat;
		for(int i = 0; i < 6; i++)
			sendData.Angle[i] = prevAngle[i];
		fingerTransform(&sendData);
		robotvisServer.SendAndCheck(sendData);
	}
#endif

	return retVal;
}

bool InvalidMotionHandler::robotConnect(RobotArm *robotArm){
	veci angi(6);
	robotArm->Arm_Get_JointValue(&angi);

	//맥시멈 앵글 체크 - 쓰레기값 걸러내기
	for(int JointNum = 0; JointNum < 6; JointNum++)
	{
		if(abs(angi[JointNum]) > robot.GetJointInfo(JointNum)->GetMaxAngleInValue() + 10)
		{
			cout<<"read fail"<<endl;
			printf("Data Fail %d\n", angi[JointNum]);
			return false;
		}
	}

#ifdef USING_SIMULATOR
	RobotInfoData sendData = robotDataFormat;
	for(int i = 0; i < 6; i++)
		sendData.Angle[i] = angi[i];
	fingerTransform(&sendData);
	robotvisServer.SendAndCheck(sendData);
#endif

	return true;
}

void InvalidMotionHandler::Initialize(){
	DeinitCheck = false;
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
	kin.InitRobot(&robot);

#ifdef USING_SIMULATOR
	HANDLE _TThreadHandle = (HANDLE)_beginthreadex(NULL, 0, simulateThread, NULL, 0, NULL);			//simulator start - TO-DO : How to exit thread safely
	robotvisServer.Init(NULL, PORT);																//server start
#endif
}

UINT WINAPI InvalidMotionHandler::simulateThread(LPVOID param){
	system("..\\..\\robotArmVis\\RobotSimulator.exe");

	return 1;
}

void InvalidMotionHandler::fingerTransform(RobotInfoData *src, armsdk::Pose3D *xaxis, armsdk::Pose3D *yaxis, armsdk::Pose3D *zaxis){
	veci angi(6);
	for(int i = 0; i < 6; i++)	angi[i] = src->Angle[i];
	vecd angd = kin.Value2Rad(angi);
	armsdk::Pose3D CurrentPose, xaxis_, yaxis_, zaxis_;
	kin.EndAxis(angd, &CurrentPose, &xaxis_, &yaxis_, &zaxis_);

	cv::Mat m_RotMat(3,3,CV_32FC1);

	cv::Mat invR;
	invR.create(3,3,CV_32FC1);

	invR.at<float>(0,0) = xaxis_.x;
	invR.at<float>(1,0) = xaxis_.y;
	invR.at<float>(2,0) = xaxis_.z;

	invR.at<float>(0,1) = yaxis_.x;
	invR.at<float>(1,1) = yaxis_.y;
	invR.at<float>(2,1) = yaxis_.z;

	invR.at<float>(0,2) = zaxis_.x;
	invR.at<float>(1,2) = zaxis_.y;
	invR.at<float>(2,2) = zaxis_.z;

	if(xaxis != NULL)	*xaxis = xaxis_;
	if(yaxis != NULL)	*yaxis = yaxis_;
	if(zaxis != NULL)	*zaxis = zaxis_;

	//m_RotMat = invR.t();

	rot(invR, &src->Thumb);
	rot(invR, &src->upperLeft);
	rot(invR, &src->upperRight);
}

void InvalidMotionHandler::rot(cv::Mat rotMat, FingerInfo *fin){
	cv::Mat temp, resultMat;
	temp.create(3,1,CV_32FC1);
	resultMat.create(3,1,CV_32FC1);

	temp.at<float>(0,0) = fin->x;
	temp.at<float>(1,0) = fin->y;
	temp.at<float>(2,0) = fin->z;

	resultMat = rotMat * temp;

	fin->x = resultMat.at<float>(0,0);
	fin->y = resultMat.at<float>(1,0);
	fin->z = resultMat.at<float>(2,0);
}

void InvalidMotionHandler::Deinitialize(){
	HWND handle = FindWindow(NULL, TEXT("robotArmVis"));
	SendMessage(handle, WM_CLOSE, 0, 0);
	DeinitCheck = true;
}

armsdk::Pose3D InvalidMotionHandler::ForwardEnd(RobotArm *robotArm){
	veci angi(6);

	armsdk::Pose3D endeffector;
	robotArm->Arm_Get_JointValue(&angi);

	vecd angd = kin.Value2Rad(angi);
	kin.Forward(angd, &endeffector);

	printf("%f %f %f\n", endeffector.x, endeffector.y, endeffector.z);
	return endeffector;
}

bool InvalidMotionHandler::inROI(armsdk::Pose3D end){
	if(ROI3D.first.x < end.x && end.x < ROI3D.second.x)
		if(ROI3D.first.y < end.y && end.y < ROI3D.second.y)
			if(ROI3D.first.z < end.z && end.z < ROI3D.second.z)
				return true;

	return false;
}
