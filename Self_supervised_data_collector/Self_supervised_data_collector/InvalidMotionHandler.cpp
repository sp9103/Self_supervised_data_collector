#include "InvalidMotionHandler.h"


InvalidMotionHandler::InvalidMotionHandler(void)
{
	DeinitCheck = true;
}


InvalidMotionHandler::~InvalidMotionHandler(void)
{
	if(!DeinitCheck)
		Deinitialize();
}

//true == colision detected. false == safe
bool InvalidMotionHandler::InvalidCheck(int *angle){
	bool retVal = false;
	const int FingerMaxLimit[3] = {};
	const int FingerMinLimit[3] = {};

	for(int i = 0; i < 3; i++)
		if(angle[NUM_JOINT + i] < FingerMinLimit[i] || angle[NUM_JOINT + i] > FingerMaxLimit[i])
			return false;


	//관심영역 안인지 밖인지를 체크
	veci angi(6);
	vecd angd;
	armsdk::Pose3D endEffector;
	angi.resize(6);
	for(int i = 0; i < 6; i++)		angi[i] = angle[i];
	angd = kin.Value2Rad(angi);
	kin.Forward(angd, &endEffector);

	//EndEffector ROI check
	if(true /*TO-DO*/)	retVal = true;
	else				retVal = false;

#ifdef USING_SIMULATOR
	//시뮬레이터 체크
	RobotInfoData sendData;
	for(int i = 0; i < 6; i++)
		sendData.Angle[i] = angle[i];
	sendData.Thumb.x = -40.0f;
	sendData.Thumb.y = 0.0f;
	sendData.Thumb.z = 70.0f;
	sendData.upperLeft.x = 40.0f;
	sendData.upperLeft.y = 30.0f;
	sendData.upperLeft.z = 70.0f;
	sendData.upperRight.x = 40.0f;
	sendData.upperRight.y = -30.0f;
	sendData.upperRight.z = 70.0f;
	retVal = robotvisServer.SendAndCheck(sendData);
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
	RobotInfoData sendData;
	for(int i = 0; i < 6; i++)
		sendData.Angle[i] = angi[i];
	sendData.Thumb.x = -40.0f;
	sendData.Thumb.y = 0.0f;
	sendData.Thumb.z = 70.0f;
	sendData.upperLeft.x = 40.0f;
	sendData.upperLeft.y = 30.0f;
	sendData.upperLeft.z = 70.0f;
	sendData.upperRight.x = 40.0f;
	sendData.upperRight.y = -30.0f;
	sendData.upperRight.z = 70.0f;
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

void InvalidMotionHandler::Deinitialize(){
	HWND handle = FindWindow(NULL, TEXT("robotArmVis"));
	SendMessage(handle, WM_CLOSE, 0, 0);
	DeinitCheck = true;
}