#include "InvalidMotionHandler.h"


InvalidMotionHandler::InvalidMotionHandler(void)
{
}


InvalidMotionHandler::~InvalidMotionHandler(void)
{
}

//true == colision detected. false == safe
bool InvalidMotionHandler::InvalidCheck(int *angle){
	bool retVal = false;

	//관심영역 안인지 밖인지를 체크

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

	return retVal;
}

bool InvalidMotionHandler::robotConnect(){
	//robot.Arm_Get_JointValue(&angi);

	////맥시멈 앵글 체크 - 쓰레기값 걸러내기
	//for(int JointNum = 0; JointNum < 6; JointNum++)
	//{
	//	if(abs(angi[JointNum]) > robot.GetJointInfo(JointNum)->GetMaxAngleInValue() + 10)
	//	{
	//		cout<<"read fail"<<endl;
	//		return;
	//	}
	//}

	return true;
}

void InvalidMotionHandler::Initialize(){
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
	robotvisServer.Init(NULL, PORT);																//server start
	HANDLE _TThreadHandle = (HANDLE)_beginthreadex(NULL, 0, simulateThread, NULL, 0, NULL);			//simulator start - TO-DO : How to exit thread safely
#endif
}

UINT WINAPI InvalidMotionHandler::simulateThread(LPVOID param){
	system("..\\robotArmVis\\RobotSimulator.exe");

	return 1;
}

void InvalidMotionHandler::Deinitialize(){

}