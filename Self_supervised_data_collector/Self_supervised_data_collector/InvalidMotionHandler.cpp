#include "InvalidMotionHandler.h"


InvalidMotionHandler::InvalidMotionHandler(void)
{
}


InvalidMotionHandler::~InvalidMotionHandler(void)
{
}


bool InvalidMotionHandler::InvalidCheck(int *angle){
	//TO-DO

	return true;
}

bool InvalidMotionHandler::robotConnect(){
#ifdef USING_SIMULATOR
	robotvis.Init(NULL, PORT);
#endif

	//robot.Arm_Get_JointValue(&angi);

	////¸Æ½Ã¸Ø ¾Þ±Û Ã¼Å© - ¾²·¹±â°ª °É·¯³»±â
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
	HANDLE _TThreadHandle = (HANDLE)_beginthreadex(NULL, 0, simulateThread, NULL, 0, NULL);
#endif
}

UINT WINAPI InvalidMotionHandler::simulateThread(LPVOID param){
	system("simulator\\RobotSimulator.exe");

	return 1;
}

void InvalidMotionHandler::Deinitialize(){

}