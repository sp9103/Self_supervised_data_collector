//#include <Windows.h>
#include <stdio.h>
#include <string>

#include "RobotVisClient.h"
#include "ARMSDK\Kinematics.h"
#include "ARMSDK\RobotInfo.h"

#define EXPORT_API __declspec(dllexport)
#define PORT 2252

#define RIGHT_ARM_USE /*LEFT_ARM_USE*/

using namespace armsdk;
using namespace std;

extern "C"{
	armsdk::RobotInfo robot;

	RobotVisClient client;
	Kinematics kin;

	void EXPORT_API TEST(){
		MessageBox(NULL, L"Client DLL Test Successs", L"TEST", MB_OK);
	}

	void EXPORT_API clientInit(){
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
		kin.setRobotInfo(&robot);

		client.Init(NULL, PORT);
	}

	void EXPORT_API clientDeinit(){
		client.DeInit();
	}

	//x,y,z ������ �ؿ��� ���� ��ȯ
	int EXPORT_API Receive(float position[], float fingerPos[]){
		RobotState temp;
		float pos[3*7 + 3*3];

		if(client.getData(&temp) == -1)
			return -1;

		fingerPos[3*0 + 0] = temp.upperLeft.x / 10.f;
		fingerPos[3*0 + 1] = temp.upperLeft.y / 10.f;
		fingerPos[3*0 + 2] = temp.upperLeft.z / 10.f;

		fingerPos[3*1 + 0] = temp.upperRight.x / 10.f;
		fingerPos[3*1 + 1] = temp.upperRight.y / 10.f;
		fingerPos[3*1 + 2] = temp.upperRight.z / 10.f;

		fingerPos[3*2 + 0] = temp.Thumb.x / 10.f;
		fingerPos[3*2 + 1] = temp.Thumb.y / 10.f;
		fingerPos[3*2 + 2] = temp.Thumb.z / 10.f;

		//��ǥ�� ��ȯ
		/*for(int i = 0; i < 6; i++)
		(angi)[i] = temp.Angle[i];
		kin.Forward(kin.Value2Rad(angi), pos);*/

		//memcpy(position, pos, sizeof(float)*(3*7 + 3*3));

		//�� ����
		for(int i = 0; i < 6; i++){
			position[i] = (float)temp.Angle[i];
		}

		return 1;
	}

	void EXPORT_API AngleToPos(float angle[], float pos[]){
		veci angi(6);
		for(int i = 0; i < 6; i++)
			(angi)[i] = angle[i];
		kin.Forward(kin.Value2Rad(angi), pos);
	}

	void EXPORT_API SendResult(int result){
		//client.SendResult(true);
		bool bResult = (result == 1) ? true : false;
		client.CalcCollision(bResult);
	}
}