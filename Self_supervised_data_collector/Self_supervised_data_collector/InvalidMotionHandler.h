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

	//�ش� �ޱ۷� ������ �� ������ return true, else return false
	void Initialize();
	void Deinitialize();
	bool InvalidCheck(int *angle);
	bool robotConnect(RobotArm *robot);

private:
	armsdk::RobotInfo robot;
	armsdk::Kinematics kin;
	RobotVisServer robotvisServer;
	bool DeinitCheck;

	static UINT WINAPI simulateThread(LPVOID param); // ������ �Լ�.
};

