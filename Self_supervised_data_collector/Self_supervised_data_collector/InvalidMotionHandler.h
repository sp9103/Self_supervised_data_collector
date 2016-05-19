#include <process.h>

#include "ARMSDK\include\ARMSDK.h"
#include "stdafx.h"
#include "RobotVisServer.h"

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
	bool robotConnect();

private:
	armsdk::RobotInfo robot;
	armsdk::Kinematics kin;
	RobotVisServer robotvis;

	static UINT WINAPI simulateThread(LPVOID param); // ������ �Լ�.
};

