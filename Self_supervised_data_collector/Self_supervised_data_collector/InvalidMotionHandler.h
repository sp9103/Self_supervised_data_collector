#include "ARMSDK\include\ARMSDK.h"
#include "stdafx.h"

class InvalidMotionHandler
{
public:
	InvalidMotionHandler(void);
	~InvalidMotionHandler(void);

	//�ش� �ޱ۷� ������ �� ������ return true, else return false
	void Initialize();
	bool InvalidCheck(int *angle);
	bool robotConnect();

private:
	armsdk::RobotInfo robot;
};

