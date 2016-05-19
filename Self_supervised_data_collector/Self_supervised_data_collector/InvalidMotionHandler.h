#include "ARMSDK\include\ARMSDK.h"
#include "stdafx.h"

class InvalidMotionHandler
{
public:
	InvalidMotionHandler(void);
	~InvalidMotionHandler(void);

	//해당 앵글로 움직일 수 있으면 return true, else return false
	void Initialize();
	bool InvalidCheck(int *angle);
	bool robotConnect();

private:
	armsdk::RobotInfo robot;
};

