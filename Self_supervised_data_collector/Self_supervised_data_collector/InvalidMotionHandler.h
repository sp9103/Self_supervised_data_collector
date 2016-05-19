#pragma once
class InvalidMotionHandler
{
public:
	InvalidMotionHandler(void);
	~InvalidMotionHandler(void);

	//해당 앵글로 움직일 수 있으면 return true, else return false
	bool InvalidCheck(int *angle);
};

