#pragma once
class InvalidMotionHandler
{
public:
	InvalidMotionHandler(void);
	~InvalidMotionHandler(void);

	//�ش� �ޱ۷� ������ �� ������ return true, else return false
	bool InvalidCheck(int *angle);
};

