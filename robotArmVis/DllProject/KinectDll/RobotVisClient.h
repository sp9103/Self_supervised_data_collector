#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <winsock2.h>

#include "Thread.h"

#pragma comment(lib, "ws2_32.lib")

typedef struct fingerInfo{
	float x,y,z;
}FingerInfo;

typedef struct robotInfo{
	int Angle[6];
	FingerInfo upperLeft;
	FingerInfo upperRight;
	FingerInfo Thumb;
}RobotState;

class RobotVisClient
{
public:
	RobotVisClient(void);
	~RobotVisClient(void);

	void Init(char *ip, int portNum);
	void DeInit();

	//외부로 자료 가져오는 함수		(true면 쓰고 false면 연산 포기)
	int getData(RobotState *dst);

	//Collision result 쓰레드 전달함수
	bool CollisionCheck();
	void CalcCollision(bool result);

private:
	int _portNum;
	char _IP[256];
	bool isNewData;
	bool isCollision;
	RobotState robotData;

	WSADATA wsaData;
	SOCKET hSocket;
	SOCKADDR_IN servAddr;

	void ErrorHandling(char *message);

	//Thread 관련 함수
	CRITICAL_SECTION m_cs;
	bool m_EndThread;
	bool m_EnableThread;
	bool m_InitCheckThread;
	bool m_isSocketOpen;
	Thread m_Thread;

	static UINT WINAPI socketThread(LPVOID param); // 쓰레드 함수.
	void GetIPAddress(char *ip);

	//현재는 안쓰는 함수
	void SendResult(bool result);
	void Recv();								//추후 구조체
};

