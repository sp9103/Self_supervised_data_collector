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

	//�ܺη� �ڷ� �������� �Լ�		(true�� ���� false�� ���� ����)
	int getData(RobotState *dst);

	//Collision result ������ �����Լ�
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

	//Thread ���� �Լ�
	CRITICAL_SECTION m_cs;
	bool m_EndThread;
	bool m_EnableThread;
	bool m_InitCheckThread;
	bool m_isSocketOpen;
	Thread m_Thread;

	static UINT WINAPI socketThread(LPVOID param); // ������ �Լ�.
	void GetIPAddress(char *ip);

	//����� �Ⱦ��� �Լ�
	void SendResult(bool result);
	void Recv();								//���� ����ü
};

