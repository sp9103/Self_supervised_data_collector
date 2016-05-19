//#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <winsock.h>

#pragma comment(lib, "ws2_32.lib")

typedef struct fingerInfo{
	float x,y,z;
}FingerInfo;

typedef struct robotInfo{
	int Angle[6];
	FingerInfo upperLeft;
	FingerInfo upperRight;
	FingerInfo Thumb;
}RobotInfoData;

class RobotVisServer
{
public:
	RobotVisServer(void);
	~RobotVisServer(void);

	void GetIPAddress(char *ip);
	void Init(char *ip, int portNum);
	void DeInit();

	bool SendAndCheck(RobotInfoData data);

private:
	int _portNum;
	char _IP[256];
	WSADATA wsaData;
	SOCKET hServSock;
	SOCKET hClntSock;
	SOCKADDR_IN servAddr;
	SOCKADDR_IN clntAddr;
	int szClntAddr;

	void ErrorHandling(char *message);
};

