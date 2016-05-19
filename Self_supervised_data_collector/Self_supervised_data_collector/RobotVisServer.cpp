#include "RobotVisServer.h"


RobotVisServer::RobotVisServer(void)
{
}


RobotVisServer::~RobotVisServer(void)
{
}

void RobotVisServer::GetIPAddress(char *ip){
	WSADATA wsaData;
	WSAStartup(MAKEWORD(2, 2), &wsaData);

	PHOSTENT hostinfo;
	char hostname[50];
	char ipaddr[50];
	memset(hostname, 0, sizeof(hostname));
	memset(ipaddr, 0, sizeof(ipaddr));

	int nError = gethostname(hostname, sizeof(hostname));
	if (nError == 0)
	{
		hostinfo = gethostbyname(hostname);
		// ip address �ľ�
		strcpy(ipaddr, inet_ntoa(*(struct in_addr*)hostinfo->h_addr_list[0]));
	}

	WSACleanup();

	strcpy(ip, ipaddr);
}

void RobotVisServer::Init(char *ip, int portNum){
	_portNum = portNum;

	if(ip == NULL){
		char temp[256];
		GetIPAddress(temp);
		strcpy(_IP, temp);
	}else{
		strcpy(_IP, ip);
	}

	// Load WinSocket 2.2 DLL
	if(WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
	{
		ErrorHandling("WSAStartup(), error");
	}

	// ���� ���� ����
	hServSock = socket(PF_INET, SOCK_STREAM, 0);
	if(hServSock == INVALID_SOCKET)
	{
		ErrorHandling("socket() error");
	}

	memset(&servAddr, 0, sizeof(servAddr));
	servAddr.sin_family = AF_INET;
	servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	servAddr.sin_port = htons(_portNum);

	// ���Ͽ� �ּ� �Ҵ�
	if(bind(hServSock, (SOCKADDR*) &servAddr, sizeof(servAddr)) == SOCKET_ERROR)
	{
		ErrorHandling("bind() error");
	}

	// ���� ��û ��� ����
	if(listen(hServSock, 5) == SOCKET_ERROR)
	{
		ErrorHandling("listen() error");
	}

	// ���� ��û ����
	szClntAddr = sizeof(clntAddr);
	hClntSock = accept(hServSock, (SOCKADDR*) &clntAddr, &szClntAddr);
	if(hClntSock == INVALID_SOCKET)
	{
		ErrorHandling("accept() error");
	}
}

void RobotVisServer::ErrorHandling(char *message)
{
	fputs(message, stderr);
	fputc('\n', stderr);
	exit(1);
}

void RobotVisServer::DeInit(){
	// ���� ����
	closesocket(hClntSock);
	WSACleanup();
}

bool RobotVisServer::SendAndCheck(RobotInfoData data){
	char buf[256];
	memcpy(buf, &data, sizeof(RobotInfoData));
	send(hClntSock, buf, sizeof(RobotInfoData), 0);
	
	// ������ ���� 
	char check;
	int strLen = recv(hClntSock, &check, 1, 0);
	if(strLen == -1)
	{
		ErrorHandling("read() error");
	}

	return (check == 1) ? true : false;
}