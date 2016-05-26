#include "RobotVisClient.h"


RobotVisClient::RobotVisClient(void)
{
	m_EnableThread = false;
	m_EndThread = false;
	m_InitCheckThread = false;
	isCollision = false;
	isNewData = false;
	m_isSocketOpen = false;

	InitializeCriticalSection(&m_cs);
}


RobotVisClient::~RobotVisClient(void)
{
	//Thread�� ���� ����ɶ����� ��ٸ�
	m_EnableThread = false;

	while(m_EndThread){
		Sleep(33);
	}
}

void RobotVisClient::GetIPAddress(char *ip){
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

void RobotVisClient::Init(char *ip, int portNum){
	_portNum = portNum;

	if(ip == NULL){
		char temp[256];
		GetIPAddress(temp);
		strcpy(_IP, temp);
	}else{
		strcpy(_IP, ip);
	}

	//Thread Initialize
	m_EnableThread = true;
	m_Thread.StartThread(socketThread, this);

	//Thread Initialize wait
	while(!m_InitCheckThread){
		Sleep(10);
	}

}

void RobotVisClient::ErrorHandling(char *message)
{
	wchar_t text1[100];
	mbstowcs(text1, message, strlen(message) + 1);
	LPCWSTR test = text1;

	MessageBox(NULL, test, L"DLL Error", MB_OK);
}

void RobotVisClient::DeInit(){
	// ���� ����
	closesocket(hSocket);
	WSACleanup();
	m_isSocketOpen = false;
}

void RobotVisClient::SendResult(bool result){
	if(m_isSocketOpen){
		char temp = (result == true) ? 1 : 0;
		send(hSocket, &temp, 1, 0);
	}
}

void RobotVisClient::Recv(){								//���� ����ü
	char message[256];

	// ������ ���� 
	int strLen = recv(hSocket, message, sizeof(message)-1, 0);
	if(strLen == -1)
	{
		ErrorHandling("read() error");
	}
	message[strLen] = 0;
	printf("Message from server : %s \n", message);
}

UINT WINAPI RobotVisClient::socketThread(LPVOID param){
	RobotVisClient *t_RobotVisClient = (RobotVisClient *)param;
	RobotState temp;
	char buf[1024];

	//���� �ʱ�ȭ
	if(WSAStartup(MAKEWORD(2, 2), &t_RobotVisClient->wsaData) != 0)
	{
		t_RobotVisClient->ErrorHandling("WSAStartup(), error");
	}

	// ���� ������ ���� ���� ����
	t_RobotVisClient->hSocket = socket(PF_INET, SOCK_STREAM, 0);
	if(t_RobotVisClient->hSocket == INVALID_SOCKET)
	{
		t_RobotVisClient->ErrorHandling("hSocketet(), error");
	}

	memset(&t_RobotVisClient->servAddr, 0, sizeof(t_RobotVisClient->servAddr));
	t_RobotVisClient->servAddr.sin_family = AF_INET;
	t_RobotVisClient->servAddr.sin_addr.s_addr = inet_addr(t_RobotVisClient->_IP);
	t_RobotVisClient->servAddr.sin_port = htons(t_RobotVisClient->_portNum);

	// ������ ���� ��û
	if(connect(t_RobotVisClient->hSocket, (SOCKADDR*) &t_RobotVisClient->servAddr, sizeof(t_RobotVisClient->servAddr)) == SOCKET_ERROR)
	{
		t_RobotVisClient->ErrorHandling("Connect() error");
		//���� ����
		t_RobotVisClient->DeInit();

		t_RobotVisClient->m_EndThread = true;
		return -1;
	}

	t_RobotVisClient->m_InitCheckThread = true;
	t_RobotVisClient->m_isSocketOpen = true;
	//MessageBox

	//���� ������
	while(t_RobotVisClient->m_EnableThread == true){
		// ������ ���� 
		int strLen = recv(t_RobotVisClient->hSocket, buf, sizeof(buf), 0);
		if(strLen == -1){
			t_RobotVisClient->ErrorHandling("read() error");
			t_RobotVisClient->m_EnableThread = false;
			t_RobotVisClient->m_isSocketOpen = false;
			break;
		}

		// ���� ������ ó��
		memcpy(&temp, buf, sizeof(RobotState));

		// ���� ������ ����
		EnterCriticalSection(&t_RobotVisClient->m_cs);
		memcpy(&t_RobotVisClient->robotData, &temp, sizeof(RobotState));
		t_RobotVisClient->isNewData = true;
		LeaveCriticalSection(&t_RobotVisClient->m_cs);

		// ������ ó�� ��� ����
		bool CollisionResult =t_RobotVisClient->CollisionCheck();										//������ ó�� ����� �޴� �Լ��� �����ؾ���
		char data = (char)CollisionResult;
		send(t_RobotVisClient->hSocket, &data, 1, 0);

	}

	//���� ����
	t_RobotVisClient->DeInit();

	t_RobotVisClient->m_EndThread = true;

	return 0;
}

int RobotVisClient::getData(RobotState *dst){
	if(!m_isSocketOpen){
		MessageBox(NULL, L"Socket not opened", L"Get Data Method Error", MB_OK);
		return -1;
	}

	int RetisNewData;

	EnterCriticalSection(&m_cs);
	RetisNewData = isNewData ? 1 : -1;
	memcpy(dst, &robotData, sizeof(RobotState));
	LeaveCriticalSection(&m_cs);

	return RetisNewData;
}

bool RobotVisClient::CollisionCheck(){
	while(isNewData){
		Sleep(20);
	}

	return isCollision;
}

void RobotVisClient::CalcCollision(bool result){
	isCollision = result;
	isNewData = false;
}