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
	//Thread가 정상 종료될때까지 기다림
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
		// ip address 파악
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
	// 연결 종료
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

void RobotVisClient::Recv(){								//추후 구조체
	char message[256];

	// 데이터 수신 
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

	//소켓 초기화
	if(WSAStartup(MAKEWORD(2, 2), &t_RobotVisClient->wsaData) != 0)
	{
		t_RobotVisClient->ErrorHandling("WSAStartup(), error");
	}

	// 서버 접속을 위한 소켓 생성
	t_RobotVisClient->hSocket = socket(PF_INET, SOCK_STREAM, 0);
	if(t_RobotVisClient->hSocket == INVALID_SOCKET)
	{
		t_RobotVisClient->ErrorHandling("hSocketet(), error");
	}

	memset(&t_RobotVisClient->servAddr, 0, sizeof(t_RobotVisClient->servAddr));
	t_RobotVisClient->servAddr.sin_family = AF_INET;
	t_RobotVisClient->servAddr.sin_addr.s_addr = inet_addr(t_RobotVisClient->_IP);
	t_RobotVisClient->servAddr.sin_port = htons(t_RobotVisClient->_portNum);

	// 서버로 연결 요청
	if(connect(t_RobotVisClient->hSocket, (SOCKADDR*) &t_RobotVisClient->servAddr, sizeof(t_RobotVisClient->servAddr)) == SOCKET_ERROR)
	{
		t_RobotVisClient->ErrorHandling("Connect() error");
		//소켓 정리
		t_RobotVisClient->DeInit();

		t_RobotVisClient->m_EndThread = true;
		return -1;
	}

	t_RobotVisClient->m_InitCheckThread = true;
	t_RobotVisClient->m_isSocketOpen = true;
	//MessageBox

	//소켓 구동부
	while(t_RobotVisClient->m_EnableThread == true){
		// 데이터 수신 
		int strLen = recv(t_RobotVisClient->hSocket, buf, sizeof(buf), 0);
		if(strLen == -1){
			t_RobotVisClient->ErrorHandling("read() error");
			t_RobotVisClient->m_EnableThread = false;
			t_RobotVisClient->m_isSocketOpen = false;
			break;
		}

		// 수신 데이터 처리
		memcpy(&temp, buf, sizeof(RobotState));

		// 수신 데이터 복사
		EnterCriticalSection(&t_RobotVisClient->m_cs);
		memcpy(&t_RobotVisClient->robotData, &temp, sizeof(RobotState));
		t_RobotVisClient->isNewData = true;
		LeaveCriticalSection(&t_RobotVisClient->m_cs);

		// 데이터 처리 결과 전송
		bool CollisionResult =t_RobotVisClient->CollisionCheck();										//데이터 처리 결과를 받는 함수를 구현해야함
		char data = (char)CollisionResult;
		send(t_RobotVisClient->hSocket, &data, 1, 0);

	}

	//소켓 정리
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