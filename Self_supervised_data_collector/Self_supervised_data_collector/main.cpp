#include <stdio.h>
#include <afx.h>
#include <time.h>
#include <stdlib.h>

#include "ARMSDK\include\ARMSDK.h"
#include "KinectMangerThread.h"
#include "Robot\RobotArm.h"
#include "InvalidMotionHandler.h"
#include "ColorBasedTracker.h"

#ifdef _DEBUG
#pragma comment(lib, "ARMSDKd.lib")
#endif
#ifdef NDEBUG
#pragma comment(lib, "ARMSDK.lib") 
#endif

#define DEFAULT_PATH "data"

void writeData(int *angle, cv::Mat img, const int count);
int WaitUntilMoveEnd(RobotArm *robot);
void ControllerInit(RobotArm *robot);
void CreateRGBDdir(const char* className);

int main(){
	//class 
	KinectMangerThread kinectManager;
	RobotArm arm;
	InvalidMotionHandler motionHandler;
	ColorBasedTracker tracker;

	//variable
	cv::Rect RobotROI((KINECT_DEPTH_WIDTH - 160) / 2, (KINECT_DEPTH_HEIGHT- 160) / 2, 160, 160);
	bool saveCheck = false;
	int sampleAngleBox[9], count = 0;
	const int sampleAngleLimit[9] = {10000, 10000, 10000, 10000, 5000, 5000, 400, 400, 400};
	char dirName[256];

	//initialize
	motionHandler.Initialize();
	kinectManager.Initialize(RobotROI);

	int presentSecond = time(NULL);
	srand(presentSecond);

#ifdef RIGHT_ARM_USE
	dxl_write_dword(arm.DXL_Get_Port(), 7, NX::P_HOMING_OFFSET_LL,  62750, 0);
#elif defined LEFT_ARM_USE
	dxl_write_dword(arm.DXL_Get_Port(), 8, NX::P_HOMING_OFFSET_LL, -62750, 0);
#endif

	ControllerInit(&arm);
	if(!motionHandler.robotConnect(&arm)){
		printf("Robot can not connect.\n");
		motionHandler.Deinitialize();
		return -1;
	}

	//arm.TorqueOff();
	printf("If u want to start program, press any key.\n");
	getch();

	//디렉토리 생성
	itoa(presentSecond, dirName, 10);
	CreateRGBDdir(dirName);

	while(!kinectManager.isThreadDead()){

		//샘플링된 모션이 가능한 모션인지를 체크
		int getAngle[9], tmpAngle[9];
		arm.GetPresPosition(getAngle);
		while(1){
			//angle sampling - limit 이내의 각도 샘플링
			memcpy(tmpAngle, getAngle, sizeof(int) * 9);
			for(int i = 0; i < NUM_XEL; i++){
				sampleAngleBox[i] = (rand() % sampleAngleLimit[i]) - sampleAngleLimit[i] / 2;
				tmpAngle[i] += sampleAngleBox[i];
			}
			//motion check
			if(motionHandler.InvalidCheck(tmpAngle))
				break;
		}

		//실제 움직임
		arm.SetGoalPosition(tmpAngle);
		WaitUntilMoveEnd(&arm);
		arm.GetPresPosition(getAngle);

		//write file
		//cv::Mat cropImage = KinectMappingImage(RobotROI);
		cv::Mat img = kinectManager.getImg();
		cv::imshow("cropImg", img);
		cv::waitKey(1);
		//writeData(getAngle, cropImage, count);
		count++;
		printf("[%d] data saveComplete.\n", count);
	}

	//Deallocation
	cv::destroyAllWindows();
	motionHandler.Deinitialize();
	kinectManager.Deinitialize();

	int Initpos[] = {0, 0, 0, 0, 0, 0, 2622, 1534, 3531};
	arm.SetGoalPosition(Initpos);
	WaitUntilMoveEnd(&arm);
	arm.TorqueOff();

	return 0;
}

void writeData(int *angle, cv::Mat img, const int count){
	char imgpath[256], anglepath[256];
	sprintf(imgpath, "%s\\%d.bmp", IMG_PATH, count);
	sprintf(anglepath, "%s\\%d.txt", ANGLE_PATH, count);
	cv::imwrite(imgpath, img);

	FILE *fp = fopen(anglepath, "w");
	fprintf(fp, "%d\t", count);
	for(int i = 0; i < NUM_XEL; i++)	fprintf(fp, "%d ", angle[i]);
	fclose(fp);
}

int isAllZero(int src[]){
	for(int i = 0; i < NUM_XEL; i++){
		if(src[i] != 0)
			return -1;
	}

	return 1;
}

int WaitUntilMoveEnd(RobotArm *robot){
	int checkTerm = 10;
	int presVel[NUM_XEL];
	int fingerLoad[NUM_FINGER];

	while(1){
		_sleep(33);
		robot->GetPresVelocity(presVel);
		robot->GetFingerLoad(fingerLoad);

		if(isAllZero(presVel) == 1)
			return 1;
	}
	return 1;
}

void ControllerInit(RobotArm *robot){
	int robotid[] = {1,3,5,7,9,11,13,15,17};
	int vel[] = {1000, 1000, 1000, 1000, 1000, 1000, 50, 50, 50};
	//Upper Left, UpperRight, Thumb

	int Initpos[] = {0, 0, 0, 0, 0, 0, 2622, 1534, 3531};

	robot->Init(6,3, robotid);

	robot->TorqueOff();
	robot->TorqueOn();

	robot->SetGoalVelocity(vel);

}

void CreateRGBDdir(const char* className){
	TCHAR szDir[MAX_PATH] = {0,};
	TCHAR RGBDDir[MAX_PATH] = {0,};
	TCHAR DepthDir[MAX_PATH] = {0,};
	TCHAR xyzDir[MAX_PATH] = {0,};
	char dirpath[256];
	sprintf(dirpath, "%s\\%s", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), szDir, MAX_PATH);
	bool mkdir_check = CreateDirectory(szDir, NULL);									//루트 디렉토리
	sprintf(dirpath, "%s\\%s\\RGB", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), RGBDDir, MAX_PATH);
	mkdir_check = CreateDirectory(RGBDDir, NULL);											//컬러 디렉토리
	sprintf(dirpath, "%s\\%s\\DEPTHMAP", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), DepthDir, MAX_PATH);
	mkdir_check = CreateDirectory(DepthDir, NULL);											//뎁스 디렉토리
	sprintf(dirpath, "%s\\%s\\XYZMAP", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), xyzDir, MAX_PATH);
	mkdir_check = CreateDirectory(xyzDir, NULL);
	sprintf(dirpath, "%s\\%s\\ANGLE", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), xyzDir, MAX_PATH);
	mkdir_check = CreateDirectory(xyzDir, NULL);
}