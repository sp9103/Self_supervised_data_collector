#include <stdio.h>
#include <afx.h>
#include <time.h>
#include <stdlib.h>
#include <random>

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

bool writeData(cv::Mat RGBimg, cv::Mat DEPTHimg, cv::Mat pointCloud, ColorBasedTracker *cbTracker, int* angle, char* path, const int count);
int WaitUntilMoveEnd(RobotArm *robot);
void ControllerInit(RobotArm *robot);
void CreateRGBDdir(const char* className);
void writeDepthData(cv::Mat src, char* path, char* name);
int compareMaxSubPos(int *goal, int *pres);

int main(){
	//class 
	KinectMangerThread kinectManager;
	RobotArm arm;
	InvalidMotionHandler motionHandler;
	ColorBasedTracker tracker;

	//variable
	cv::Rect RobotROI((KINECT_DEPTH_WIDTH - 160) / 2 + 40, (KINECT_DEPTH_HEIGHT- 160) / 2, 160, 160);
	bool saveCheck = false;
	int sampleAngleBox[9], count = 0;
	const int sampleAngleLimit[9] = {10000, 10000, 10000, 10000, 10000, 10000, 400, 400, 400};
	char dirName[256];
	float averBox[NUM_XEL] = {0};

	//uniform random sampler
	const int range_from  = 0;
	const int range_to    = 10;
	std::random_device                  rand_dev;
	std::mt19937                        generator(rand_dev());
	std::uniform_int_distribution<int>  distr_arm(-5000, 5000);
	std::uniform_int_distribution<int>  distr_finger(-200, 200);

	//initialize
	kinectManager.Initialize(RobotROI);
	motionHandler.Initialize();

	int presentSecond = time(NULL);

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

	arm.TorqueOff();
	printf("If u want to start program, press any key.\n");
	getch();


	//디렉토리 생성
	itoa(presentSecond, dirName, 10);
	CreateRGBDdir(dirName);

	//배경취득
	cv::Mat backRGB = kinectManager.getImg();
	cv::Mat backDepth = kinectManager.getDepth();
	cv::imshow("background", backRGB);
	cv::waitKey(1);
	char buf[256];
	strcpy(buf, DEFAULT_PATH);
	strcat(buf, dirName);
	sprintf(buf, "%s\\%s", DEFAULT_PATH, dirName);
	writeDepthData(backDepth, buf, "backDepth");
	strcat(buf, "\\backRGB.bmp");
	cv::imwrite(buf, backRGB);
	tracker.InsertBackGround(backRGB, backDepth);

	printf("\nIf u save background, press any key\n");
	getch();
	arm.TorqueOn();

	while(!kinectManager.isThreadDead()){

		//샘플링된 모션이 가능한 모션인지를 체크
		int getAngle[9], tmpAngle[9], completePosition[9];

		//motionHandler.ForwardEnd(&arm);

		printf("Sampling start....");
		while(1){
			arm.GetPresPosition(getAngle);
			//angle sampling - limit 이내의 각도 샘플링
			memcpy(tmpAngle, getAngle, sizeof(int) * 9);
			for(int i = 0; i < NUM_XEL; i++){
				if(i < NUM_JOINT) sampleAngleBox[i] = distr_arm(generator);
				else				sampleAngleBox[i] = distr_finger(generator);
				tmpAngle[i] += sampleAngleBox[i];
			}
			//motion check
			if(motionHandler.InvalidCheck(tmpAngle, getAngle))
				break;
		}
		printf("complete!\n");
		printf("sampling result : \n");
		for(int i = 0; i < NUM_XEL; i++)	printf("%d ", sampleAngleBox[i]);
		printf("\n");

		//실제 움직임
		printf("Move Robot.....");
		arm.SetGoalPosition(tmpAngle);
		WaitUntilMoveEnd(&arm);
		arm.GetPresPosition(completePosition);
		printf("complete!\n");

		//write file
		cv::Mat img = kinectManager.getImg();
		cv::Mat depth = kinectManager.getDepth();
		cv::Mat pointCloud = kinectManager.getPointCloud();
		cv::imshow("cropImg", img);
		cv::waitKey(1);
		if(writeData(img, depth, pointCloud, &tracker, completePosition, dirName, count)){
			count++;
			printf("[%d] data saveComplete.\n", count);
			printf("average : ");
			for(int i = 0; i < NUM_XEL; i++){
				averBox[i] = (averBox[i] * (count - 1) + sampleAngleBox[i]) / (float)count;
				printf("%.1f ", averBox[i]);
			}
			printf("\n");
		}else{
			printf("Move previous position.....");
			arm.SetGoalPosition(getAngle);
			WaitUntilMoveEnd(&arm);
			printf("complete!\n");
		}
	}

	//Deallocation
	cv::destroyAllWindows();
	motionHandler.Deinitialize();
	kinectManager.Deinitialize();

	printf("Go End position.....");
	int Initpos[] = {0, 0, 0, 0, 0, 0, 2622, 1534, 2100};
	arm.SetGoalPosition(Initpos);
	WaitUntilMoveEnd(&arm);
	printf("complete!\n");
	arm.TorqueOff();

	return 0;
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
	int goalPos[NUM_XEL], presPos[NUM_XEL];

	while(1){
		_sleep(33);
		robot->GetPresVelocity(presVel);
		robot->GetGoalPosition(goalPos);
		robot->GetPresPosition(presPos);

		int maxSub = compareMaxSubPos(goalPos, presPos);
		//robot->GetFingerLoad(fingerLoad);
		if(isAllZero(presVel) == 1 || maxSub < 10)
			return 1;
	}
	return 1;
}

void ControllerInit(RobotArm *robot){
	int robotid[] = {1,3,5,7,9,11,13,15,17};
	int vel[] = {1000, 1000, 1000, 1000, 1000, 1000, 50, 50, 50};
	//Upper Left, UpperRight, Thumb

	int Initpos[] = {0, 0, 0, 0, 0, 0, 2622, 1534, 2100};

	robot->Init(6,3, robotid);

	robot->TorqueOff();
	robot->TorqueOn();

	robot->SetGoalVelocity(vel);
	//robot->SetGoalPosition(Initpos);
}

void CreateRGBDdir(const char* className){
	TCHAR szDir[MAX_PATH] = {0,};
	TCHAR RGBDDir[MAX_PATH] = {0,};
	TCHAR DepthDir[MAX_PATH] = {0,};
	TCHAR xyzDir[MAX_PATH] = {0,};
	char dirpath[256];
	sprintf(dirpath, "%s\\%s\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), szDir, MAX_PATH);
	bool mkdir_check = CreateDirectory(szDir, NULL);									//루트 디렉토리
	sprintf(dirpath, "%s\\%s\\RGB\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), RGBDDir, MAX_PATH);
	mkdir_check = CreateDirectory(RGBDDir, NULL);											//컬러 디렉토리 - 원본
	sprintf(dirpath, "%s\\%s\\ANGLE\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), xyzDir, MAX_PATH);
	mkdir_check = CreateDirectory(xyzDir, NULL);											//Angle
	sprintf(dirpath, "%s\\%s\\DEPTHMAP\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), DepthDir, MAX_PATH);
	mkdir_check = CreateDirectory(DepthDir, NULL);											//뎁스 디렉토리 - 원본
	sprintf(dirpath, "%s\\%s\\XYZMAP\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), xyzDir, MAX_PATH);
	mkdir_check = CreateDirectory(xyzDir, NULL);											//포인트 클라우드 디렉토리 - 원본
	sprintf(dirpath, "%s\\%s\\BACKGROUND\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), xyzDir, MAX_PATH);
	mkdir_check = CreateDirectory(xyzDir, NULL);
	sprintf(dirpath, "%s\\%s\\PROCESSIMG\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), xyzDir, MAX_PATH);
	mkdir_check = CreateDirectory(xyzDir, NULL);	
}

bool writeData(cv::Mat RGBimg, cv::Mat DEPTHimg, cv::Mat pointCloud, ColorBasedTracker *cbTracker, int* angle, char* path, const int count){
	cv::Mat processImg = cbTracker->calcImage(RGBimg, DEPTHimg);
	if(processImg.rows == 0)	return false;

	char pathBuf[256], buf[256], id[256];
	sprintf(pathBuf, "%s\\%s", DEFAULT_PATH, path);
	itoa(count, id, 10);

	//store RGB
	sprintf(buf, "%s\\RGB\\%d.bmp", pathBuf, count);
	cv::imwrite(buf, RGBimg);
	//store Depth
	sprintf(buf, "%s\\DEPTHMAP", pathBuf);
	writeDepthData(DEPTHimg, buf, id);
	//store Angle
	sprintf(buf, "%s\\ANGLE\\%d.txt", pathBuf, count);
	FILE *fp = fopen(buf, "w");
	for(int i = 0; i < NUM_XEL; i++)	fprintf(fp, "%d\n", angle[i]);
	fclose(fp);
	//store Process Img
	sprintf(buf, "%s\\PROCESSIMG\\%d.bmp", pathBuf, count);
	cv::imwrite(buf, processImg);
	cv::imshow("Process Img", processImg);
	cv::waitKey(1);
	//store point cloud
	sprintf(buf, "%s\\XYZMAP\\%d.bin", pathBuf, count);
	fp = fopen(buf, "wb");
	fwrite(&pointCloud.rows, sizeof(int), 1, fp);
	fwrite(&pointCloud.cols, sizeof(int), 1, fp);
	int Type = pointCloud.type();
	fwrite(&Type, sizeof(int), 1, fp);
	for(int i = 0; i < pointCloud.rows * pointCloud.cols; i++)
		for(int c = 0; c < pointCloud.channels(); c++)
			fwrite(&pointCloud.at<Vec3f>(i)[c], sizeof(float), 1, fp);
	fclose(fp);

	return true;
}

void writeDepthData(cv::Mat src, char* path, char* name){
	//Depth Infomation write
	char buf[256];
	sprintf(buf, "%s\\%s.bin", path, name);
	FILE *fp = fopen(buf, "wb");
	fwrite(&src.rows, sizeof(int), 1, fp);
	fwrite(&src.cols, sizeof(int), 1, fp);
	int Type = src.type();
	fwrite(&Type, sizeof(int), 1, fp);
	for(int i = 0; i < src.rows * src.cols; i++)		fwrite(&src.at<float>(i), sizeof(float), 1, fp);
	fclose(fp);
}

int compareMaxSubPos(int *goal, int *pres){
	int maxSub = -1;

	for(int i = 0; i < NUM_XEL; i++){
		int sub = abs(goal[i] - pres[i]);

		if(maxSub < sub)	maxSub = sub;
	}

	return maxSub;
}