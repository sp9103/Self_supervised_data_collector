#include <stdio.h>
#include <afx.h>
#include <time.h>
#include <stdlib.h>

#include "ARMSDK\include\ARMSDK.h"
#include "KinectConnecter.h"
#include "Robot\RobotArm.h"
#include "InvalidMotionHandler.h"

#ifdef _DEBUG
#pragma comment(lib, "ARMSDKd.lib")
#endif
#ifdef NDEBUG
#pragma comment(lib, "ARMSDK.lib") 
#endif

void writeData(int *angle, cv::Mat img, const int count);
int WaitUntilMoveEnd(RobotArm *robot);
void ControllerInit(RobotArm *robot);

int main(){
	//class 
	KinectConnecter kinect;
	RobotArm arm;
	InvalidMotionHandler motionHandler;

	//variable
	cv::Mat KinectMappingImage;
	cv::Mat KinectDepthimage;
	cv::Mat KinectXYZImage;
	cv::Mat KinectColorImage;
	cv::Rect RobotROI((KINECT_DEPTH_WIDTH - 160) / 2, (KINECT_DEPTH_HEIGHT- 160) / 2, 160, 160);
	bool saveCheck = false;
	int sampleAngleBox[9], count = 0;
	const int sampleAngleLimit[9] = {100, 100, 100, 100, 50, 50, 20, 20, 20};

	//initialize
	kinect.KinectInitialize();
	motionHandler.Initialize();
	KinectColorImage.create(KINECT_COLOR_HEIGHT, KINECT_COLOR_WIDTH, CV_8UC4);			//Kinect Color Image format BGRA 4 channel image
	srand(time(NULL));

#ifdef RIGHT_ARM_USE
	dxl_write_dword(arm.DXL_Get_Port(), 7, NX::P_HOMING_OFFSET_LL,  62750, 0);
#elif defined LEFT_ARM_USE
	dxl_write_dword(arm.DXL_Get_Port(), 8, NX::P_HOMING_OFFSET_LL, -62750, 0);
#endif

	/*printf("If u want program start, press 's' button.\n");
	int temp = getch();
	if(temp != 's')
		return -1;*/

	ControllerInit(&arm);
	if(!motionHandler.robotConnect(&arm)){
		printf("Robot can not connect.\n");
		return -1;
	}

	while(1){
		char key = cv::waitKey(10);

		kinect.GetColorImage(&KinectColorImage);
		kinect.GetRGBDnDepthnXYZ(&KinectMappingImage, &KinectDepthimage, &KinectXYZImage);
		if(KinectMappingImage.cols != 0){
			cv::Mat tempMapImg = KinectMappingImage.clone();
			cv::rectangle(tempMapImg, RobotROI, cv::Scalar(0,255,0));
			imshow("KinectMapFrame", tempMapImg);
		}

		if(key == 27)
			break;
		else if(key == 's')		saveCheck = !saveCheck;

		//실제 저장부
		if(saveCheck){
			//샘플링된 모션이 가능한 모션인지를 체크
			int getAngle[9], tmpAngle[9];
			arm.GetPresPosition(getAngle);
			while(1){
				//angle sampling - limit 이내의 각도 샘플링
				for(int i = 0; i < NUM_XEL; i++){
					sampleAngleBox[i] = rand() % sampleAngleLimit[i];
					tmpAngle[i] = sampleAngleBox[i] + getAngle[i];
				}
				//motion check
				if(motionHandler.InvalidCheck(tmpAngle))
					break;
			}

			//실제 움직임
			arm.SetGoalPosition(tmpAngle);
			WaitUntilMoveEnd(&arm);

			//write file
			cv::Mat cropImage = KinectMappingImage(RobotROI);
			writeData(getAngle, cropImage, count);
			count++;
			printf("[%d] data saveComplete.\n");
		}
	}

	//Deallocation
	kinect.KinectDestroy();
	cv::destroyAllWindows();
	motionHandler.Deinitialize();

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
	//int Initpos[] = {0, 0, 0, 0, 0, 0, 2268, 2452, 1625};

	robot->Init(6,3, robotid);

	robot->TorqueOff();
	robot->TorqueOn();

	robot->SetGoalVelocity(vel);
	//robot->SetGoalPosition(Initpos);
	//robot->SetFingerPosition(&Initpos[6]);
}