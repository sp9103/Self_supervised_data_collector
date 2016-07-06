#include "KinectMangerThread.h"


KinectMangerThread::KinectMangerThread(void)
{
	endCheck = false;
	loopClose = false;
}


KinectMangerThread::~KinectMangerThread(void)
{

}

void KinectMangerThread::Initialize(cv::Rect srcROI){
	InitializeCriticalSection(&cs);
	
	imgROI = srcROI;

	HANDLE _TThreadHandle = (HANDLE)_beginthreadex(NULL, 0, KinectThread, this, 0, NULL);
}

void KinectMangerThread::Deinitialize(){
	loopClose = true;

	while(!endCheck)	Sleep(10);

	DeleteCriticalSection(&cs);
}

cv::Mat KinectMangerThread::getImg(){
	cv::Mat retMat;

	EnterCriticalSection(&cs);
	retMat = frame_;
	LeaveCriticalSection(&cs);

	return retMat;
}

cv::Mat KinectMangerThread::getDepth(){
	cv::Mat retMat;

	EnterCriticalSection(&cs);
	retMat = depth_;
	LeaveCriticalSection(&cs);

	return retMat;
}

cv::Mat KinectMangerThread::getPointCloud(){
	cv::Mat retMat;

	EnterCriticalSection(&cs);
	retMat = pointCloud_;
	LeaveCriticalSection(&cs);

	return retMat;
}

UINT WINAPI KinectMangerThread::KinectThread(LPVOID param){
	KinectMangerThread* p = (KinectMangerThread*)param;

	KinectConnecter kinect;

	cv::Mat KinectMappingImage;
	cv::Mat KinectDepthimage;
	cv::Mat KinectXYZImage;
	cv::Mat KinectColorImage;

	kinect.KinectInitialize();
	KinectColorImage.create(KINECT_COLOR_HEIGHT, KINECT_COLOR_WIDTH, CV_8UC4);			//Kinect Color Image format BGRA 4 channel image

	while(1){
		char key = cv::waitKey(10);

		kinect.GetColorImage(&KinectColorImage);
		kinect.GetRGBDnDepthnXYZ(&KinectMappingImage, &KinectDepthimage, &KinectXYZImage);
		if(KinectMappingImage.cols != 0){
			cv::Mat tempMapImg = KinectMappingImage.clone();

			EnterCriticalSection(&p->cs);
			p->frame_ = tempMapImg(p->imgROI).clone();
			p->depth_ = KinectDepthimage(p->imgROI).clone();
			p->pointCloud_ = KinectXYZImage(p->imgROI).clone();
			LeaveCriticalSection(&p->cs);

			cv::rectangle(tempMapImg, p->imgROI, cv::Scalar(0,255,0));
			imshow("KinectMapFrame", tempMapImg);
		}

		if(p->loopClose || key == 27)
			break;
	}

	p->endCheck = true;
	p->loopClose = true;

	kinect.KinectDestroy();
	cv::destroyAllWindows();

	printf("Kinect Thread safely terminated.\n");

	return 1;
}

bool KinectMangerThread::isThreadDead(){
	return loopClose;
}