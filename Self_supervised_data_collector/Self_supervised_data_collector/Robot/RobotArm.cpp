#include "RobotArm.h"


RobotArm::RobotArm(void)
{
	sp_.dPacketStartTime = 0;
	sp_.fByteTransferTime = 0;
	sp_.fPacketWaitTime = 0;
	sp_.hComm = 0;
	sp_.iBusUsing = 0;

	Port_ = &sp_;
}


RobotArm::~RobotArm(void)
{
}

int RobotArm::Init(int PortNum, int BaudRateNum, int *ID_list){
	SetID(ID_list);

	Com_port_num_ = PortNum;
	Baud_rate_num_ = BaudRateNum;

	if(dxl_initialize(Port_, Com_port_num_, Baud_rate_num_) == COMM_RXSUCCESS )
		printf("Succeed to open USB2Dynamixel!\n");
	else
	{
		printf( "Failed to open USB2Dynamixel!\n" );
		return -1;
	}

	fingercontroller_.Init(sp_, fingerID_);
	jointcontroller_.Init(sp_, jointID_);

	return 1;
}

int RobotArm::DeInit(){

	dxl_terminate(Port_);

	return 1;
}

int RobotArm::SetID(int *ID_list){
	//각각 아이디 부여
	int i;
	for(i = 0; i < NUM_JOINT; i++)
		jointID_[i] = ID_list[i];
	for(int j = 0; j < NUM_FINGER; j++)
		fingerID_[j] = ID_list[i++];

	return 1;
}

int RobotArm::TorqueOn(){
	fingercontroller_.TorqueOn();
	jointcontroller_.TorqueOn();

	printf("=================Torque ON====================\n");
	return 1;
}

int RobotArm::TorqueOff(){
	fingercontroller_.TorqueOff();
	jointcontroller_.TorqueOff();

	printf("=================Torque OFF====================\n");
	return 1;
}

int RobotArm::GetTemperature(int *Temperature){
	if(!Temperature){
		printf("Present Temperature\n");
		jointcontroller_.GetTemperature();
		fingercontroller_.GetTemperature();
	}else{
		jointcontroller_.GetTemperature(Temperature);
		fingercontroller_.GetTemperature(&Temperature[NUM_JOINT]);
	}

	return 1;
}

int RobotArm::isMoving(bool *ismove){
	if(!ismove){
		printf("Moving State\n");
		jointcontroller_.isMoving();
		fingercontroller_.isMoving();
	}else{
		jointcontroller_.isMoving(ismove);
		fingercontroller_.isMoving(&ismove[NUM_JOINT]);
	}

	return 1;
}

int RobotArm::GetPresPosition(int *PresentPosition){
	if(!PresentPosition){
		printf("Present position\n");
		jointcontroller_.GetPresPosition();
		fingercontroller_.GetPresPosition();
	}else{
		jointcontroller_.GetPresPosition(PresentPosition);
		fingercontroller_.GetPresPosition(&PresentPosition[NUM_JOINT]);
	}

	return 1;
}

int RobotArm::GetGoalPosition(int *GoalPosition){
	if(!GoalPosition){
		printf("Goal position\n");
		jointcontroller_.GetGoalPosition();
		fingercontroller_.GetGoalPosition();
	}else{
		jointcontroller_.GetGoalPosition(GoalPosition);
		fingercontroller_.GetGoalPosition(&GoalPosition[NUM_JOINT]);
	}

	return 1;
}

int RobotArm::GetPresVelocity(int *PresentVelocity){
	if(!PresentVelocity){
		printf("Present velocity\n");
		jointcontroller_.GetPresVelocity();
		fingercontroller_.GetPresVelocity();
	}else{
		jointcontroller_.GetPresVelocity(PresentVelocity);
		fingercontroller_.GetPresVelocity(&PresentVelocity[NUM_JOINT]);
	}

	return 1;
}
int RobotArm::GetGoalVelocity(int *GoalVelocity){
	if(!GoalVelocity){
		printf("Goal velocity\n");
		jointcontroller_.GetGoalVelocity();
		fingercontroller_.GetGoalVelocity();
	}else{
		jointcontroller_.GetGoalVelocity(GoalVelocity);
		fingercontroller_.GetGoalVelocity(&GoalVelocity[NUM_JOINT]);
	}

	return 1;
}

int RobotArm::SetGoalVelocity(int *GoalVelocity){
	if(!jointcontroller_.SetGoalVelocity(GoalVelocity))					return -1;
	if(!fingercontroller_.SetGoalVelocity(&GoalVelocity[NUM_JOINT]))	return -1;

	return 1;
}

int RobotArm::SetGoalPosition(int *GoalPosition){
	if(!jointcontroller_.SetGoalPosition(GoalPosition))					return -1;
	if(!fingercontroller_.SetGoalPosition(&GoalPosition[NUM_JOINT]))	return -1;

	return 1;
}

int RobotArm::SetLED(bool onoff){
	unsigned char LEDPRO[NUM_JOINT];
	unsigned char LEDMX[NUM_FINGER];

	unsigned char onoffval = onoff == true ? 255 : 0;

	for(int i = 0; i < NUM_JOINT; i++)
		LEDPRO[i] = onoffval;
	for(int i = 0; i < NUM_FINGER; i++)
		LEDMX[i] = onoffval;

	if(!jointcontroller_.SetLED(LEDPRO))					return -1;
	if(!fingercontroller_.SetLED(LEDMX))					return -1;

	return 1;
}

int RobotArm::GetFingerLoad(int *load){
	if(!load){
		printf("present Load\n");
		fingercontroller_.GetPresLoad();
	}else{
		jointcontroller_.GetPresCurrent();
		fingercontroller_.GetPresLoad(load);
	}

	return 1;
}

int RobotArm::SetFingerPosition(int *GoalPosition){
	if(!fingercontroller_.SetGoalPosition(GoalPosition))					return -1;
}

int RobotArm::Arm_Get_JointValue(Eigen::VectorXi *value)
{
	value->resize(NUM_JOINT);

	int temp_val[9];
	GetPresPosition(temp_val);

	for(int i = 0; i < NUM_JOINT; i++){
		(*value)[i] = temp_val[i];
	}

	return 1;
}

SerialPort* RobotArm::DXL_Get_Port(void)
{
	return Port_;
}