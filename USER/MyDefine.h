#ifndef __MYDEF_H
#define __MYDEF_H

#include "sys.h"



#define DEBUG_MODE 0							//�Ƿ����ⲿ��������ģʽ
																	//����ʱ�رգ���ʡϵͳ��Դ
//ʾ����
//#if DEBUG_MODE
//************
//#endif



#define ABS(x)	( (x>0) ? (x) : (-x) )


/***********************************----pid.c define----*****************************************/

#define ROBUST_P 120
#define ROBUST_D 0.5
#define ROBUST_MER 150
#define ROBUST_DEAD 0


////////////////////////////////////////////////////////////////////////////////////////


/********************************---led.c define----****************************************/
#define LIGHT 0
#define LIGHT_OUT 1


extern u8 LED_PointStatu;

//
// 
/**************************-----pwm.c define-----******************************/
#define SERVO_CYCLE 20000		//84M/84=1Mhz�ļ���Ƶ��,��װ��ֵ20000������PWMƵ��Ϊ 1M/20000=50hz.  ��20ms����
#define ESC_CYCLE 2500			//2.5ms����


/******************************************************************************/



//
//
/**************************-----control.c define----***************************/

/**********�������************/
#define STEP1 PFout(10)	//�������1����������˿�
#define DIR1 PIout(9)	//�������1���������
#define STEP2 PAout(5)	//�������2����������˿�
#define DIR2 PAout(4)	//�������2���������

#define DIR_PLUS 1	//��������ƽ����
#define DIR_MINUS 0	//���ط����ƽ����


#define STEPMOTOR_STARTSPEED 5	//������������ٶ�

#define STEP1_LIMIT_DISTANCE 24000	//���� X�Ჽ����������г�
#define STEP2_LIMIT_DISTANCE 6000	//���� Z�Ჽ����������г�
#define STEP1_CONTACT_DISTANCE 22000//���� �Ӵ��г�
#define STEP2_CONTACT_DISTANCE 3500//���� �Ӵ��г�

/**********�������***********/
#define FALL 15
#define ISLAND 670
#define BULLET 770	//?����

/************���***********/
#define VIDEO_UP 1500	//500
#define VIDEO_BACK 600	//2500

#define OPEN_LEFT 500
#define OPEN_RIGHT 1450
#define OPEN_BACK 2500	//����	//���2500&1500��
#define CLOSE_LEFT 1500
#define CLOSE_RIGHT 600
#define CLOSE_BACK 1800	//����

/***********Ħ����************/
#define FRICTION_OPEN 1750	//�������Ų���
#define FRICTION_CLOSE 1000   

/************����************/
#define FOREROKE 380	//ǰ����б��ֵ
#define HYPSOKINESIS -580

#define CYCLE_LIFTRAMP 10	//���㷽��CYCLE=CYCLE_LIFTRAMP*100*time��ʱ��Ƶ��

typedef struct
{
	u32 cycle;
	u8 count;	//�ۼӼ���
	float calc;	//����ϵ��0-1
	s32 last_value;	//Ϊ����������ӵļ�¼ֵ
} RampTypeDef;		//���½ṹ��


typedef struct
{
	s32 position_val;	//��¼��ǰ�������λ��ֵ
	u8 dir_set;
	u32 step_count;	//������������������¼��ǰ��������
	u32 step_set;
	u8 cycle_set;
	u8 step_complete;//��־һ������������
} StepMotorTypeDef;		//��������ṹ��

extern StepMotorTypeDef StepMotor1;
extern StepMotorTypeDef StepMotor2;


extern RampTypeDef Step1Ramp;
extern RampTypeDef Step2Ramp;
extern RampTypeDef CM12_Ramp;
extern RampTypeDef CM34_Ramp;

//extern u8 dir_set;
//extern u32 step_count;
//extern u32 step_set;
//extern u8 cycle_set;
//extern u8 step_complete;


extern s32 stepmotor1_position_send;
extern u8 stepmotor1_cycle_send;
extern s32 stepmotor2_position_send;
extern u8 stepmotor2_cycle_send;
/******************************************************************************/


/********************************--imu.c define---**********************************/
#define FILTERING_RANGE 8

typedef struct
{
	s32 ax_sum[FILTERING_RANGE];
	s32 ay_sum[FILTERING_RANGE];
	s32 az_sum[FILTERING_RANGE];
	u8 i;
	s32 ax_calc;
	s32 ay_calc;
	s32 az_calc;
	u8 count;	//����
} ImuTypeDef;		//���������ݴ���ṹ��

extern ImuTypeDef Imu_Deal;

////////////////////////////////////////////////////////////////////////////////////



/**************************-----timer.c define------***************************/

extern u32 TimeCount_Common;	//ȫ������ʱ�����

////////////////////////////////////////////////////////////////////////////////


/******************************-----keyboard.c define-----************************************************/

typedef struct
{
	u16 count;
	u8 value;
	u8 last;
	u8 statu;
}KeyBoardTypeDef;


#define Y_RANGE 3

typedef struct
{
	u32 time_record;
	u8 sendlast;
	u8 statu;
	u8 send[Y_RANGE];
	u8 circulation_i;
	u8 send_sign;
	u8 depth;
}TakeBulletTypeDef;


#define STEPMOTOR_DELAY 0
#define STEPMOTOR1_EXTEND 1
#define STEPMOTOR1_BACK 2
#define STEPMOTOR2_EXTEND 3
#define STEPMOTOR_BACK 4
#define DELAY_TIME 1000


extern u8 MainBoard_SendDate;
extern u8 MainBoard_SendBuffer_L;
extern u8 MainBoard_SendBuffer_H;

extern TakeBulletTypeDef Bullet;

extern KeyBoardTypeDef Key1;
extern KeyBoardTypeDef Key2;
extern KeyBoardTypeDef Key3;
extern KeyBoardTypeDef Key4;
extern KeyBoardTypeDef Key5;
extern KeyBoardTypeDef Key6;
extern KeyBoardTypeDef Key7;
extern KeyBoardTypeDef KeyQ;

extern u8 KeyValue[8];
extern u8 TakBult_STPCM;
////////////////////////////////////////////////////////////////////////////////////////////////////////


/***********************--����״̬--**********************/
typedef enum
{
		
    PREPARE_STATE,     		//�ϵ���ʼ��״̬ 4s������
 //   STANDBY_STATE,			//��ֹ̨ͣ��ת״̬
    NORMAL_STATE,			//��������״̬
 //   STOP_STATE,        	//ֹͣ�˶�״̬
    CALI_STATE,    			//У׼״̬
}WorkState_e;

extern WorkState_e workState;
void SetWorkState(WorkState_e state);
WorkState_e GetWorkState(void);
/*
WorkState_e workState = PREPARE_STATE;

static void SetWorkState(WorkState_e state)
{
    workState = state;
}


WorkState_e GetWorkState()
{
	return workState;
}

//ʹ��ʾ��
if(GetWorkState()==PREPARE_STATE) //�����׶Σ����̲���ת
	{
		ChassisSpeedRef.rotate_ref = 0;	 
	}
*/	
	
	
	
#endif

