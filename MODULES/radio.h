#ifndef __RADIO_H
#define __RADIO_H
#include "stdint.h"
#include "stdbool.h"
#include "sys.h"


#define VERSION_CNT     100     //����λ��100msһ�� ������Ӳ���汾��Ϣ     ������
#define SENSOR_CNT      10      //����λ��10msһ�� ���ʹ�����ԭʼ����      ������
#define STATUS_CNT      10      //����λ��10msһ�� ������̬���߶�����
#define RCDATA_CNT      20      //����λ��20msһ�� ����ң��������ԭʼͨ������
#define MOTORPWM_CNT    20      //����λ��20msһ�� ���͵�����յ�PWM����
#define POWER_CNT       50      //����λ��50msһ�� ���͵�����ѹ����        �ð汾Ӳ��δʵ�֣�������













/*����֡ͷ*/
#define UP_BYTE1 0xAA
#define UP_BYTE2 0xAA

/*����֡ͷ*/
#define DOWN_BYTE1 0xAA
#define DOWN_BYTE2 0xAF

#define ATKP_MAX_DATA_SIZE 30
#define ATKP_RX_QUEUE_SIZE 10

/*ͨѶ���ݽṹ*/
typedef struct
{
	uint8_t msgID;
	uint8_t dataLen;
	uint8_t data[ATKP_MAX_DATA_SIZE];
}atkp_t;


/*����ָ��ID ��Ӧ�ŷɿ�->��λ��������֡�Ĺ�����*/
typedef enum 
{
	UP_VERSION		= 0x00,
	UP_STATUS		= 0x01,
	UP_SENSER		= 0x02,
	UP_RCDATA		= 0x03,
	UP_GPSDATA		= 0x04,
	UP_POWER		= 0x05,
	UP_MOTOR		= 0x06,
	UP_SENSER2		= 0x07,
	UP_FLYMODE		= 0x0A,
	UP_SPEED 		= 0x0B,
	UP_PID1			= 0x10,
	UP_PID2			= 0x11,
	UP_PID3			= 0x12,
	UP_PID4			= 0x13,
	UP_PID5			= 0x14,
	UP_PID6			= 0x15,
	UP_RADIO		= 0x40,
	UP_MSG			= 0xEE,
	UP_CHECK		= 0xEF,
	
	UP_REMOTER		= 0x50,
	UP_PRINTF		= 0x51,
	
	UP_USER_DATA1	= 0xF1,
	UP_USER_DATA2	= 0xF2,
	UP_USER_DATA3	= 0xF3,
	UP_USER_DATA4	= 0xF4,
	UP_USER_DATA5	= 0xF5,
	UP_USER_DATA6	= 0xF6,
	UP_USER_DATA7	= 0xF7,
	UP_USER_DATA8	= 0xF8,
	UP_USER_DATA9	= 0xF9,
	UP_USER_DATA10	= 0xFA,
}upmsgID_e;


/*����ָ�� ��Ӧ����λ��->ң�ع�����Ϊ01������֡�е�CMD1������
�������Ϊ����λ���з���У׼ָ��*/
#define  D_COMMAND_ACC_CALIB		0x01    //ACCУ׼
#define  D_COMMAND_GYRO_CALIB		0x02    //GYROУ׼
#define  D_COMMAND_MAG_CALIB		0x04    //MAGУ׼
#define  D_COMMAND_BARO_CALIB		0x05    //BAROУ׼
#define  D_COMMAND_ACC_CALIB_EXIT	0x20    //�˳�6��У׼
#define  D_COMMAND_ACC_CALIB_STEP1	0x21    //����У׼��һ��
#define  D_COMMAND_ACC_CALIB_STEP2	0x22    //����У׼�ڶ���
#define  D_COMMAND_ACC_CALIB_STEP3	0x23    //����У׼������
#define  D_COMMAND_ACC_CALIB_STEP4	0x24    //����У׼���Ĳ�
#define  D_COMMAND_ACC_CALIB_STEP5	0x25    //����У׼���岽
#define  D_COMMAND_ACC_CALIB_STEP6	0x26    //����У׼������
#define  D_COMMAND_FLIGHT_LOCK		0xA0    //�ɿ����� �������ֻ���������
#define  D_COMMAND_FLIGHT_ULOCK		0xA1    //�ɿؽ��� �������ֻ���������

/*����ָ�� ��Ӧ����λ��->ң�ع�����Ϊ02������֡�е�CMD2������
�������Ϊ����λ���з��� ����ɿص�ǰ���ݵ�ָ��*/
#define  D_ACK_READ_PID				0x01    //��ȡPID���� ����AAAA 10/11/12/13/14/15 ����֡

#define  D_ACK_READ_MODE			0x02    //��ȡ����ģʽ�������� ����AAAA 0A����֡
#define  D_ACK_READ_WAYPOINT		0x21    //��ȡ�ɿ��ں������� ����AAAA 20����֡
#define  D_ACK_READ_VERSION			0xA0    //��ȡ��λ���汾��Ϣ
#define  D_ACK_RESET_PARAM			0xA1    //�ָ�Ĭ�ϲ���
/*����ָ��ID ������*/
typedef enum 
{
	DOWN_COMMAND	= 0x01,
	DOWN_ACK		= 0x02,
	DOWN_RCDATA		= 0x03,
	DOWN_POWER		= 0x05,
	DOWN_FLYMODE	= 0x0A,
	DOWN_PID1		= 0x10,
	DOWN_PID2		= 0x11,
	DOWN_PID3		= 0x12,
	DOWN_PID4		= 0x13,
	DOWN_PID5		= 0x14,
	DOWN_PID6		= 0x15,
	DOWN_RADIO		= 0x40,
	
	DOWN_REMOTER	= 0x50,
}downmsgID_e;


typedef struct
{
    u8 send_senser;
    u8 send_status;
    u8 send_rcdata;
    u8 send_motopwm;
    u8 send_power;
    u8 send_version;
    u8 send_pid1;
    u8 send_pid2;
    u8 send_pid3;
    u8 Acc_CALIBRATE;
    u8 Gyro_CALIBRATE;

}dt_flag_t;

typedef struct
{
   float kp;
   float ki;
   float kd;
}pid_s;

void getPID(float*dp,float*dp_rate);

void Radiotx_task(void *pvParameters);
void Radiorx_task(void *pvParameters);
void ANO_DT_Data_Exchange(void);
void ANO_DT_Send_Data(u8 *dataToSend , u8 length);
void ANO_DT_Data_Receive_Prepare(u8 data);
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num);
void ANO_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver);
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z,s32 bar);
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6);
void ANO_DT_Send_Power(u16 votage, u16 current);
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8);
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);
void ANO_DT_Send_Cailibrate_Check(u8 msg_id, u8 msg_data);
#endif
