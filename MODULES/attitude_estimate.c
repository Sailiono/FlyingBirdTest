
/**
* @file     attitude_estimate.c
* @brief    直接读取xsens传感器数据
* @version  1.1
* @author   ljt
* @date     2020.8.3
*/

#include "attitude_estimate.h"
#include "main.h"
#include "math.h"


u32 tick=0;	
static Attitude_S euler_est;
static IMURAWDATAFTypeDef imurawdataf;


 void imuUpdate(void)
 {
    u8 xsensdata[47],res;
    u32 rollu32,yawu32,pitchu32,accxu32,accyu32,acczu32,gyroxu32,gyroyu32,gyrozu32;
    float roll,yaw,pitch,accx,accy,accz,gyrox,gyroy,gyroz,accxtemp,accytemp,accztemp;
     
    res=IIC_Read_Len_Byte(0x6B,0x06,47,xsensdata);
    if (res==0)
    {
        pitchu32 = (u32)((xsensdata[5]<<24)|(xsensdata[6]<<16)|(xsensdata[7]<<8)|(xsensdata[8])); 
        rollu32 = (u32)((xsensdata[9]<<24)|(xsensdata[10]<<16)|(xsensdata[11]<<8)|(xsensdata[12])); 
        yawu32 = (u32)((xsensdata[13]<<24)|(xsensdata[14]<<16)|(xsensdata[15]<<8)|(xsensdata[16]));
        accxu32 =  (u32)((xsensdata[20]<<24)|(xsensdata[21]<<16)|(xsensdata[22]<<8)|(xsensdata[23]));  
        accyu32 =  (u32)((xsensdata[24]<<24)|(xsensdata[25]<<16)|(xsensdata[26]<<8)|(xsensdata[27])); 
        acczu32 =  (u32)((xsensdata[28]<<24)|(xsensdata[29]<<16)|(xsensdata[30]<<8)|(xsensdata[31]));  
        gyroxu32 = (u32)((xsensdata[35]<<24)|(xsensdata[26]<<16)|(xsensdata[37]<<8)|(xsensdata[38])); 
        gyroyu32 = (u32)((xsensdata[39]<<24)|(xsensdata[40]<<16)|(xsensdata[41]<<8)|(xsensdata[42])); 
        gyrozu32 = (u32)((xsensdata[43]<<24)|(xsensdata[44]<<16)|(xsensdata[45]<<8)|(xsensdata[46])); 
        
        
        //下面是由于传感器坐标系与机体坐标系不一致，做了如下变换
        pitch = *(float*)&rollu32;
        roll = *(float*)&pitchu32;
        yaw = *(float*)&yawu32;
        yaw = -yaw;
        accxtemp = *(float*)&accyu32;
        accytemp = *(float*)&accxu32;
        accztemp = *(float*)&acczu32;
        accztemp = -accztemp;
        gyrox = *(float*)&gyroyu32;
        gyroy = *(float*)&gyroxu32;
        gyroz = *(float*)&gyrozu32;
        gyroz = -gyroz;
        
        //六面校准 校准参数为matlab程序处理校准数据后得到的
        accx = 1.002229f * (accxtemp-0.046913f) + -0.006321f * (accytemp-0.003118f) + 0.010441f * (accztemp+0.086567f);
        accy = 0.014512f * (accxtemp-0.046913f) + 1.001084f * (accytemp-0.003118f) + 0.0151f * (accztemp+0.086567f);
        accz = 0.026192f * (accxtemp-0.046913f) + -0.004022f * (accytemp-0.003118f) + 1.00029f * (accztemp+0.086567f);
        
        
        gyrox = RAD2DEG * gyrox;
        gyroy = RAD2DEG * gyroy;
        gyroz = RAD2DEG * gyroz;
        
        if(!isnan(pitch)) //判断xsens输出的是否是有效值，当有效之后后面都是有效
        {
            euler_est.pitch = pitch;
            euler_est.roll = roll;
            imurawdataf.accx = accx;
            imurawdataf.accy = accy;
            imurawdataf.accz = accz;
            imurawdataf.gyrox = gyrox;
            imurawdataf.gyroy = gyroy;
            imurawdataf.gyroz = gyroz;
            
            if(yaw < 0 )
            {   
                euler_est.yaw = yaw + 360.0f;
            }else
            {
                euler_est.yaw = yaw;
            }
            
//           printf("%.2f\t%.2f\t%.2f\r\n",accx,accy,accz); 
           //printf("%.2f \t%.2f \t%.2f %.2f \t%.2f \t%.2f \t %.2f\t%.2f\t%.2f\r\n",euler_est.roll,euler_est.pitch,euler_est.yaw,accx,accy,accz,gyrox,gyroy,gyroz);
        }else
        {
            imurawdataf.accx = 0.0f;
            imurawdataf.accy = 0.0f;
            imurawdataf.accz = 0.0f;
            euler_est.pitch = 0.0f;
            euler_est.roll = 0.0f;
            euler_est.yaw = 0.0f;
            printf("isnan");
        
        }

       
    }
    

 }
/*向外提供解算后的姿态角*/
void getAttitudeData(Attitude_S* get)
{
    get->roll = euler_est.roll;
	get->pitch = euler_est.pitch;
	get->yaw = euler_est.yaw;
}

/*向外提供IMU数据*/
void getImuRawData(IMURAWDATAFTypeDef* get)
{
    get->accx = imurawdataf.accx;
	get->accy = imurawdataf.accy;
	get->accz = imurawdataf.accz;
    get->gyrox = imurawdataf.gyrox;
    get->gyroy = imurawdataf.gyroy;
    get->gyroz = imurawdataf.gyroz;
}



//mpu6050任务函数
 void Attitude_estimate_task(void *param)
{																			//定时器时钟，每隔1ms +1
	u32 lastWakeTime = xTaskGetTickCount();
    
	while(1)
	{
        
        vTaskDelayUntil(&lastWakeTime,10);
        imuUpdate();
        tick++;
	}
}





