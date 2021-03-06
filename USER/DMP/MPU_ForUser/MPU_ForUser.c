#include "MPU_ForUser.h"

struct MPU_Typedef MPU_Data;

uint8 MPU_Init_ForUser(void)
{
	if(MPU9250_Init())return 1;
	if(mpu_dmp_init())return 1;
	return 0;
}

uint8 Refresh_MPUTeam(MPU_DataTeam_TypeDef MPU_DataTeam)//更新MPU数据
{
  switch(MPU_DataTeam)
	{
	  case ACC:
			MPU_Get_Accelerometer(&MPU_Data.ACC.x,&MPU_Data.ACC.y,&MPU_Data.ACC.z);break;
	  case GYRO:
		 MPU_Get_Gyroscope(&MPU_Data.GYRO.x,&MPU_Data.GYRO.y,&MPU_Data.GYRO.z);break;
	  case MAG:
		 MPU_Get_Magnetometer(&MPU_Data.MAG.x,&MPU_Data.MAG.y,&MPU_Data.MAG.z);break;
	  case TEMP:
		 MPU_Data.TEMP=MPU_Get_Temperature();break;
#if defined USE_DMP
		case DMP_MPL:
		 mpu_dmp_get_data(&MPU_Data.Pitch, &MPU_Data.Roll, &MPU_Data.Yaw);break;
#elif defined USE_MPL
		case DMP_MPL:
		 mpu_mpl_get_data(&MPU_Data.Pitch, &MPU_Data.Roll, &MPU_Data.Yaw);break;
#endif
	  default : break;
	}
	return 0;
}

uint8 Save_MPUTeam(MPU_DataTeam_TypeDef MPU_DataTeam)//保存MPU上次数据
{
  	switch(MPU_DataTeam)
	{
	case GYRO:
	       MPU_Data.GYRO_Last.x=MPU_Data.GYRO.x;
	       MPU_Data.GYRO_Last.y=MPU_Data.GYRO.y;
	       MPU_Data.GYRO_Last.z=MPU_Data.GYRO.z;break;
	case ACC:
	       MPU_Data.ACC_Last.x=MPU_Data.ACC.x;
	       MPU_Data.ACC_Last.y=MPU_Data.ACC.y;
	       MPU_Data.ACC_Last.z=MPU_Data.ACC.z;break;
	case MAG:
	       MPU_Data.MAG_Last.x=MPU_Data.MAG.x;
	       MPU_Data.MAG_Last.y=MPU_Data.MAG.y;
	       MPU_Data.MAG_Last.z=MPU_Data.MAG.z;break;
	case DMP_MPL:
	       MPU_Data.Pitch_Last=MPU_Data.Pitch;
	       MPU_Data.Roll_Last=MPU_Data.Roll;
	       MPU_Data.Yaw_Last=MPU_Data.Yaw;break;
	
	default : break;
	}
	return 0;
}

float Mpu_Normalization(float Mpu6050_Value,float Mpu6050_Zero)  //Mpu6050值归为0-360
{
  	float Return_Data;
	  Return_Data=Mpu6050_Value-Mpu6050_Zero;
	  if(Return_Data < -180)
		Return_Data=Mpu6050_Value-Mpu6050_Zero+360;
	  if(Return_Data > 180)
		Return_Data=Mpu6050_Value-Mpu6050_Zero-360;
		
    return Return_Data;
}
