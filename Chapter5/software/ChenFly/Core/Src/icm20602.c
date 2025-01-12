#include "icm20602.h"
#include "spi.h"
#include "usart.h"
#include "math.h"
#include "string.h"

int16_t icm_gyro_x,icm_gyro_y,icm_gyro_z;
int16_t icm_acc_x,icm_acc_y,icm_acc_z;

#define delta_T     0.001f
#define PI          3.1415926f

uint8_t sendflag = 0;
int16_t rollint16 = 0;
int16_t pitchint16 = 0;
int16_t yawint16 = 0;
uint8_t send_attitude[11] = {0xFE, 0xEF, 0x0B, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

typedef struct 
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
} icm_param_t;
typedef struct 
{
    float q0;
    float q1;
    float q2;
    float q3;
} quater_param_t;
typedef struct 
{
    float pitch;
    float roll;
    float yaw;
} euler_param_t;
typedef struct 
{
    float Xdata;
    float Ydata;
    float Zdata;
} gyro_param_t;

float I_ex, I_ey, I_ez;
quater_param_t Q_info = {1, 0, 0, 0};
euler_param_t eulerAngle;
icm_param_t icm_data;
float icm_kp= 0.17;
float icm_ki= 0.004;
gyro_param_t GyroOffset;
float roll, pitch, yaw;
float roll_rate, pitch_rate, yaw_rate;

void icm_spi_w_reg_byte(uint8_t cmd, uint8_t val)
{
    uint8_t dat[2];
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_RESET);
    dat[0] = cmd | ICM20602_SPI_W;
    dat[1] = val;
    HAL_SPI_Transmit(&hspi1,dat,2,500);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_SET);
}

void icm_spi_r_reg_byte(uint8_t cmd, uint8_t *val)
{
    uint8_t dat[2];
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_RESET);
    dat[0] = cmd | ICM20602_SPI_R;
    dat[1] = *val;
    HAL_SPI_TransmitReceive(&hspi1,dat,dat,2,500);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_SET);
    *val = dat[1];
}

void icm_spi_r_reg_bytes(uint8_t * val, uint8_t num)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1,val,val,num,500);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_SET);
}

void icm20602_self_check(void)
{
    uint8_t dat=0;
    while(0x12 != dat)
    {
        icm_spi_r_reg_byte(ICM20602_WHO_AM_I,&dat);
        HAL_Delay(10);
    }

}

void icm20602_init_spi(void)
{
    uint8_t val = 0x00;

	HAL_Delay(10);
    icm20602_self_check();

    icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1,0x80);
    HAL_Delay(2);
    do
    {
        icm_spi_r_reg_byte(ICM20602_PWR_MGMT_1,&val);
    } while(0x41 != val);

    icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1,     0x01);
    icm_spi_w_reg_byte(ICM20602_PWR_MGMT_2,     0x00);
    icm_spi_w_reg_byte(ICM20602_CONFIG,         0x01);
    icm_spi_w_reg_byte(ICM20602_SMPLRT_DIV,     0x07);
    icm_spi_w_reg_byte(ICM20602_GYRO_CONFIG,    0x18);
    icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG,   0x10);
    icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG_2, 0x03);
}

void get_icm20602_accdata_spi(void)
{
    struct
    {
        uint8_t reg;
        uint8_t dat[6];
    } buf;

    buf.reg = ICM20602_ACCEL_XOUT_H | ICM20602_SPI_R;

    icm_spi_r_reg_bytes(&buf.reg, 7);
    icm_acc_x = (int16_t)(((uint16_t)buf.dat[0]<<8 | buf.dat[1]));
    icm_acc_y = (int16_t)(((uint16_t)buf.dat[2]<<8 | buf.dat[3]));
    icm_acc_z = (int16_t)(((uint16_t)buf.dat[4]<<8 | buf.dat[5]));
}

void get_icm20602_gyro_spi(void)
{
    struct
    {
        uint8_t reg;
        uint8_t dat[6];
    } buf;

    buf.reg = ICM20602_GYRO_XOUT_H | ICM20602_SPI_R;

    icm_spi_r_reg_bytes(&buf.reg, 7);
    icm_gyro_x = (int16_t)(((uint16_t)buf.dat[0]<<8 | buf.dat[1]));
    icm_gyro_y = (int16_t)(((uint16_t)buf.dat[2]<<8 | buf.dat[3]));
    icm_gyro_z = (int16_t)(((uint16_t)buf.dat[4]<<8 | buf.dat[5]));
}

void gyroOffsetInit(void)     
{
    GyroOffset.Xdata = 0;
    GyroOffset.Ydata = 0;
    GyroOffset.Zdata = 0;
    for (uint16_t i = 0; i < 100; ++i) 
	{
        get_icm20602_gyro_spi();
        GyroOffset.Xdata += icm_gyro_x;
        GyroOffset.Ydata += icm_gyro_y;
        GyroOffset.Zdata += icm_gyro_z;
        HAL_Delay(5);
    }
    GyroOffset.Xdata /= 100;
    GyroOffset.Ydata /= 100;
    GyroOffset.Zdata /= 100;
}


void icmGetValues(void) 
{
    float alpha = 0.3;

    icm_data.acc_x = (((float) icm_acc_x) * alpha) / 4096 + icm_data.acc_x * (1 - alpha);
    icm_data.acc_y = (((float) icm_acc_y) * alpha) / 4096 + icm_data.acc_y * (1 - alpha);
    icm_data.acc_z = (((float) icm_acc_z) * alpha) / 4096 + icm_data.acc_z * (1 - alpha);

    icm_data.gyro_x = ((float) icm_gyro_x - GyroOffset.Xdata) * PI / 180 / 16.4f; 
    icm_data.gyro_y = ((float) icm_gyro_y - GyroOffset.Ydata) * PI / 180 / 16.4f;
    icm_data.gyro_z = ((float) icm_gyro_z - GyroOffset.Zdata) * PI / 180 / 16.4f;
	
	roll_rate = (((float) icm_gyro_x - GyroOffset.Xdata) / 16.4f)*0.2f+roll_rate*0.8f; 
    pitch_rate = (((float) icm_gyro_y - GyroOffset.Ydata) / 16.4f)*0.2f+pitch_rate*0.8f;
    yaw_rate = (((float) icm_gyro_z - GyroOffset.Zdata) / 16.4f)*0.2f+yaw_rate*0.8f;
}

float myRsqrt(float num)
{
	float halfx = 0.5f * num;
	float y = num;
	long i = *(long*)&y;
	i = 0x5f375a86 - (i >> 1); 
 
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	
	return y;     
}

void icmAHRSupdate(icm_param_t* icm)            
{
    float halfT = 0.5f * delta_T; 
    float vx, vy, vz;               
    float ex, ey, ez;               
    
    float q0 = Q_info.q0;  
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    
    float q0q0 = q0 * q0;  
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    if(icm->acc_x * icm->acc_y * icm->acc_z == 0) 
        return;

    float norm = myRsqrt(icm->acc_x * icm->acc_x + icm->acc_y * icm->acc_y + icm->acc_z * icm->acc_z); 
    icm->acc_x = icm->acc_x * norm;
    icm->acc_y = icm->acc_y * norm;
    icm->acc_z = icm->acc_z * norm;

    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    ex = icm->acc_y * vz - icm->acc_z * vy;
    ey = icm->acc_z * vx - icm->acc_x * vz;
    ez = icm->acc_x * vy - icm->acc_y * vx;

    I_ex += halfT * ex;  
    I_ey += halfT * ey;
    I_ez += halfT * ez;

    icm->gyro_x = icm->gyro_x + icm_kp* ex + icm_ki* I_ex;
    icm->gyro_y = icm->gyro_y + icm_kp* ey + icm_ki* I_ey;
    icm->gyro_z = icm->gyro_z + icm_kp* ez + icm_ki* I_ez;

    q0 = q0 + (-q1 * icm->gyro_x - q2 * icm->gyro_y - q3 * icm->gyro_z) * halfT;
    q1 = q1 + (q0 * icm->gyro_x + q2 * icm->gyro_z - q3 * icm->gyro_y) * halfT;
    q2 = q2 + (q0 * icm->gyro_y - q1 * icm->gyro_z + q3 * icm->gyro_x) * halfT;
    q3 = q3 + (q0 * icm->gyro_z + q1 * icm->gyro_y - q2 * icm->gyro_x) * halfT;

    norm = myRsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    Q_info.q0 = q0 * norm;
    Q_info.q1 = q1 * norm;
    Q_info.q2 = q2 * norm;
    Q_info.q3 = q3 * norm;
}

void get_eulerAngle(void)
{
	float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    eulerAngle.pitch = asin(2 * q0 * q2 - 2 * q1 * q3) * 180 / PI; 
    eulerAngle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / PI; 
    eulerAngle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / PI;
	pitch = eulerAngle.pitch;
	roll = eulerAngle.roll;
	yaw = eulerAngle.yaw;
}

void icm20602_update(void)
{
	get_icm20602_accdata_spi();
	get_icm20602_gyro_spi();
	icmGetValues();
	icmAHRSupdate(&icm_data);
	get_eulerAngle();
	sendflag++;
	if(sendflag > 100)
	{
		sendflag = 0;
		
		rollint16 = eulerAngle.roll*10;
		pitchint16 = eulerAngle.pitch*10;
		yawint16 = eulerAngle.yaw*10;

		memcpy(send_attitude+4,&rollint16,sizeof(int16_t));
		memcpy(send_attitude+6,&pitchint16,sizeof(int16_t));
		memcpy(send_attitude+8,&yawint16,sizeof(int16_t));
		send_attitude[10] = 0;
		for(int i = 0; i < 10; i++)
		{
			send_attitude[10] = send_attitude[10]+send_attitude[i];
		}
//		HAL_UART_Transmit_DMA(&huart1, send_attitude, 11);
	}
}
