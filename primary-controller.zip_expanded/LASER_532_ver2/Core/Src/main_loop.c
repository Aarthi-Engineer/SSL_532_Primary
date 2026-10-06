/*
   Lab Machine
   Main loop code for Multispot laser system
   Written for STM32F100RB2 controller
   tested with stm32f100RB2 Discovery board
   code has functions to control
    1) foot switch
    2) Accessory
    3) Remote Interlock
    4) Communicate with Optotune mirror driver through SPI
    5) Control 532nm LASER power through internal DAC
    6) Generate TTL signal for 532nm LASER
    7) Control 635nm LASER with external SPI DAC

     program written on STCUBE IDE Version: 1.6.1
     used on board USB
*/

#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "spi.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
#include "stdlib.h"
#include "main_loop.h"
#include "Calibration.h"

bool  foot_key,r_foot_key=0;
bool Pattern;
bool FOOTSWITCH, REMOTE_INTERLOCK, LASER_532_OUT,  Red_break, Foot_Switch;
bool Treat_mode, Stand_by_mode;
bool RI ,FS,ACC_K;
int pat;
// LASER repeat interval ON / OFF Status
bool Repeat_mode_on = 0;
//software internal loop status 1 - yes; 0 - No
bool Main_loop_ON = 0;
bool ON = 1;
bool OFF = 0;

int LASER532_DAC_VALUE = 0;
int LASER635_DAC_VALUE = 0;

char serial_data_message[25] = "null";
char serial_data_command[25] = "null";

#define char_to_integer
#define mirror_k 33.80675

char nh0;
char nh1;
char nh2;

DMA_HandleTypeDef hdma_usart1_rx;
int count1 = 0;
int laser_ready;
int laser_temp;
// variable to hold LASER Duration interval by default it is initialized to zero
int Duration = 10;
int  D;
int mm_nn;
// variable to hold LASER repeat interval by default it is initialized to zero
int Repeat_interval = 0;
int R;
// variable to hold LASER power by default it is initialized to zero
int Power = 0, Power_DAC = 0, Power_watts = 25,MJ=0;
float Power_volt = 0;
// variable to hold LASER power by default it is initialized to zero
int Buzzer = 0;
// variable to hold LASER power by default it is initialized to zero
int Adapter = 0;
float spot_size = 0.2;//Fixed for 200um
int spot_size_int;
float  Circle_points;
float Spot_space = 1.0;//1.0;
float temp_Spot_space = 1.0;
int start_angle, end_angle;

// LASER duration interval timer set to default value 0
int Duration_set_Value = 10;

// LASER repeat interval timer set to default value 0
int Repeat_interval_Set_value =0;

// LASER power DAC value set to default 0
int Power_Set_value = 0;
// BUZZER power PWM value set to default 0
int Buzzer_Set_value = 0;
// AIM power DAC value set to default 0
int Aim_Set_value = 0;

int A5[31]={0};
int p;
int Adapter_Set_value = 0;
uint16_t AD_RES = 0;


int get_pattern_type(char ch);
int get_pattern_type_red(char ch);
int get_duration_timer_value(int);
int get_repeat_interval_timer_value(int);
void line_VH(float, float, int, int,int);
void line_VH_2(int, int);
void drawCircle(float, float, float, float, int);
void drawARC(float, float, float, float, int);
void line_VH_red(float, float, int, int, int, int);
void line_VH_red_3(float, float, int, int, int, int);
void line_VH_red_4(float, float, int, int, int, int);
void line_VH_red_5(float, float, int, int, int, int);
void line_VH_red_3x5(float, float, int, int, int, int);
void drawCircle_red(float, float, float, float, int, int);
void drawARC_red(float, float, float, float, int, int);
void line_horizontal_red(float, float, int, int, int, int);
void line_horizontal(float, float, int, int);
void line_vertical_red(float, float, int, int, int, int);
void line_vertical(float, float, int, int);
void MoveTocentre();
void Spot_size_read();
void single_spot();
void single_spot_red();
void stone();//Added for DWIN
void laser_check();


uint8_t DATA_mode[14] = {0x00, 0x01, 0x40, 0x02, 0x40, 0x07, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0xc1};// mirror closed loop command

uint8_t DATA_zero7[14] = {0x00, 0x01, 0x50, 0x02, 0x51, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};// mirror set zero position command

uint8_t DATA_r[14] = {0};

uint8_t write_command_MSB = 0x00;
uint8_t write_command_LSB = 0x01;

uint8_t x_reg_MSB = 0x50; // x_regiter add
uint8_t x_reg_LSB = 0x02;
uint8_t y_reg_MSB = 0x51;// y_regiter add
uint8_t y_reg_LSB = 0x02;

int d1,d;

/* USER CODE END PFP */
int j = 0, i = 0;
float x, y, xx, yy, size, x_offset = 0.13,   radius,y_offset = 0.15,y_0=0.25,x_0=0.03;//-0.050;
float space; // Space between Spots
float space1; //Space between Circle
float radius0 = 1.0;
float dx = 0.0, dy = 0.0;
int mm = 0, nn = 0, set_delay =0;

int start, end, start_angle, end_angle;
int y_location_a[8][8], x_location_a[8][8];
int y_location_a2[8][8], x_location_a2[8][8];
//int y_location_a3[8][8], x_location_a3[8][8];
int y_location_a4[8][8], x_location_a4[8][8];
int y_location_a5[8][8], x_location_a5[8][8];
int y_location, x_location;
char mh1 = 'G';

int x_grid[31],y_grid[31],x_grid_2[9],y_grid_2[9],x_grid_3[15],y_grid_3[15],x_grid_4[20],y_grid_4[20],x_grid_5[30],y_grid_5[30];
int x_grid_g4[20],y_grid_g4[20];
float theta[6] = {0.0,0.0,4.2,4.8,0.0,5.0};
float theta_g[6] = {0.0,0.0,4.2,4.8,0.0,5.0};
float theta_1[6]={0.0,0.0,8.0,1,0,2.5};//For 100 um
int on_delay[6] = {0,0,2000,300,375,250};
int off_delay[6] = {0,0,0,200,300,0};//250 t0 175
int H_delay[6] = {0,0,5,3,1,1};//For Red

uint8_t data_1[9] = {0};
uint8_t data_2[9] = {0};
uint8_t data_3 [9]= {0};
uint8_t data_4 [9]= {0};
uint8_t data_5 [9]= {0};
uint8_t data_6 [9]= {0};
uint8_t data_7 [9]= {0};
uint8_t data_8 [9]= {0};

/***********************************************Added for DWIN *****************************************************/
uint8_t RxBuffer[9]={0};//Rxbuffer
uint8_t Txbuffer[8]={0x5A,0xA5,0x05,0x82,0x00,0x00,0x00,0x00};
uint8_t Default_Power[8]={0x5A,0xA5,0x05,0x82,0x10,0x00,0x00,0x19};
uint8_t Default_Exposure_M[8]={0x5A,0xA5,0x05,0x82,0x10,0x01,0x00,0x1E};
uint8_t Default_Exposure_S[8]={0x5A,0xA5,0x05,0x82,0x10,0x01,0x00,0x64};
uint8_t Default_Repeat_Int[8]={0x5A,0xA5,0x05,0x82,0x10,0x09,0x00,0x00};
uint8_t Treat_stand[8]={0x5A,0xA5,0x05,0x82,0x25,0x00,0x00,0x00};
uint8_t Disable_Screen[10]={0x5A,0xA5,0x07,0x82,0x00,0xFC,0x55,0xAA,0x5A,0x5A};
uint8_t Enable_Screen[10]={0x5A,0xA5,0x07,0x82,0x00,0xFC,0x00,0x00,0x00,0x00};
//uint8_t Exs[8]={0x5A,0xA5,0x05,0x82,0x94,0x00,0x00,0x01};
//////////////////////////////////////////////////////////////////////////////////////
/*uint8_t Clear_Ready_Buffer[11]={0x5A,0xA5,0x08,0x82,0x85,0x04,0x00,0x00,0x00,0x00,0x00};
uint8_t Clear_Not_Ready_Buffer[14]={0x5A,0xA5,0x0B,0x82,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t Ready_Buffer[11]={0x5A,0xA5,0x08,0x82,0x85,0x04,0x52,0x45,0x41,0x44,0x59};
uint8_t Not_Ready_Buffer[14]={0x5A,0xA5,0x0B,0x82,0x80,0x00,0x4e,0x4f,0x54,0x52,0x45,0x41,0x44,0x59};*/
uint8_t Ready_Buffer[8]={0x5A,0xA5,0x05,0x82,0x85,0x04,0x00,0x02};
uint8_t Not_Ready_Buffer[8]={0x5A,0xA5,0x05,0x82,0x85,0x04,0x00,0x01};
////////////////////////////////////////////////////////////////////////////////////////
uint8_t Spot_spacing[8]={0x5A,0xA5,0x05,0x82,0x43,0x00,0x00,0x00};
uint8_t Spot_Distance[6],nh6,nh7,Spot_Size[3];
uint16_t address;
uint8_t Aim = 0,c;
uint8_t p1=0,p2=0,p3=0,p4=0,p5=0,p6 =0;//For Calib Password screen
uint16_t Radius=1000;
int Repeat_Interval =0;
char mh = 'G';

int Si_D;//For Spot Distance
float dist;//For Spot Distance
uint16_t Disp_Spot_Distance;

uint8_t Footswitch_Buffer[10]={0x5A,0xA5,0x07,0x82,0x00,0x84,0x5A,0x01,0x00,0x17};// As per the page.no.36 of T5L Development guide PIC_Set condition//Note:0x16 is the page.no in Hexa
uint8_t Initial_Footswitch_Buffer[10]={0x5A,0xA5,0x07,0x82,0x00,0x84,0x5A,0x01,0x00,0x0B};
uint8_t Square_Blocks [10]={0x5A,0xA5,0x07,0x82,0x00,0x84,0x5A,0x01,0x00,0x24};
uint8_t Circle_Blocks [10]={0x5A,0xA5,0x07,0x82,0x00,0x84,0x5A,0x01,0x00,0x23};
uint8_t Special_Blocks [10]={0x5A,0xA5,0x07,0x82,0x00,0x84,0x5A,0x01,0x00,0x25};
uint8_t RI_Buffer[10]={0x5A,0xA5,0x07,0x82,0x00,0x84,0x5A,0x01,0x00,0x18};
uint8_t Initial_Page_RI[10] = {0x5A,0xA5,0x07,0x82,0x00,0x84,0x5A,0x01,0x00,0x05};
uint8_t Spot_Size_Display[8]={0x5A,0xA5,0x05,0x82,0x42,0x00,0x00,0x00};
uint8_t Spot_Count_Display[8]={0x5A,0xA5,0x05,0x82,0x41,0x00,0x00,0x00};
uint16_t Shot_Count = 0;
uint8_t Transmit = 0;
uint8_t Initial_loop =1;
uint8_t SLA[8]={0x5A,0xA5,0x05,0x82,0x77,0x00,0x00,0x01};
uint8_t LIO[8]={0x5A,0xA5,0x05,0x82,0x77,0x00,0x00,0x02};
uint8_t ENDO[8]={0x5A,0xA5,0x05,0x82,0x77,0x00,0x00,0x03};

uint8_t Acc_Key[10]={0x5A,0xA5,0x07,0x82,0x00,0x84,0x5A,0x01,0x00,0x1A};
uint8_t Initial_Page_Acc[10]={0x5A,0xA5,0x07,0x82,0x00,0x84,0x5A,0x01,0x00,0x0C};
bool Acc_key_Variable = 0;
/***********************************************************For Calibration**************************/
uint8_t Calib_V1[8] = {0x5A,0xA5,0x05,0x82,0x47,0x50,0x00,0x64};
uint8_t Calib_V2[8] = {0x5A,0xA5,0x05,0x82,0x95,0x50,0x00,0xC8};
uint8_t Calib_V3[8] = {0x5A,0xA5,0x05,0x82,0x95,0x60,0x01,0x2C};
uint8_t Calib_V4[8] = {0x5A,0xA5,0x05,0x82,0x95,0x70,0x01,0x90};
uint8_t Calib_V5[8] = {0x5A,0xA5,0x05,0x82,0x95,0x80,0x01,0xF4};
uint8_t Calib_V6[8] = {0x5A,0xA5,0x05,0x82,0x85,0x90,0x02,0x58};
uint8_t Calib_V7[8] = {0x5A,0xA5,0x05,0x82,0x86,0x00,0x02,0xBC};
uint8_t Calib_V8[8] = {0x5A,0xA5,0x05,0x82,0x86,0x10,0x03,0xE8};
uint8_t Calib_V9[8] = {0x5A,0xA5,0x05,0x82,0x86,0x20,0x05,0xDC};
uint8_t Calib_V10[8] = {0x5A,0xA5,0x05,0x82,0x86,0x30,0x07,0xD0};

uint8_t Flash_write[14]={0x5A,0xA5,0x0B,0x82,0x00,0x08,0x00,0x00,0x00,0x02,0x20,0x02,0x00,0x02};
uint8_t Flash_write_LIO[14]={0x5A,0xA5,0x0B,0x82,0x00,0x08,0x00,0x00,0x00,0x0A,0x20,0x02,0x00,0x02};
uint8_t Flash_write_ENDO[14]={0x5A,0xA5,0x0B,0x82,0x00,0x08,0x00,0x00,0x00,0x12,0x20,0x02,0x00,0x02};
uint8_t Flash_read[14]={0x5A,0xA5,0x0B,0x82,0x00,0x08,0x00,0x00,0x00,0x02,0x20,0x02,0x00,0x02};
uint8_t Flash_read_LIO[14]={0x5A,0xA5,0x0B,0x82,0x00,0x08,0x00,0x00,0x00,0x0A,0x20,0x02,0x00,0x02};
uint8_t Flash_read_ENDO[14]={0x5A,0xA5,0x0B,0x82,0x00,0x08,0x00,0x00,0x00,0x12,0x20,0x02,0x00,0x02};
uint8_t Flash_read_variable[7]={0x5A,0xA5,0x04,0x83,0x20,0x02,0x01};
bool Interrupt_Calib;
uint8_t Page_switch[10]={0x5A,0xA5,0x07,0x82,0x00,0x84,0x5A,0x01,0x00,0x0C};
uint8_t Page_switch_SS[10]={0x5A,0xA5,0x07,0x82,0x00,0x84,0x5A,0x01,0x00,0x13};
uint8_t Page_Calibration[10]={0x5A,0xA5,0x07,0x82,0x00,0x84,0x5A,0x01,0x00,0x08};//17.08.2024
uint8_t Exs_Page [10]={0x5A,0xA5,0x07,0x82,0x00,0x84,0x5A,0x01,0x00,0x09};
int Variable_1=100.0,Variable_2=200.0,Variable_3=300.0,Variable_4=400.0,Variable_5=500.0,Variable_6=600.0,Variable_7=700.0,Variable_8=1000.0,Variable_9=1500.0,Variable_10=2000.0;
int Set_V1=100.0,Set_V2=200.0,Set_V3=300.0,Set_V4=400.0,Set_V5=500.0,Set_V6=600.0,Set_V7=700.0,Set_V8=1000.0,Set_V9=1500.0,Set_V10=2000.0;
float calibration_value = 1.0;
float calibration_value_upto_7 = 1.0;
float new_calibration_value = 1.0;
uint8_t Calib_Value[8] = {0x5A,0xA5,0x07,0x82,0x20,0x02,0x00,0x00};
int Calib_Flag;
int no_of_var = 10;
uint32_t tot_x,tot_y,tot_x_sq;
float xy,xy_7;
int Calibration;
bool Interrupt_Calib;
int Initialisation = 1;
uint16_t Zero_Shot_Count = 0;
uint16_t Total_Shot_Count = 0;
uint16_t Temp_Total_Shot_Count = 0;
uint16_t Total_Shot_Count_1 = 0;
uint16_t Temp_Total_Shot_Count_1 = 0;
uint16_t Update_Shot_Count = 0;
uint16_t Update_Shot_Count_1 = 0;
uint8_t eXs =0;

uint8_t Total_SC_Value[8] = {0x5A,0xA5,0x07,0x82,0x20,0x10,0x00,0x00};
uint8_t Total_SC_Value_1[8] = {0x5A,0xA5,0x07,0x82,0x20,0x20,0x00,0x00};
uint8_t Flash_Shot_Count[14]={0x5A,0xA5,0x0B,0x82,0x00,0x08,0x00,0x00,0x00,0x42,0x20,0x10,0x00,0x02};
uint8_t Flash_R_Shot_Count[14]={0x5A,0xA5,0x0B,0x82,0x00,0x08,0x00,0x00,0x00,0x42,0x20,0x10,0x00,0x02};
uint8_t Flash_read_variable_1[7]={0x5A,0xA5,0x04,0x83,0x20,0x10,0x01};
uint8_t Flash_read_variable_2[7]={0x5A,0xA5,0x04,0x83,0x20,0x20,0x01};
uint8_t Flash_Shot_Count_1[14]={0x5A,0xA5,0x0B,0x82,0x00,0x08,0x00,0x00,0x00,0x52,0x20,0x20,0x00,0x02};
uint8_t Flash_R_Shot_Count_1[14]={0x5A,0xA5,0x0B,0x82,0x00,0x08,0x00,0x00,0x00,0x52,0x20,0x20,0x00,0x02};

/*************************************************************************************************************/
uint8_t data_1_1 = 0x00;
uint8_t data_2_2 = 0x00;
uint8_t data_3_3 = 0x00;
uint8_t data_4_4 = 0x00;
uint8_t data_5_5 = 0x00;
uint8_t data_6_6 = 0x00;
uint8_t data_7_7 = 0x00;
uint8_t data_8_8 = 0x00;
/************************************************************************************************************/
/*Default*/

/*
uint8_t Kp[14] = {0x00, 0x01, 0xC0, 0x01, 0xC1, 0x01, 0x41, 0x70, 0x00, 0x00, 0x41, 0x70, 0x00, 0x00};// mirror P gain
uint8_t Ki[14]={0x00, 0x01, 0xC0, 0x02, 0xC1, 0x02, 0x43, 0x48, 0x80, 0x00, 0x43, 0x48, 0x80, 0x00};// mirror I gain
uint8_t Kd[14]={0x00, 0x01, 0xC0, 0x03, 0xC1, 0x03, 0x3d, 0x4c, 0xcc, 0xcd, 0x3d, 0x4c, 0xcc, 0xcd};// mirror D gain
uint8_t Enable_Filter[14]={0x00, 0x01, 0xC0, 0x13, 0xC1, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};// Filter
uint8_t Cutoff_frequency[14]={0x00, 0x01, 0xC0, 0x14, 0xC1, 0x14, 0x44, 0x48, 0x00, 0x00, 0x44, 0x48, 0x00, 0x00};// Cutoff frequency of derivative term filter
*/

/*Degree 1-5 */
uint8_t Kp[14] = {0x00, 0x01, 0xC0, 0x01, 0xC1, 0x01, 0x42, 0xb4, 0x00, 0x00, 0x42, 0xb4, 0x00, 0x00};// mirror P gain
uint8_t Ki[14]={0x00, 0x01, 0xC0, 0x02, 0xC1, 0x02, 0x44, 0x3b, 0x80, 0x00, 0x44, 0x3b, 0x80, 0x00};// mirror I gain
uint8_t Kd[14]={0x00, 0x01, 0xC0, 0x03, 0xC1, 0x03, 0x3d, 0xae, 0x14, 0x7b, 0x3d, 0xae, 0x14, 0x7b};// mirror D gain
uint8_t Enable_Filter[14]={0x00, 0x01, 0xC0, 0x13, 0xC1, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};// Filter
uint8_t Cutoff_frequency[14]={0x00, 0x01, 0xC0, 0x14, 0xC1, 0x14, 0x44, 0x7a, 0x00, 0x00, 0x44, 0x7a, 0x00, 0x00};// Cutoff frequency of derivative term filter

/*Degree 0.1-1 Degree*/
uint8_t Kp_L[14] = {0x00, 0x01, 0xC0, 0x01, 0xC1, 0x01, 0x43, 0x0c, 0x00, 0x00, 0x43, 0x0c, 0x00, 0x00};// mirror P gain
uint8_t Ki_L[14]={0x00, 0x01, 0xC0, 0x02, 0xC1, 0x02, 0x44, 0x3b, 0x80, 0x00, 0x44, 0x3b, 0x80, 0x00};// mirror I gain
uint8_t Kd_L[14]={0x00, 0x01, 0xC0, 0x03, 0xC1, 0x03, 0x3d, 0xb8, 0x51, 0xec, 0x3d, 0xb8, 0x51, 0xec};// mirror D gain
uint8_t Enable_Filter_L[14]={0x00, 0x01, 0xC0, 0x13, 0xC1, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};// Filter
uint8_t Cutoff_frequency_L[14]={0x00, 0x01, 0xC0, 0x14, 0xC1, 0x14, 0x45, 0x22, 0x80, 0x00, 0x45, 0x22, 0x80, 0x00};// Cutoff frequency of derivative term filter

bool PID =0;//For setting PID
/////Added for m and c
uint8_t Flash_write_m_SLA[14]={0x5A,0xA5,0x0B,0x82,0x00,0x08,0x00,0x00,0x00,0x22,0x20,0x32,0x00,0x02};
uint8_t Flash_write_c_SLA[14]={0x5A,0xA5,0x0B,0x82,0x00,0x08,0x00,0x00,0x00,0x32,0x20,0x42,0x00,0x02};

uint8_t Flash_write_m_LIO[14]={0x5A,0xA5,0x0B,0x82,0x00,0x08,0x00,0x00,0x00,0x27,0x20,0x32,0x00,0x02};
uint8_t Flash_write_c_LIO[14]={0x5A,0xA5,0x0B,0x82,0x00,0x08,0x00,0x00,0x00,0x37,0x20,0x42,0x00,0x02};

uint8_t Flash_write_m_ENDO[14]={0x5A,0xA5,0x0B,0x82,0x00,0x08,0x00,0x00,0x00,0x0F,0x20,0x32,0x00,0x02};
uint8_t Flash_write_c_ENDO[14]={0x5A,0xA5,0x0B,0x82,0x00,0x08,0x00,0x00,0x00,0x17,0x20,0x42,0x00,0x02};

uint8_t Flash_read_m[7]={0x5A,0xA5,0x04,0x83,0x20,0x32,0x01};
uint8_t Flash_read_c[7]={0x5A,0xA5,0x04,0x83,0x20,0x42,0x01};
uint8_t Flash_read_Aim[7]={0x5A,0xA5,0x04,0x83,0x20,0x52,0x01};

uint8_t Flash_read_m_SLA[14]={0x5A,0xA5,0x0B,0x82,0x00,0x08,0x00,0x00,0x00,0x22,0x20,0x32,0x00,0x02};
uint8_t Flash_read_c_SLA[14]={0x5A,0xA5,0x0B,0x82,0x00,0x08,0x00,0x00,0x00,0x32,0x20,0x42,0x00,0x02};

uint8_t Flash_read_m_LIO[14]={0x5A,0xA5,0x0B,0x82,0x00,0x08,0x00,0x00,0x00,0x27,0x20,0x32,0x00,0x02};
uint8_t Flash_read_c_LIO[14]={0x5A,0xA5,0x0B,0x82,0x00,0x08,0x00,0x00,0x00,0x37,0x20,0x42,0x00,0x02};

uint8_t Flash_read_m_ENDO[14]={0x5A,0xA5,0x0B,0x82,0x00,0x08,0x00,0x00,0x00,0x0F,0x20,0x32,0x00,0x02};
uint8_t Flash_read_c_ENDO[14]={0x5A,0xA5,0x0B,0x82,0x00,0x08,0x00,0x00,0x00,0x17,0x20,0x42,0x00,0x02};

uint8_t Flash_write_Aim_S[14]={0x5A,0xA5,0x0B,0x82,0x00,0x08,0x00,0x00,0x00,0x47,0x20,0x52,0x00,0x02};
uint8_t Flash_read_Aim_S[14]={0x5A,0xA5,0x0B,0x82,0x00,0x08,0x00,0x00,0x00,0x47,0x20,0x52,0x00,0x02};
float m_value = 1.0;
float c_value = 1.0;
int Aim_S = 8;
int get_pattern_type(char ch)
{

  switch (ch)
  {

  case 'G':
     	//MoveTocentre();
       single_spot();
       break;


    default:
      break;

  }
  return 0;
}

int get_pattern_type_red(char ch)
{

 // Spot_size_read();

  switch (ch)
  {
    case 'G':
     	//MoveTocentre();
       	single_spot_red();
         break;

    default:
        break;
  }
  return 0;
}

/*  function to return back Duration interval timer register value with respect to timer value 5ms to 100ms */
int get_duration_timer_value(int i)
{
  D = Duration;
  return D;
}
/* function to return back repeat interval timer register value with respect to timer value 5ms to 100ms */
int get_repeat_interval_timer_value(int i)
{

	if (mh == 'G')
  {
   	  R=Repeat_Interval;
  }
  else
  {
    R = 0;
  }
  return R;
}

/* function to return back DAC value with respect to power value 25mW to 2000mW */
void check_loop()
{


  MoveTocentre();

  DATA_r[0] = write_command_MSB;
  DATA_r[1] = write_command_LSB;
  DATA_r[2] = x_reg_MSB;
  DATA_r[3] = x_reg_LSB;
  DATA_r[4] = y_reg_MSB;
  DATA_r[5] = y_reg_LSB;

  HAL_UART_Receive_IT(&huart1, RxBuffer,sizeof(RxBuffer));//Added to receive command from Dwin Display

  if(Initial_loop == 1)
        {

	        Duration = (int)((Default_Exposure_S[6]<<8) + Default_Exposure_S[7]);
	        HAL_UART_Transmit(&huart3,&Transmit,1,100);
	        HAL_UART_Transmit(&huart1, Default_Exposure_S, 8, 100);
	        HAL_UART_Transmit(&huart1, Default_Repeat_Int, 8, 100);
	        Initial_loop = 2;
        }
     /*   Remote interlock check   */
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6))
    {
      REMOTE_INTERLOCK = 1;
    }
    else
    {
      REMOTE_INTERLOCK = 0;
      Treat_mode = OFF;
      Stand_by_mode = ON;
     // HAL_UART_Transmit(&huart1, Clear_Ready_Buffer, 11, 100);
      HAL_UART_Transmit(&huart1, Not_Ready_Buffer, 8, 100);//Not Ready Buffer ON //Added for DWIN
      HAL_UART_Transmit(&huart1, RI_Buffer, 10, 100);

    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /*  Foot switch check   */
    if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
    {
       FOOTSWITCH = 1;
    }
    else
    {
       FOOTSWITCH = 0;
       Stand_by_mode = ON;
       Treat_mode = OFF;
       //HAL_UART_Transmit(&huart1, Clear_Ready_Buffer, 11, 100);
       HAL_UART_Transmit(&huart1, Not_Ready_Buffer, 8, 100);//Not Ready Buffer ON //Added for DWIN
       HAL_UART_Transmit(&huart1, Footswitch_Buffer, 10, 100);//To Display Foot Switch Disconnected Pop_Menu on the Screen
    }
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (REMOTE_INTERLOCK && FOOTSWITCH)
    {
        Treat_mode = ON;//To Step into Treatment mode (For Dwin Display)
        Stand_by_mode = OFF;
        /*Delivery Device Check*/
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))
        {
        	HAL_UART_Transmit(&huart1, SLA, 8, 100);
      	    Adapter=1;
        	Acc_key_Variable = 0;
        }
        else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
      	{

      	    HAL_UART_Transmit(&huart1, LIO, 8, 100);
      	    Adapter=2;  mh='G';
      	    Acc_key_Variable = 0;

      	}
        else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
      	{

        	HAL_UART_Transmit(&huart1, ENDO, 8, 100);
        	Adapter=3;mh='G';
        	Acc_key_Variable = 0;


      	}

        else
        {
            Acc_key_Variable =1 ;
        	Stand_by_mode = ON;
        	Treat_mode = OFF;
        	//HAL_UART_Transmit(&huart1, Clear_Ready_Buffer, 11, 100);
        	HAL_UART_Transmit(&huart1, Not_Ready_Buffer, 8, 100);//Not Ready Buffer ON //Added for DWIN

        	HAL_UART_Transmit(&huart1, Acc_Key, 10, 100);
        }
    }
    else
    {
        Treat_mode = OFF;Stand_by_mode = ON;//To Step into standby mode (For Dwin Display)
        if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
        {

        }
        else
        {
            HAL_UART_Transmit(&huart1, Footswitch_Buffer, 10, 100);
        }
        if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6))
        {

        }
        else
        {
            HAL_UART_Transmit(&huart1, RI_Buffer, 10, 100);//Remote Interlock Not Connected - DWIN
        }

     }
    //Spot_size_read();


  while (Main_loop_ON)
  {

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	  /*   Remote interlock check   */
	 if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6))
	 {
	     REMOTE_INTERLOCK = 1;
	 }
	 else
	 {
	     REMOTE_INTERLOCK = 0;
	     Treat_mode = OFF;
	     Stand_by_mode = ON;
	     HAL_UART_Transmit(&huart1, RI_Buffer, 10, 100);//21.06.2022
	     HAL_UART_Transmit(&huart1, Treat_stand, 8, 100);
	     Main_loop_ON=0;
	     mh='G';
	  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////
	   /*   Foot switch check  */
	  if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
	  {
	      FOOTSWITCH = 1;
	  }
	  else
	  {
	      FOOTSWITCH = 0;
	      Stand_by_mode = ON;
	      Treat_mode = OFF;
	      HAL_UART_Transmit(&huart1, Footswitch_Buffer, 10, 100);//To Display Foot Switch Disconnected Pop_Menu on the Screen
	      HAL_UART_Transmit(&huart1, Treat_stand, 8, 100);
	      Main_loop_ON=0;
	      mh='G';
	  }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	  if (REMOTE_INTERLOCK && FOOTSWITCH)
	  {
	      Treat_mode = ON;//To Step into Treatment mode (For Dwin Display)
	      Stand_by_mode = OFF;

	      /*Delivery Device Check*/

	      if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))
	      {
	    	  HAL_UART_Transmit(&huart1, SLA, 8, 100);
	    	  Adapter=1;
	    	  Acc_key_Variable = 0;
	    	   Treat_mode = ON;//To Step into Treatment mode (For Dwin Display)
	    		      Stand_by_mode = OFF;
	      }

	      else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
	      {

	    	  HAL_UART_Transmit(&huart1, LIO, 8, 100);
	    	  Adapter=2;
	    	  mh='G';
	    	  Acc_key_Variable = 0;
	    	  Treat_mode = ON;//To Step into Treatment mode (For Dwin Display)
	          Stand_by_mode = OFF;
	    	 // HAL_UART_Transmit(&huart1, Page_switch_SS, 10, 100);
	      }

	      else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
	      {
	    	  HAL_UART_Transmit(&huart1, ENDO, 8, 100);
	    	  Adapter=3;
	    	  mh='G';
	    	  Acc_key_Variable = 0;
	    	  Treat_mode = ON;//To Step into Treatment mode (For Dwin Display)
	    	  Stand_by_mode = OFF;
	    	//  HAL_UART_Transmit(&huart1, Page_switch_SS, 10, 100);
	      }
	      else
	      {
	    	  Acc_key_Variable =1 ;
	    	  Stand_by_mode = ON;
	    	  Treat_mode = OFF;
	    	  Main_loop_ON=0;
	    	  HAL_UART_Transmit(&huart1, Acc_Key, 10, 100);

	      }

	  }
	  else
	  {
	      Treat_mode = OFF;//To Step into standby mode (For Dwin Display)
	      if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
	      {

	      }
	      else
	      {
	          HAL_UART_Transmit(&huart1, Footswitch_Buffer, 10, 100);
	          HAL_UART_Transmit(&huart1, Treat_stand, 8, 100);
	          Main_loop_ON=0;
	      }
	      if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6))
	      {

	      }
	      else
	      {
	           HAL_UART_Transmit(&huart1, RI_Buffer, 10, 100);//Remote Interlock Not Connected - DWIN
	           HAL_UART_Transmit(&huart1, Treat_stand, 8, 100);
	           Main_loop_ON=0;
	       }

	   }

////////////////////////////////////////////////////////////////////////////////////////////////////////////

	 HAL_UART_Receive_IT(&huart1, RxBuffer,sizeof(RxBuffer));

   //  Spot_size_read();

     //MoveTocentre();
     laser_check();
     //line_VH_2(mm,nn);

   /*  if(Initial_loop == 1)
     {
        //HAL_UART_Transmit(&huart1, Initial_Page, 10, 100);
    	//MoveTocentre();
        line_VH_2(mm,nn);
        //HAL_UART_Transmit(&huart1, Main_Page, 10, 100);
        Initial_loop =3;
     }*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////
     if (Treat_mode == ON && Stand_by_mode == OFF)
     {
    	 if(Aim==0)
    	 {
    		 Aim_Set_value = 0;
    	 }
    	 else
    	 {
    		 if(mh=='G')
    		 {

    			 Aim_Set_value = ((Aim) * Aim_S);//8//5//
    		 }
    		 else
    		   {

    		    Aim_Set_value = ((Aim) * 40);//Here the maximum DAC value is 3500 to make the aim value in percentage the 3500 has been divided by 100 also added 700 for adjustment//get_aim_value(Aim);
    		    if(Aim==100)
    		    {
    		    	Aim_Set_value = 4090;
    		    }
    		   }
    		 }
    	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); // Laser status
         HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, Aim_Set_value);

         if (Treat_mode == ON && laser_ready == 0)
          {

            if((laser_temp==1) && (laser_ready==0))

             {
        	   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
              // HAL_UART_Transmit(&huart1, Clear_Not_Ready_Buffer, 14, 100);
               HAL_UART_Transmit(&huart1, Ready_Buffer, 8, 100);//Ready Buffer ON //Added for DWIN Display
             }

             //  Red_break = 0;//Commenting it for footswitch problem as we have changed the IO Board
            //   GPIOC->ODR&=~GPIO_PIN_8;
               HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);//For Board2

                get_pattern_type_red(mh);

              // single_spot_red();
               HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);//For Board2
               Red_break = 0;

               GPIOC->ODR&=~GPIO_PIN_8;

           }
      }
      else
         {
           HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); // Trigger LASER 532 //LASER ON
           HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);   // LASER 635 DAC OUT to 0
           //HAL_UART_Transmit(&huart1, Clear_Ready_Buffer, 11, 100);
           HAL_UART_Transmit(&huart1, Not_Ready_Buffer, 8, 100);//Not Ready Buffer ON //Added for DWIN
         }
                      /*** LASER TEMP check ***/
//////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10))
    {
    	laser_temp = 1;//Ok =1//C10 is for laser Temperature signal from the laser Driver
    }
    else
    {
      laser_temp = 0;
    }
    //LASER READY check/
    if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11))
    {
      laser_ready = 0;//C11 is for laser Ready signal from the laser Driver - Not Connected
     // HAL_UART_Transmit(&huart1, Clear_Not_Ready_Buffer, 14, 100);
      HAL_UART_Transmit(&huart1, Ready_Buffer, 8, 100);//Ready Buffer ON //Added for DWIN Display
    }
    else
    {
      laser_ready = 1;
    }
    Duration_set_Value = Duration;
    //Adapter=1;
    Power_Set_value = get_Adapter_value(Adapter);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Power_Set_value);
    Buzzer_Set_value = Buzzer;//get_buzzer_value(Buzzer);


    if (Repeat_mode_on  == ON )
    {
      Repeat_interval_Set_value = Repeat_Interval;
      MX_TIM6_Init(Duration_set_Value);
      MX_TIM7_Init(Repeat_interval_Set_value);

    }
    else
    {
      Repeat_interval_Set_value = 0;
      MX_TIM6_Init(Duration_set_Value);
      MX_TIM7_Init(0);

    }

    /***********Loop for foot switch press routine*********/
    foot_key = 1;
    if (Treat_mode)
          {
    	while ((!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)))//while(foot_key==1)// while ((!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)))
    {

    	HAL_UART_Transmit(&huart1, Disable_Screen, 10, 100);
        MX_TIM6_Init(Duration);

        MX_TIM7_Init(Repeat_Interval);
       //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // Shutter pin//Commented on 17.05.2022

       //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET); // Reset 532 Laser trigger

      //  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Power_Set_value);   // LASER 532 DAC OUT to 0*/
      //  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, Aim_Set_value);   // LASER 635 DAC OUT to 0*/


        if((!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))&& (foot_key==1))//if(foot_key==1)   //if((!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))&& (foot_key==1))
        {

         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);//For Board2


        //single_spot();

         get_pattern_type(mh);


         laser_check();

         HAL_Delay(1);

         foot_key = 0;

         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);//For Board2


        }

        HAL_TIM_Base_Stop_IT(&htim6); // OFF timer
        HAL_TIM_Base_Stop_IT(&htim7); // OFF timer
        HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1); // OFF PWM
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); // Reset 532 Laser trigger
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);   // LASER 532 DAC  OUT to 0

        TIM3->CCR1 = 0;

        HAL_Delay(1);
        HAL_UART_Transmit(&huart1, Enable_Screen, 10, 100);
      }
       //HAL_UART_Transmit(&huart1, Enable_Screen, 10, 100);
     }
    } // Main Loop

}

void laser_check()
{
	if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10))
	    {

	    	laser_temp = 1;//Ok =1//C10 is for laser Temperature signal from the laser Driver

	    }
	    else
	    {
	      laser_temp = 0;
	    }
	    //LASER READY check/

	 if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11))

	    {

	      laser_ready = 0;//C11 is for laser Ready signal from the laser Driver - Not Connected
	      //HAL_UART_Transmit(&huart1, Clear_Not_Ready_Buffer, 14, 100);
	      HAL_UART_Transmit(&huart1, Ready_Buffer, 8, 100);//Ready Buffer ON //Added for DWIN Display
	    }
	    else
	    {
	      laser_ready = 1;
	     // HAL_UART_Transmit(&huart1, Clear_Ready_Buffer, 11, 100);
	      HAL_UART_Transmit(&huart1, Not_Ready_Buffer, 8, 100);//Ready Buffer ON //Added for DWIN Display

	    }
}


/* function to initialize the system to default settings GPIO and peripherals on controls are done through HAL */
void initialize_system()
{

  // Start internal DAC To convert digital value to analog equivalents
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

  DAC->CR|=DAC_CR_BOFF1;
  DAC->CR|=DAC_CR_BOFF2;
  /* Green LASER connected to DAC Channel 1 LASER 532 DAC OUT to 0 as default  */
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);

  /*  Red LASER connected to DAC Channel 2 LASER 635 DAC OUT to 0 as default  */
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);

  // start internal ADC to convert analog signal to digital value
  HAL_ADC_Start(&hadc1);

  /*set digital output pins to zero*/
  /*  GPIOC.2 -> 532nm LASER trigger
      GPIOC.9 -> LASER status*/

  /*set digital output pins to High*/
  /*  GPIOC.0 -> Shutter
      GPIOB.12-> CS pin mirror*/

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // Reset Shutter pin//PC0//SET
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); // Reset 532 Laser trigger
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); // System fail trigger
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); // Laser status
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET); // SBC strobe OUT
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); // LASER TEMP
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // LASER READY
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // CS pin mirror
  // Initialize UART1 to transmit and receive commands
  HAL_UART_Init(&huart1);
  HAL_UART_Init(&huart3);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // System fail trigger

  // By default on start up system to be on standby
  Stand_by_mode = ON;
  Treat_mode = OFF;

  /* Mirror closed loop setting  */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);// Chip select pin set high
  HAL_Delay(4);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Chip select pin set low
  HAL_Delay(4);
  HAL_SPI_Transmit(&hspi2, (uint8_t*)&Kp, 14, 100);// set mirror Kp
  HAL_Delay(4);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);// Chip select pin set high
  HAL_Delay(4);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Chip select pin set low
  HAL_Delay(4);
  HAL_SPI_Transmit(&hspi2, (uint8_t*)&Ki, 14, 100);// set mirror Ki
  HAL_Delay(4);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);// Chip select pin set high
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Chip select pin set low
  HAL_Delay(4);
  HAL_SPI_Transmit(&hspi2, (uint8_t*)&Kd, 14, 100);// set mirror Kd
  HAL_Delay(4);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);// Chip select pin set high
  HAL_Delay(4);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Chip select pin set low
  HAL_Delay(4);
  HAL_SPI_Transmit(&hspi2, (uint8_t*)&Cutoff_frequency, 14, 100);// set mirror Kp
  HAL_Delay(4);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);// Chip select pin set high
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Chip select pin set low
  HAL_Delay(1);
  HAL_SPI_Transmit(&hspi2, (uint8_t*)&DATA_mode, 14, 100);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);// Chip select pin set high
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Chip select pin set low
  HAL_Delay(10);
  HAL_SPI_Transmit(&hspi2, (uint8_t*)&DATA_zero7, 14, 100);// set mirror zero position command
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);// Chip select pin set high
  HAL_Delay(100);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);// Shutter pin

  HAL_Delay(1100);

 //SS//TEMP COMMENTED ON 20.01.2023 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // Shutter pin//Commented on 17.05.2022 //Eswar //Eswar

  HAL_Delay(1100);
}

// loop where main functions of the program starts to execute
void single_spot_red()
{
	MoveTocentre();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	while (Red_break == 0)
	  {}
}
// Single Spot
void single_spot()
{
  //MoveTocentre();
  Buzzer_Set_value = Buzzer;//get_buzzer_value(Buzzer);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, Aim_Set_value);   // LASER 635 DAC OUT to 0*/

  MX_TIM6_Init(Duration-1);
  MX_TIM7_Init(Repeat_Interval);

  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Power_Set_value);   // LASER 532 DAC OUT to 0
 // HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, Aim_Set_value);   // LASER 635 DAC OUT to 0*/
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1); // ON PWM
  TIM3->CCR1 = Buzzer_Set_value;
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET); // Reset 532 Laser trigger

     while (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))
     {
        // foot pedal loop for repeated shots
     }


  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); // Reset 532 Laser trigger
  HAL_TIM_Base_Stop_IT(&htim6); // OFF timer
  HAL_TIM_Base_Stop_IT(&htim7); // ON timer

  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1); // ON PWM


}
//////////////////////////////////////////////////////////////////////////////////////////////////

void MoveTocentre()
{
	  x = 0;  y = 0;

	  union
	    {
	      float f;
	      uint32_t u;
	    }
	    f2u = { .f = x };

	      union
	      {
	        float f;
	        uint32_t v;
	      }
	      f3v = { .f = y };

	      x_location_a[0][0] = f2u.u;
	      y_location_a[0][0] = f3v.v;

	      x_grid[0]=x_location_a[0][0];
	      y_grid[0]=y_location_a[0][0];

	      data_4_4 = (uint8_t)(x_grid[0] & 0xff);
	      data_3_3 = (uint8_t)(x_grid[0] >> 8) & 0xff;
	      data_2_2 = (uint8_t)(x_grid[0] >> 16) & 0xff;
	      data_1_1 = (uint8_t)(x_grid[0] >> 24) & 0xff;

	      data_8_8 = (uint8_t)(y_grid[0] & 0xff);
	      data_7_7 = (uint8_t)(y_grid[0] >> 8) & 0xff;
	      data_6_6 = (uint8_t)(y_grid[0] >> 16) & 0xff;
	      data_5_5 = (uint8_t)(y_grid[0] >> 24) & 0xff;


	      DATA_zero7[6] = data_1_1;
	      DATA_zero7[7] = data_2_2;
	      DATA_zero7[8] = data_3_3;
	      DATA_zero7[9] = data_4_4;
	      DATA_zero7[10] = data_5_5;
	      DATA_zero7[11] = data_6_6;
	      DATA_zero7[12] = data_7_7;
	      DATA_zero7[13] = data_8_8;


	/* Mirror closed loop setting  */
	if(PID==1)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);// Chip select pin set high
		HAL_Delay(1);//4
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Chip select pin set low
		HAL_Delay(1);//4
		HAL_SPI_Transmit(&hspi2, (uint8_t*)&Kp_L, 14, 100);// set mirror Kp
		HAL_Delay(2);//4
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);// Chip select pin set high
		HAL_Delay(1);//4
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Chip select pin set low
		HAL_Delay(1);//4
		HAL_SPI_Transmit(&hspi2, (uint8_t*)&Ki_L, 14, 100);// set mirror Ki
		HAL_Delay(2);//4
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);// Chip select pin set high
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Chip select pin set low
		HAL_Delay(1);//4
		HAL_SPI_Transmit(&hspi2, (uint8_t*)&Kd_L, 14, 100);// set mirror Kd
		HAL_Delay(2);//4
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);// Chip select pin set high
		HAL_Delay(1);//4
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Chip select pin set low
		HAL_Delay(1);//4
		HAL_SPI_Transmit(&hspi2, (uint8_t*)&Cutoff_frequency_L, 14, 100);// set mirror Kp
		HAL_Delay(2);//4
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);// Chip select pin set high
		HAL_Delay(1);//4
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Chip select pin set low
		HAL_Delay(1);//4
		HAL_SPI_Transmit(&hspi2, (uint8_t*)&Kp, 14, 100);// set mirror Kp
		HAL_Delay(2);//4
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);// Chip select pin set high
		HAL_Delay(1);//4
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Chip select pin set low
		HAL_Delay(1);//4
		HAL_SPI_Transmit(&hspi2, (uint8_t*)&Ki, 14, 100);// set mirror Ki
		HAL_Delay(2);//4
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);// Chip select pin set high
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Chip select pin set low
		HAL_Delay(1);//4
		HAL_SPI_Transmit(&hspi2, (uint8_t*)&Kd, 14, 100);// set mirror Kd
		HAL_Delay(2);//4
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);// Chip select pin set high
		HAL_Delay(1);//4
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Chip select pin set low
		HAL_Delay(1);//4
		HAL_SPI_Transmit(&hspi2, (uint8_t*)&Cutoff_frequency, 14, 100);// set mirror Kp
		HAL_Delay(2);//4
	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);// Chip select pin set high
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Chip select pin set low
	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi2, (uint8_t*)&DATA_mode, 14, 100);
	HAL_Delay(1);//4
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);// Chip select pin set high
	HAL_Delay(1);//4
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Chip select pin set low
	HAL_Delay(1);//4
	HAL_SPI_Transmit(&hspi2, (uint8_t*)&DATA_zero7, 14, 100);// set mirror zero position command
	HAL_Delay(1);//4
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);// Chip select pin set high
	HAL_Delay(1);//4
}


void string_to_char_array(char str_data[])
{
  int str_data_length = strlen(str_data);
  int temp_cnt = 0;
  for (temp_cnt = 0; temp_cnt <= str_data_length; temp_cnt++)
  {
    serial_data_message[temp_cnt] = str_data[temp_cnt];
  }

}


// Duration interval timer
void Duration_timer()
{
	//r_foot_key=0;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); // Reset 532 Laser trigger

  HAL_TIM_Base_Stop_IT(&htim6);
  HAL_TIM_Base_Stop_IT(&htim7);


  HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1); // OFF PWM

  if (Stand_by_mode == OFF)
  {

    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, Aim_Set_value);   // LASER 635 DAC OUT to 0*/
    TIM3->CCR1 = 0; // Buzzer

   	HAL_TIM_Base_Start_IT(&htim7);

  }

}


// repeat interval timer
void Repeat_timer()
{

  HAL_TIM_Base_Stop_IT(&htim6);
  HAL_TIM_Base_Stop_IT(&htim7);
  HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1); // OFF PWM

  if (Stand_by_mode == OFF)
  {
    TIM3->CCR1 = Buzzer_Set_value;
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, Aim_Set_value);   // LASER 635 DAC OUT to 0*/

    HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1); //  PWM ON
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Power_Set_value);   // LASER 532 DAC OUT to 0
    HAL_TIM_Base_Start_IT(&htim6);// ON timer
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);

  }

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if(GPIO_Pin == GPIO_PIN_4) // If The INT Source Is EXTI Line4 - PB4 ////For Pattern Change
	        {
	        	Red_break = 1;
	            //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
	        	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
	        }

	if(GPIO_Pin == GPIO_PIN_1) // If The INT Source Is EXTI Line1 - PA1 ////For Foot Switch
	          {

	    	Red_break = 1;


	    	GPIOC->ODR|=GPIO_PIN_8;

	          }


    if(GPIO_Pin == GPIO_PIN_11)
    {

    	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); // Laser status

    	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1); // Toggle The Output (LED) Pin
    }
}


// timer ISR
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim6)
  {
    Duration_timer();

  }
  if (htim == &htim7)
  {
    Repeat_timer();
    if(Repeat_Interval == 0 )
            {
              count1 = count1+1;
              Update_Shot_Count = Update_Shot_Count + 1;
              Shot_Count = count1;
             }

            else

            {
              count1 = count1+1;
              Update_Shot_Count = Update_Shot_Count + 1;
             Shot_Count = count1;

            }

       Spot_Count_Display[7]=(uint8_t) ((Shot_Count) & 0xff);
       Spot_Count_Display[6]=(uint8_t) (((Shot_Count)>>8) & 0xff);

       HAL_UART_Transmit(&huart1, Spot_Count_Display, 8, 100);

  }

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//Added for DWIN
{

	HAL_UART_Receive_IT(&huart1, RxBuffer,sizeof(RxBuffer));


	  if (1)
	  {
	    nh0 = RxBuffer[0];
	 if(nh0==0x5A)
	  {
		nh0=36;
	  }
	    nh1 = (char)RxBuffer[1];

	    switch ((char)nh0)
	    {
	      case '$':
	         stone();

	        break;

	      default:

	        break;
	    }
	  }
	  else
	  {

	  }
}

void stone()//Added for DWIN
{
	if (RxBuffer[0]==0x5A)
	      {
	        nh6 = RxBuffer[4];
	        nh7 = RxBuffer[5];

	        address = (nh6*256)+nh7;//Converting the Hexadecimal address to Decimal

	        switch (address)
	        {

	        case 9472://For Treat/Standby 0x2500

	        	Main_loop_ON=(RxBuffer[7]<<8 )+ RxBuffer[8];
	        //	Main_loop_ON=RxBuffer[8];
	        	if(Main_loop_ON==00)
	        	{

	             HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); // Laser status
	             Transmit = RxBuffer[0];

	             HAL_UART_Transmit(&huart3,&Transmit,1,100);
	             GPIOC->ODR|=GPIO_PIN_8;
	           	 Red_break = 1;
	        	// HAL_UART_Transmit(&huart1, Clear_Ready_Buffer, 11, 100);//Ready Buffer ON
	        	 HAL_UART_Transmit(&huart1, Not_Ready_Buffer, 8, 100);

	        	 HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);   // LASER 635 DAC OUT to 0


	        		if (Update_Shot_Count >= 9998)

	        		{

	        		    Total_Shot_Count_1 = Total_Shot_Count_1 + (Update_Shot_Count / 9998);
	        		  //  Total_Shot_Count_1 = 0;
	        		    Total_SC_Value_1[7]= (uint8_t)(Total_Shot_Count_1 & 0xff);
	        		    Total_SC_Value_1[6] = (uint8_t)(Total_Shot_Count_1 >> 8) & 0xff;
	        		    HAL_UART_Transmit(&huart1, Total_SC_Value_1, 8, 100);
	        		    Flash_Shot_Count_1[6]=0x00;
	        		    HAL_UART_Transmit(&huart1, Flash_Shot_Count_1, 14, 100);
	        		    Flash_Shot_Count_1[6]=0xA5;
	        		    HAL_UART_Transmit(&huart1, Flash_Shot_Count_1, 14, 100);
	        		    Update_Shot_Count = Update_Shot_Count % 10;
	        		    //Total_Shot_Count = 0;

	        		}

	        		else

	        		{
	        			//Total_Shot_Count =  Update_Shot_Count;
	        			//Update_Shot_Count =0;
	        			Total_SC_Value[7]= (uint8_t)(Update_Shot_Count & 0xff);
	        			Total_SC_Value[6] = (uint8_t)(Update_Shot_Count >> 8) & 0xff;
	        			HAL_UART_Transmit(&huart1, Total_SC_Value, 8, 100);
	        			Flash_Shot_Count[6]=0x00;
	        			HAL_UART_Transmit(&huart1, Flash_Shot_Count, 14, 100);
	        			Flash_Shot_Count[6]=0xA5;
	        			HAL_UART_Transmit(&huart1, Flash_Shot_Count, 14, 100);
	        			//Update_Shot_Count = 0;
	        		}

	            }

	        	 break;

	          case 4096://For Power(0x1000)
	        	  Power_watts = (RxBuffer[7]<<8 )+ RxBuffer[8];
	        	  MJ = Power_watts *Duration;///1000;
	        	  if(eXs == 0)

	        	  {
	        		  if(MJ>14999)
	        		  {
	        			  HAL_UART_Transmit(&huart1, Exs_Page, 10, 100);
	        		  }
	        	  }

             break;

	          case 4097://For Exposure Duration(0x1001)
	        	  Duration = (RxBuffer[7]<<8) + RxBuffer[8];
	        	  MJ = Power_watts *Duration;///1000;
	        	  if(eXs == 0)
	        	  {
	        	      if(MJ>14999)
	        	     {
	        		   HAL_UART_Transmit(&huart1, Exs_Page, 10, 100);
	        	     }
	        	  }

	            break;
	          case 37888 ://9400 - For eXs
	        	  eXs=(RxBuffer[7]<<8) + RxBuffer[8];
	        	  MJ = Power_watts *Duration;///1000;
	        	  if(eXs == 0)
	        	  {
	        		  if(MJ>14999)
	        		  {
	        			  HAL_UART_Transmit(&huart1, Exs_Page, 10, 100);
	        		  }
	        	  }

	        	  break;

	          case 12544://For Aim_Intensity 0x3100

	        	  Aim = (RxBuffer[7]<<8) + RxBuffer[8];
	        	  if (Aim == 10)
	        	  {
	        		  Aim = 20;
	        	  }
	        	  if(Aim == 100)
	        	  {
	        		  Aim = 95;
	        	  }
	        	  Transmit = RxBuffer[0];
	        	HAL_UART_Transmit(&huart3,&Transmit,1,100);

	        break;




	          case 4105: //Repeat Interval for Single Spot 0x1009

	        	  Repeat_Interval = (int)((RxBuffer[7]<<8) + RxBuffer[8]);


	          break;



	          case 4120: //For Single Spot 0x1018

	        	  mh='G';PID=0;//eXs = 1;
	        	 Transmit = RxBuffer[0];

	            // HAL_UART_Transmit(&huart1, Exs, 8, 100);
	             if(Adapter == 2||3)
	             {
	            	HAL_UART_Transmit(&huart1, Page_switch_SS, 10, 100);
	            	mh='G';
	             }
	               break;


	          case 25856://For Buzzer 0x6500

             	  Buzzer = (int)((RxBuffer[7]<<8) + RxBuffer[8]);

	        	  break;
	          case  34048:  //0x8500

	        	  Variable_1 = (int)((RxBuffer[7]<<8) + RxBuffer[8]);
	        	  break;
	          case  38224:  //0x9550
	        	  Variable_2 = (int)((RxBuffer[7]<<8) + RxBuffer[8]);
	        	  break;
	          case  38240:  //0x9560

	        	  Variable_3 = (int)((RxBuffer[7]<<8) + RxBuffer[8]);
	        	  break;
	          case  38256:  //0x9570
	        	  Variable_4 = (int)((RxBuffer[7]<<8) + RxBuffer[8]);
	        	  break;
	          case  38272:  //0x9580
	        	  Variable_5 = (int)((RxBuffer[7]<<8) + RxBuffer[8]);
	        	  break;
	          case  34192:  //0x8590
	        	  Variable_6 = (int)((RxBuffer[7]<<8) + RxBuffer[8]);
	        	  break;
	          case  34304:  //0x8600
	        	  Variable_7 = (int)((RxBuffer[7]<<8) + RxBuffer[8]);
	        	  break;
	          case  34320:  //0x8610
	        	  Variable_8 = (int)((RxBuffer[7]<<8) + RxBuffer[8]);
	        	  break;
	          case  34336:  //0x8620
	        	  Variable_9 = (int)((RxBuffer[7]<<8) + RxBuffer[8]);
	        	  break;
	          case  34352:  //0x8630
	        	  Variable_10 = (int)((RxBuffer[7]<<8) + RxBuffer[8]);
	        	  break;


           case 37120: //0x9100 To Calibration Value

	        	  Calib_Flag = 1;
	         	  // xy =((float)(Set_V1)/(float)(Variable_1))+((float)(Set_V2)/(float)(Variable_2))+((float)(Set_V3)/(float)(Variable_3))+((float)(Set_V4)/(float)(Variable_4))+((float)(Set_V5)/(float)(Variable_5))+((float)(Set_V6)/(float)(Variable_6))+((float)(Set_V7)/(float)(Variable_7))+((float)(Set_V8)/(float)(Variable_8))+((float)(Set_V9)/(float)(Variable_9))+((float)(Set_V10)/(float)(Variable_10));
	              /* xy = (Set_V1*Variable_1)+(Set_V2*Variable_2)+(Set_V3*Variable_3)+(Set_V4*Variable_4)+(Set_V5*Variable_5)+(Set_V6*Variable_6)+(Set_V7*Variable_7)+(Set_V8*Variable_8)+(Set_V9*Variable_9)+(Set_V10*Variable_10);
	         	  */
	         	  /*tot_x = Set_V1+Set_V2+Set_V3+Set_V4+Set_V5+Set_V6+Set_V7+Set_V8+Set_V9+Set_V10;
	         	  tot_y = Variable_1+Variable_2+Variable_3+Variable_4+Variable_5+Variable_6+Variable_7+Variable_8+Variable_9+Variable_10;
	         	  tot_x_sq = (Set_V1*Set_V1)+(Set_V2*Set_V2)+(Set_V3*Set_V3)+(Set_V4*Set_V4)+(Set_V5*Set_V5)+(Set_V6*Set_V6)+(Set_V7*Set_V7)+(Set_V8*Set_V8)+(Set_V9*Set_V9)+(Set_V10*Set_V10);
	         	  calibration_value =(float)((no_of_var * xy) - (tot_x*tot_y))/(float)((no_of_var*tot_x_sq)-(tot_x^2));*/
	         	  //  xy_7 = ((float)(Set_V1)/(float)(Variable_1))+((float)(Set_V2)/(float)(Variable_2))+((float)(Set_V3)/(float)(Variable_3))+((float)(Set_V4)/(float)(Variable_4))+((float)(Set_V5)/(float)(Variable_5))+((float)(Set_V6)/(float)(Variable_6))+((float)(Set_V7)/(float)(Variable_7));
	         	  xy =((float)(Set_V1)/(float)(Variable_1))+((float)(Set_V2)/(float)(Variable_2))+((float)(Set_V3)/(float)(Variable_3))+((float)(Set_V4)/(float)(Variable_4))+((float)(Set_V5)/(float)(Variable_5))+((float)(Set_V6)/(float)(Variable_6))+((float)(Set_V7)/(float)(Variable_7))+((float)(Set_V8)/(float)(Variable_8))+((float)(Set_V9)/(float)(Variable_9))+((float)(Set_V10)/(float)(Variable_10));
	         	  //  new_calibration_value_upto_7 = xy_7/7.0;
	         	  //  calibration_value_upto_7 = new_calibration_value_upto_7 *calibration_value_upto_7;
	         	  new_calibration_value = xy/10.0;
	         	  calibration_value = new_calibration_value *calibration_value;
	         	  Calibration = (int)(calibration_value *10000);
	         	  Calib_Value[7]= (uint8_t)(Calibration & 0xff);
	         	  Calib_Value[6] = (uint8_t)(Calibration >> 8) & 0xff;
	         	  HAL_UART_Transmit(&huart1, Calib_Value, 8, 100);
	         	  if (Adapter == 1)
	         	  {
	         		  Flash_write[6]=0x00;
	         	      HAL_UART_Transmit(&huart1, Flash_write, 14, 100);
	         		  Flash_write[6]=0xA5;
	         		  HAL_UART_Transmit(&huart1, Flash_write, 14, 100);
	         	  }
	         	  else if (Adapter == 2)
	         	  {
	         		  Flash_write_LIO[6]=0x00;
	         		  HAL_UART_Transmit(&huart1, Flash_write_LIO, 14, 100);
	         		  Flash_write_LIO[6]=0xA5;
	         		  HAL_UART_Transmit(&huart1, Flash_write_LIO, 14, 100);
	         	  }
	         	  else
	         	  {
	         		  Flash_write_ENDO[6]=0x00;
	         		  HAL_UART_Transmit(&huart1, Flash_write_ENDO, 14, 100);
	         		  Flash_write_ENDO[6]=0xA5;
	         		  HAL_UART_Transmit(&huart1, Flash_write_ENDO, 14, 100);
	         	  }
	         	  break;
	          case 37376: //0x9200 - Back Button from calibration

	        	  Interrupt_Calib = 0;
	        	  break;
	          case 8194://0x2002 - To Display Calibration Value
	        	  calibration_value = (float)((RxBuffer[7]<<8) + RxBuffer[8]);
	        	  calibration_value = calibration_value/10000.0;
            	  break;

	          case 29952://0x7500 - Set Calibration value to 1.00


	        	  calibration_value = 1.00;
	        	  Calibration = (int)(calibration_value *10000);
	        	  Calib_Value[7]= (uint8_t)(Calibration & 0xff);
	        	  Calib_Value[6] = (uint8_t)(Calibration >> 8) & 0xff;
	        	  HAL_UART_Transmit(&huart1, Calib_Value, 8, 100);
	        	  if(Adapter ==1)
	        	  {
	        		  Flash_write[6]=0x00;
	        		  HAL_UART_Transmit(&huart1, Flash_write, 14, 100);
	        		  Flash_write[6]=0xA5;
	        		  HAL_UART_Transmit(&huart1, Flash_write, 14, 100);
	        	  }
	        	  else if (Adapter == 2)
	        	  {
	        		  Flash_write_LIO[6]=0x00;
	        		  HAL_UART_Transmit(&huart1, Flash_write_LIO, 14, 100);
	        		  Flash_write_LIO[6]=0xA5;
	        		  HAL_UART_Transmit(&huart1, Flash_write_LIO, 14, 100);
	        	  }
	        	  else
	        	  {
	        		  Flash_write_ENDO[6]=0x00;
	        		  HAL_UART_Transmit(&huart1, Flash_write_ENDO, 14, 100);
	        		  Flash_write_ENDO[6]=0xA5;
	        		  HAL_UART_Transmit(&huart1, Flash_write_ENDO, 14, 100);
	        	  }
	        	  break;

	          case 8198://0x2006 - To display Calibration Value for second set of values but currently disabled

	        	  calibration_value_upto_7 = (float)((RxBuffer[7]<<8) + RxBuffer[8]);
	        	  calibration_value_upto_7 = calibration_value_upto_7/10000.0;
	        	  break;

	          case 30464://0x7700 - To Display Adapter  variable Icon

	        	  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))
	        	  {
	        		  HAL_UART_Transmit(&huart1, SLA, 8, 100);
	        		  Adapter=1;
	        		  Acc_key_Variable = 0;
	        	  }
	        	  else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
	        	  {
	        		  HAL_UART_Transmit(&huart1, LIO, 8, 100);
	        		  Adapter=2;
	        		  mh='G';
	        		  Acc_key_Variable = 0;
	        		  HAL_UART_Transmit(&huart1, Page_switch_SS, 10, 100);
	        	  }
	        	  else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
	        	  {
	        		  HAL_UART_Transmit(&huart1, ENDO, 8, 100);
	        		  Adapter=3;
	        		  mh='G';
	        		  Acc_key_Variable = 0;
	        		  HAL_UART_Transmit(&huart1, Page_switch_SS, 10, 100);
	        	  }
                 else
                 {
                	 Acc_key_Variable =1 ;
                	 HAL_UART_Transmit(&huart1, Acc_Key, 10, 100);
                 }
	        	  break;


	          case 4944://0x1350 - For Calibration

	        	  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))
	        	  {
	        		 Adapter=1;ACC_K=1;
	        		 Initial_Page_Acc[9]=0x19;
	        		 HAL_UART_Transmit(&huart1, Initial_Page_Acc, 10, 100);

	        	  }

	        	  else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
	        	  {
	        		  Initial_Page_Acc[9]=0x19;
                      Adapter=2;ACC_K=1;
                     HAL_UART_Transmit(&huart1, Initial_Page_Acc, 10, 100);
	        	  }

	        	  else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
	        	  {
	        		  Initial_Page_Acc[9]=0x19;
	        	       Adapter=3; ACC_K=1;
	        	      HAL_UART_Transmit(&huart1, Initial_Page_Acc, 10, 100);

	        	  }
	        	  else
	        	  {
	        		  ACC_K=0;
	        	  }

	           	         Transmit = RxBuffer[0];
	     	         	 Interrupt_Calib = 1;
	     	         	 HAL_UART_Transmit(&huart3,&Transmit,1,100);
	     	     	     Main_loop_ON = 0;Treat_mode = 0;
	     	     	     HAL_UART_Transmit(&huart1, Treat_stand, 8, 100);
	     	         	 if(Adapter == 1)
	     	         	 {
	     	         		 Flash_read[6]=0x00;
	     	         		 HAL_UART_Transmit(&huart1, Flash_read, 14, 100);
	     	         		 Flash_read[6]=0x5A;
	     	         		 HAL_UART_Transmit(&huart1, Flash_read, 14, 100);
	     	         	 }
	     	         	 else if (Adapter == 2)
	     	         	 {
	     	         		 Flash_read_LIO[6]=0x00;
	     	         		 HAL_UART_Transmit(&huart1, Flash_read_LIO, 14, 100);
	     	         		 Flash_read_LIO[6]=0x5A;
	     	         		 HAL_UART_Transmit(&huart1, Flash_read_LIO, 14, 100);
	     	         	 }
	     	         	 else
	     	         	 {
	     	         		 Flash_read_ENDO[6]=0x00;
	     	         		 HAL_UART_Transmit(&huart1, Flash_read_ENDO, 14, 100);
	     	         		 Flash_read_ENDO[6]=0x5A;
	     	         		 HAL_UART_Transmit(&huart1, Flash_read_ENDO, 14, 100);
	     	         	 }
	     	         	Variable_1=100.0;
	     	         	Variable_2=200.0;
	     	         	Variable_3=300.0;
	     	         	Variable_4=400.0;
	     	         	Variable_5=500.0;
	     	         	Variable_6=600.0;
	     	         	Variable_7=700.0;
	     	         	Variable_8=1000.0;
	     	         	Variable_9=1500.0;
	     	         	Variable_10=2000.0;

	     	      	 HAL_UART_Transmit(&huart1, Calib_V1, 8, 100);
	     	      	 HAL_UART_Transmit(&huart1, Calib_V2, 8, 100);
	     	      	 HAL_UART_Transmit(&huart1, Calib_V3, 8, 100);
	     	      	 HAL_UART_Transmit(&huart1, Calib_V4, 8, 100);
	     	      	 HAL_UART_Transmit(&huart1, Calib_V5, 8, 100);
	     	      	 HAL_UART_Transmit(&huart1, Calib_V6, 8, 100);
	     	      	 HAL_UART_Transmit(&huart1, Calib_V7, 8, 100);
	     	      	 HAL_UART_Transmit(&huart1, Calib_V8, 8, 100);
	     	      	 HAL_UART_Transmit(&huart1, Calib_V9, 8, 100);
	     	      	 HAL_UART_Transmit(&huart1, Calib_V10, 8, 100);

	     	      HAL_UART_Transmit(&huart1, Flash_read_variable, 7, 100);

	        	  break;

	          case 29184://0x7200 - To Display zero shot count

	        	  Zero_Shot_Count = 0; count1=0;
	        	  Spot_Count_Display[7]=(uint8_t) (Zero_Shot_Count & 0xff);
	        	  Spot_Count_Display[6]=(uint8_t) ((Zero_Shot_Count>>8) & 0xff);
	        	  HAL_UART_Transmit(&huart1, Spot_Count_Display, 8, 100);
	        	/* Flash_R_Shot_Count[6]=0x00;
	        	  HAL_UART_Transmit(&huart1, Flash_R_Shot_Count, 14, 100);
	        	  Flash_R_Shot_Count[6]=0x5A;
	        	  HAL_UART_Transmit(&huart1, Flash_R_Shot_Count, 14, 100);
	        	  HAL_UART_Transmit(&huart1, Flash_read_variable_1, 7, 100);*/

	         	  break;

	          case 21504://0x5400  //4736: //0x1280 - For Shot Screen

	        	  if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6))
	        	     {
                      RI=1;
                      Initial_Page_RI[9]= 0x21;
					  HAL_UART_Transmit(&huart1, Initial_Page_RI, 10, 100);
					  Initial_Page_RI[9]= 0x05;
	        	     }
	        	     else
	        	     {
	        	       RI=0;
	        	      // HAL_UART_Transmit(&huart1, RI_Buffer_NC, 20, 100);
	        	     }

	        	  Transmit = RxBuffer[0];

	        	  HAL_UART_Transmit(&huart3,&Transmit,1,100);
	        	  Flash_R_Shot_Count[6]=0x00;
	        	  HAL_UART_Transmit(&huart1, Flash_R_Shot_Count, 14, 100);
	        	  Flash_R_Shot_Count[6]=0x5A;
	        	  HAL_UART_Transmit(&huart1, Flash_R_Shot_Count, 14, 100);

	        	  Variable_1=100.0;
	        	  Variable_2=200.0;
	        	  Variable_3=300.0;
	        	  Variable_4=400.0;
	        	  Variable_5=500.0;
	        	  Variable_6=600.0;
	        	  Variable_7=700.0;
	        	  Variable_8=1000.0;
	        	  Variable_9=1500.0;
	        	  Variable_10=2000.0;

	        	  HAL_UART_Transmit(&huart1, Calib_V1, 8, 100);
	        	  HAL_UART_Transmit(&huart1, Calib_V2, 8, 100);
	        	  HAL_UART_Transmit(&huart1, Calib_V3, 8, 100);
	        	  HAL_UART_Transmit(&huart1, Calib_V4, 8, 100);
	        	  HAL_UART_Transmit(&huart1, Calib_V5, 8, 100);
	        	  HAL_UART_Transmit(&huart1, Calib_V6, 8, 100);
	        	  HAL_UART_Transmit(&huart1, Calib_V7, 8, 100);
	        	  HAL_UART_Transmit(&huart1, Calib_V8, 8, 100);
	        	  HAL_UART_Transmit(&huart1, Calib_V9, 8, 100);
	        	  HAL_UART_Transmit(&huart1, Calib_V10, 8, 100);

	        	  HAL_UART_Transmit(&huart1, Flash_read_variable_1, 7, 100);

	          	  break;
	          case 22016://0x5600 - For Shot count read

	        	  if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
	        	      {
                          FS=1;
                          Initial_Footswitch_Buffer[9]=0x22;
	        		  HAL_UART_Transmit(&huart1, Initial_Footswitch_Buffer, 10, 100);
	        		  Initial_Footswitch_Buffer[9]=0x0B;

	        	      }
	        	      else
	        	      {
	        	    	  FS=0;
	        	    	 // HAL_UART_Transmit(&huart1, FS_Buffer_NC, 20, 100);
	        	      }
	        	  Transmit = RxBuffer[0];
	        	  HAL_UART_Transmit(&huart3,&Transmit,1,100);

	        	  Flash_R_Shot_Count_1[6]=0x00;
	        	  HAL_UART_Transmit(&huart1, Flash_R_Shot_Count_1, 14, 100);
	        	  Flash_R_Shot_Count_1[6]=0x5A;

	        	  HAL_UART_Transmit(&huart1, Flash_R_Shot_Count_1, 14, 100);

	        	  Variable_1=100.0;
	        	  Variable_2=200.0;
	        	  Variable_3=300.0;
	        	  Variable_4=400.0;
	        	  Variable_5=500.0;
	        	  Variable_6=600.0;
	        	  Variable_7=700.0;
	        	  Variable_8=1000.0;
	        	  Variable_9=1500.0;
	        	  Variable_10=2000.0;


	        	  HAL_UART_Transmit(&huart1, Calib_V1, 8, 100);
	        	  HAL_UART_Transmit(&huart1, Calib_V2, 8, 100);
	        	  HAL_UART_Transmit(&huart1, Calib_V3, 8, 100);
	        	  HAL_UART_Transmit(&huart1, Calib_V4, 8, 100);
	        	  HAL_UART_Transmit(&huart1, Calib_V5, 8, 100);
	        	  HAL_UART_Transmit(&huart1, Calib_V6, 8, 100);
	        	  HAL_UART_Transmit(&huart1, Calib_V7, 8, 100);
	        	  HAL_UART_Transmit(&huart1, Calib_V8, 8, 100);
	        	  HAL_UART_Transmit(&huart1, Calib_V9, 8, 100);
	        	  HAL_UART_Transmit(&huart1, Calib_V10, 8, 100);

	        	  HAL_UART_Transmit(&huart1, Flash_read_variable_2, 7, 100);

	        	  break;

	          case 8208://0x2010 - To write into flash for shot count

	        	  Update_Shot_Count = (float)((RxBuffer[7]<<8) + RxBuffer[8]);
	        	  Main_loop_ON = 0;Treat_mode = 0;
	        	  HAL_UART_Transmit(&huart1, Treat_stand, 8, 100);
	        	  Total_SC_Value[7]= (uint8_t)(Update_Shot_Count & 0xff);
	        	  Total_SC_Value[6] = (uint8_t)(Update_Shot_Count >> 8) & 0xff;
	        	  HAL_UART_Transmit(&huart1, Total_SC_Value, 8, 100);
	        	  break;

	           case 8224://To write in K - shot count - 0x2020

	        	   Total_Shot_Count_1 = (float)((RxBuffer[7]<<8) + RxBuffer[8]);
	               Main_loop_ON = 0;Treat_mode = 0;
	        	   HAL_UART_Transmit(&huart1, Treat_stand, 8, 100);
	        	   Total_SC_Value_1[7]= (uint8_t)(Total_Shot_Count_1 & 0xff);
	        	   Total_SC_Value_1[6] = (uint8_t)(Total_Shot_Count_1 >> 8) & 0xff;
	        	   HAL_UART_Transmit(&huart1, Total_SC_Value_1, 8, 100);

	        	   break;


	           case 10544://0x2930 for value 0 - p1

	        	   p1 = RxBuffer[8];
	        	   p1=0,p2=0,p3=0,p4=0,p5=0,p6 =0;
	        	   break;

	           case 8497://0x2131 for value 1 - p2 and p3

                   if((p3 == 0) && (p4==0) && (p5==0) && (p6==0))
                   {
	        	      if(p2 == 1)
	        	      {
	        		    p3 = p2;
	        	      }
	        	      else
	        	        p2 = RxBuffer[8];
	        	   }
                   else
                   {
                	   p1 = p2 = p3 = p4 = p5 = p6 = 0;
                   }
	        	 break;

	           case 16434://0x4032 for value 2 - p4 and p5

	        	   if((p5==0) && (p6==0))
	        	   {
	        	       if(p4 == 2)
	        	       {
	        	   	     p5 = p4;
	        	       }
	        	       else
	        		     p4 = RxBuffer[8];
	        	   }
	        	   else
	        	   {
	        		   p1 = p2 = p3 = p4 = p5 = p6 = 0;
	        	   }
	        	  break;
	           case 9011://0x2333 for value 3 - p6
	        	   if((p6==0))
	        	   {

	        	     p6 = RxBuffer[8];
	        	   }
	        	   else
	        	   {

	        		   p1 = p2 = p3 = p4 = p5 = p6 = 0;
	        	   }

	        	   break;

	           case 13104://0x3330 for OK

	        	   if(( p1 == 0) && ( p2 == 1)&&( p3 == 1)&&( p4 == 2)&&( p5 == 2)&&( p6 == 3))
	        	   {

	        	    HAL_UART_Transmit(&huart1, Page_Calibration, 10, 100); //
	        	    p1 = p2 = p3 = p4 = p5 = p6 = 0;
	        	   }

	        	   p1=0,p2=0,p3=0,p4=0,p5=0,p6 =0;

	        	   break;
	           case 5717://0x1655 for spot size interrupt

	        	   Transmit = RxBuffer[0];

	        	   HAL_UART_Transmit(&huart3,&Transmit,1,100);
	        	   break;

	           case 4160://0x1040 For Square blocks

	        	   if( Adapter==1)
	        	   {
	        		  HAL_UART_Transmit(&huart1, Square_Blocks, 10, 100);
	        		 // HAL_UART_Transmit(&huart1, Page_switch, 10, 100);
	        	   }
	        	   else
	        		   HAL_UART_Transmit(&huart1, Page_switch, 10, 100);
	        	   break;

	           case 4162://0x1042 For Circle Blocks

	        	   if( Adapter==1)
	        	   {
	        		  HAL_UART_Transmit(&huart1, Circle_Blocks, 10, 100);
	        	   }
	        	   else
	        		  HAL_UART_Transmit(&huart1, Page_switch, 10, 100);

	        	   break;

	           case 4164://0x1044 For Special Blocks

	        	   if( Adapter==1)
	        	   {
	        	  	  HAL_UART_Transmit(&huart1, Special_Blocks, 10, 100);
	        	   }
	        	   else
	        	  	  HAL_UART_Transmit(&huart1, Page_switch, 10, 100);
	          	   break;

	           case 8498://0x2132 for m
	        	   if(Adapter ==1)
					  {
						  Flash_write_m_SLA[6]=0x00;
						  HAL_UART_Transmit(&huart1, Flash_write_m_SLA, 14, 100);
						  Flash_write_m_SLA[6]=0xA5;
						  HAL_UART_Transmit(&huart1, Flash_write_m_SLA, 14, 100);
					  }
					  else if (Adapter == 2)
					  {
						  Flash_write_m_LIO[6]=0x00;
						  HAL_UART_Transmit(&huart1, Flash_write_m_LIO, 14, 100);
						  Flash_write_m_LIO[6]=0xA5;
						  HAL_UART_Transmit(&huart1, Flash_write_m_LIO, 14, 100);
					  }
					  else
					  {
						  Flash_write_m_ENDO[6]=0x00;
						  HAL_UART_Transmit(&huart1, Flash_write_m_ENDO, 14, 100);
						  Flash_write_m_ENDO[6]=0xA5;
						  HAL_UART_Transmit(&huart1, Flash_write_m_ENDO, 14, 100);
					  }

   /**********************************************************************************/
				Variable_1=100.0;
				Variable_2=200.0;
				Variable_3=300.0;
				Variable_4=400.0;
				Variable_5=500.0;
				Variable_6=600.0;
				Variable_7=700.0;
				Variable_8=1000.0;
				Variable_9=1500.0;
				Variable_10=2000.0;

				 HAL_UART_Transmit(&huart1, Calib_V1, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V2, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V3, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V4, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V5, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V6, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V7, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V8, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V9, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V10, 8, 100);

		/**********************************************************************************/
	        	   HAL_UART_Transmit(&huart1, Flash_read_m, 7, 100);

	        	   break;


	           case 8514://0x2142 for c
	        	   if(Adapter ==1)
				  {
					  Flash_write_c_SLA[6]=0x00;
					  HAL_UART_Transmit(&huart1, Flash_write_c_SLA, 14, 100);
					  Flash_write_c_SLA[6]=0xA5;
					  HAL_UART_Transmit(&huart1, Flash_write_c_SLA, 14, 100);
				  }
				  else if (Adapter == 2)
				  {
					  Flash_write_c_LIO[6]=0x00;
					  HAL_UART_Transmit(&huart1, Flash_write_c_LIO, 14, 100);
					  Flash_write_c_LIO[6]=0xA5;
					  HAL_UART_Transmit(&huart1, Flash_write_c_LIO, 14, 100);
				  }
				  else
				  {
					  Flash_write_c_ENDO[6]=0x00;
					  HAL_UART_Transmit(&huart1, Flash_write_c_ENDO, 14, 100);
					  Flash_write_c_ENDO[6]=0xA5;
					  HAL_UART_Transmit(&huart1, Flash_write_c_ENDO, 14, 100);
				  }

		   /**********************************************************************************/
						Variable_1=100.0;
						Variable_2=200.0;
						Variable_3=300.0;
						Variable_4=400.0;
						Variable_5=500.0;
						Variable_6=600.0;
						Variable_7=700.0;
						Variable_8=1000.0;
						Variable_9=1500.0;
						Variable_10=2000.0;

						 HAL_UART_Transmit(&huart1, Calib_V1, 8, 100);
						 HAL_UART_Transmit(&huart1, Calib_V2, 8, 100);
						 HAL_UART_Transmit(&huart1, Calib_V3, 8, 100);
						 HAL_UART_Transmit(&huart1, Calib_V4, 8, 100);
						 HAL_UART_Transmit(&huart1, Calib_V5, 8, 100);
						 HAL_UART_Transmit(&huart1, Calib_V6, 8, 100);
						 HAL_UART_Transmit(&huart1, Calib_V7, 8, 100);
						 HAL_UART_Transmit(&huart1, Calib_V8, 8, 100);
						 HAL_UART_Transmit(&huart1, Calib_V9, 8, 100);
						 HAL_UART_Transmit(&huart1, Calib_V10, 8, 100);

				/**********************************************************************************/

	        	   HAL_UART_Transmit(&huart1, Flash_read_c, 7, 100);
	        	   break;

	           case 22272://0x5700 - To Read m

	        	   if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6))
					 {
						RI=1;
					  HAL_UART_Transmit(&huart1, Initial_Page_RI, 10, 100);

					 }
					 else
					 {
					   RI=0;
					  // HAL_UART_Transmit(&huart1, RI_Buffer_NC, 20, 100);
					 }
				  Transmit = RxBuffer[0];
				  HAL_UART_Transmit(&huart3,&Transmit,1,100);

					  if(Adapter == 1)
					 {
						 Flash_read_m_SLA[6]=0x00;
						 HAL_UART_Transmit(&huart1, Flash_read_m_SLA, 14, 100);
						 Flash_read_m_SLA[6]=0x5A;
						 HAL_UART_Transmit(&huart1, Flash_read_m_SLA, 14, 100);
					 }
					 else if (Adapter == 2)
					 {
						 Flash_read_m_LIO[6]=0x00;
						 HAL_UART_Transmit(&huart1, Flash_read_m_LIO, 14, 100);
						 Flash_read_m_LIO[6]=0x5A;
						 HAL_UART_Transmit(&huart1, Flash_read_m_LIO, 14, 100);
					 }
					 else
					 {
						 Flash_read_m_ENDO[6]=0x00;
						 HAL_UART_Transmit(&huart1, Flash_read_m_ENDO, 14, 100);
						 Flash_read_m_ENDO[6]=0x5A;
						 HAL_UART_Transmit(&huart1, Flash_read_m_ENDO, 14, 100);
					 }
					Variable_1=100.0;
					Variable_2=200.0;
					Variable_3=300.0;
					Variable_4=400.0;
					Variable_5=500.0;
					Variable_6=600.0;
					Variable_7=700.0;
					Variable_8=1000.0;
					Variable_9=1500.0;
					Variable_10=2000.0;

				 HAL_UART_Transmit(&huart1, Calib_V1, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V2, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V3, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V4, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V5, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V6, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V7, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V8, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V9, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V10, 8, 100);

			     HAL_UART_Transmit(&huart1, Flash_read_m, 7, 100);


	    	   break;

	           case 8242://0x2032 - To Display m Value
				  m_value = (float)((RxBuffer[7]<<8) + RxBuffer[8]);
				  m_value = m_value/100000.0;
				  break;


	           case 22528://0x5800 - To Read c
	        	   if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
					  {
						   FS=1;

					  HAL_UART_Transmit(&huart1, Initial_Footswitch_Buffer, 10, 100);


					  }
					  else
					  {
						  FS=0;
						 // HAL_UART_Transmit(&huart1, FS_Buffer_NC, 20, 100);
					  }


				  Transmit = RxBuffer[0];
				  HAL_UART_Transmit(&huart3,&Transmit,1,100);

					  if(Adapter == 1)
					 {
						 Flash_read_c_SLA[6]=0x00;
						 HAL_UART_Transmit(&huart1, Flash_read_c_SLA, 14, 100);
						 Flash_read_c_SLA[6]=0x5A;
						 HAL_UART_Transmit(&huart1, Flash_read_c_SLA, 14, 100);
					 }
					 else if (Adapter == 2)
					 {
						 Flash_read_c_LIO[6]=0x00;
						 HAL_UART_Transmit(&huart1, Flash_read_c_LIO, 14, 100);
						 Flash_read_c_LIO[6]=0x5A;
						 HAL_UART_Transmit(&huart1, Flash_read_c_LIO, 14, 100);
					 }
					 else
					 {
						 Flash_read_c_ENDO[6]=0x00;
						 HAL_UART_Transmit(&huart1, Flash_read_c_ENDO, 14, 100);
						 Flash_read_c_ENDO[6]=0x5A;
						 HAL_UART_Transmit(&huart1, Flash_read_c_ENDO, 14, 100);
					 }
					Variable_1=100.0;
					Variable_2=200.0;
					Variable_3=300.0;
					Variable_4=400.0;
					Variable_5=500.0;
					Variable_6=600.0;
					Variable_7=700.0;
					Variable_8=1000.0;
					Variable_9=1500.0;
					Variable_10=2000.0;

				 HAL_UART_Transmit(&huart1, Calib_V1, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V2, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V3, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V4, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V5, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V6, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V7, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V8, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V9, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V10, 8, 100);

				 HAL_UART_Transmit(&huart1, Flash_read_c, 7, 100);


	          	 break;

	           case 8258://0x2042 - From Display m Value
				  c_value = (float)((RxBuffer[7]<<8) + RxBuffer[8]);
				  c_value = c_value/10000.0;
				  break;

	           case 8530:// To save Aim_single spot - 0x2152


	        	  Flash_write_Aim_S[6]=0x00;
				  HAL_UART_Transmit(&huart1, Flash_write_Aim_S, 14, 100);
				  Flash_write_Aim_S[6]=0xA5;
				  HAL_UART_Transmit(&huart1, Flash_write_Aim_S, 14, 100);


	/**********************************************************************************/
				Variable_1=100.0;
				Variable_2=200.0;
				Variable_3=300.0;
				Variable_4=400.0;
				Variable_5=500.0;
				Variable_6=600.0;
				Variable_7=700.0;
				Variable_8=1000.0;
				Variable_9=1500.0;
				Variable_10=2000.0;

				 HAL_UART_Transmit(&huart1, Calib_V1, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V2, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V3, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V4, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V5, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V6, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V7, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V8, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V9, 8, 100);
				 HAL_UART_Transmit(&huart1, Calib_V10, 8, 100);

		/**********************************************************************************/
				  HAL_UART_Transmit(&huart1, Flash_read_Aim, 7, 100);

	        	   break;

	           case 8274://0x2052 To Read Aim_S value for default setting

	               Aim_S = (float)((RxBuffer[7]<<8) + RxBuffer[8]);

	        	   break;

	           case 22784://0x5900 For Reading Aim_S
	        	   Initial_Page_Acc[9]=0x0C;

	        	   HAL_UART_Transmit(&huart1, Initial_Page_Acc, 10, 100);

	        	   Flash_read_Aim_S[6]=0x00;
	        	   HAL_UART_Transmit(&huart1, Flash_read_Aim_S, 14, 100);
	        	   Flash_read_Aim_S[6]=0x5A;
	        	   HAL_UART_Transmit(&huart1, Flash_read_Aim_S, 14, 100);

	        		Variable_1=100.0;
					Variable_2=200.0;
					Variable_3=300.0;
					Variable_4=400.0;
					Variable_5=500.0;
					Variable_6=600.0;
					Variable_7=700.0;
					Variable_8=1000.0;
					Variable_9=1500.0;
					Variable_10=2000.0;

					 HAL_UART_Transmit(&huart1, Calib_V1, 8, 100);
					 HAL_UART_Transmit(&huart1, Calib_V2, 8, 100);
					 HAL_UART_Transmit(&huart1, Calib_V3, 8, 100);
					 HAL_UART_Transmit(&huart1, Calib_V4, 8, 100);
					 HAL_UART_Transmit(&huart1, Calib_V5, 8, 100);
					 HAL_UART_Transmit(&huart1, Calib_V6, 8, 100);
					 HAL_UART_Transmit(&huart1, Calib_V7, 8, 100);
					 HAL_UART_Transmit(&huart1, Calib_V8, 8, 100);
					 HAL_UART_Transmit(&huart1, Calib_V9, 8, 100);
					 HAL_UART_Transmit(&huart1, Calib_V10, 8, 100);

					 HAL_UART_Transmit(&huart1, Flash_read_Aim, 7, 100);
	        	   break;



	          default:

	            break;
	        }

	     }
}

