/**************************************************
大然机器人-智能一体化关节函数库

适用平台：STM32平台
库版本号：v2.1
测试主控版本：STM32f407VE   
*************************************************/
#include "DrEmpower_can.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "stm32F4xx.h"
#include <stdio.h>
#include "can.h"
#include <rtthread.h>

#define ENABLE_INPUT_VALIDITY_CHECK 1   /* 输入合法性检查。为 0 时不编译空指针判断等检查，可减小程序体积、加快处理速度。 */

#define SERVO_MALLOC(size) rt_malloc(size) /* RT-Thread 动态内存分配 */
#define SERVO_FREE(ptr) rt_free(ptr)       /* RT-Thread 动态内存释放 */
#define SERVO_DELAY(n) rt_thread_mdelay(n) /* RT-Thread 毫秒延时 (让出 CPU) */
#define SERVO_SPRINTF sprintf           /* 使用这个函数需要在 Keil 里勾选 Use MicroLIB 或在 STM32CubeIDE 里勾选 Use float with printf from newlib-nano */

char command[64];       // 命令发送缓冲区
char DaTemp[4];
uint8_t i;
uint8_t rx_buffer[8];
uint16_t can_id = 0x00;
int8_t READ_FLAG = 0;  // 读取结果标志位
uint8_t set_angles_mode_1_flag = 0; // 标记上一条指令是否为 set_angles mode=1，如果是则为1，不是则为0

int8_t enable_replay_state = 1;  //如需要打开运动控制指令实时状态返回功能，请将该变量改为1，并将下面的MOTOR_NUM设置为总线上的最大关节ID号

//"""
//内部辅助函数，用户无需使用
//"""

// CAN发送函数
void send_command(uint8_t id_num, char cmd, unsigned char *data,uint8_t rt )
{
    short id_list = (id_num << 5) + cmd;
    Can_Send_Msg(id_list, 8, data);
    set_angles_mode_1_flag = 0;
}

void SERVO_DELAY_US(uint8_t tick)
{
    for(uint8_t t = 0; t<tick; t++);
}

//CAN接收函数 (使用 RT-Thread 信号量同步, 替代 busy-wait)
void receive_data(void)
{
    rt_err_t result;
    /* 阻塞等待 CAN RX 中断释放信号量, 超时 10ms */
    result = rt_sem_take(sem_can_rx_done, rt_tick_from_millisecond(10));
    if (result != RT_EOK)
    {
        /* 超时: 标记读取失败 */
        READ_FLAG = -1;
    }
    /* 成功: READ_FLAG 已在 ISR 回调中设为 1, 数据已在 rx_buffer 中 */
}


// 数据格式转换函数，decode是将二进制(bytes)转化成人看的懂得数据，encode反之
/*
float，short，unsigned short，int，unsigned int五种数据类型与byte类型的转换
五种数据对应名：0,1,2,3,4;
使用方法，在函数中调用时：
    首先要在调用前对结构体内的数据进行赋值
    value_data[3]：若将五种类型的数据转换为byte，则赋值
    byte_data[8]：若将byte转换为五种数据的类型，则赋值
    type_data[3]类型名赋值（必要）
    length：数据个数赋值（必要）

若将五种类型的数据转换为byte，调用format_data(data_struct data_list , char * str)，
    参数为：结构体指针，要做的操作（输入“encode”）
若将byte转换为五种数据的类型，调用format_data(data_struct data_list , char * str)，
    参数为：结构体指针，要做的操作（输入“decode”）

下面两个函数在上述两个函数中自动调用，不需要使用者主动调用
    byte2value()
    value2byte()

type类型：
type_data=0, 浮点数（float）,数据长度32位,符号‘f’;
type_data=1, 无符号短整数（unsigned short int）,数据长度16位,符号‘u16’;
type_data=2, 有符号短整数（short int）,数据长度16位,符号‘s16’;
type_data=3, 无符号短整数（unsigned int）,数据长度32位,符号‘u32’;
type_data=4, 有符号短整数（int）,数据长度32位,符号‘s32’;

*/
struct format_data_struct
{
    float value_data[4];//若将五种类型的数据转换为byte，则赋值
    unsigned char byte_data[8];//若将byte转换为五种数据的类型，则赋值
    int type_data[4];//类型名赋值（必要）
    int length;//数据个数赋值（必要）
}*data_struct,data_list;  //定义全局变量 data_list，用于进行CAN数据encode和decode过程中存储参数输入及结果输出

// note: the bit order is in accordance with intel little endian
static inline void uint16_to_data(uint16_t val, uint8_t *data)
{
    data[1] = (uint8_t)(val >> 8);
    data[0] = (uint8_t)(val);
}
static inline uint16_t data_to_uint16(uint8_t *data)
{
    uint16_t tmp_uint16;
    tmp_uint16 = (((uint32_t)data[1] << 8)  + ((uint32_t)data[0]));
    return tmp_uint16;
}
static inline void uint_to_data(uint32_t val, uint8_t *data)
{
    data[3] = (uint8_t)(val >> 24);
    data[2] = (uint8_t)(val >> 16);
    data[1] = (uint8_t)(val >> 8);
    data[0] = (uint8_t)(val);
}
static inline uint32_t data_to_uint(uint8_t *data)
{
    uint32_t tmp_uint;
    tmp_uint = (((uint32_t)data[3] << 24) + ((uint32_t)data[2] << 16) + ((uint32_t)data[1] << 8)  + ((uint32_t)data[0]));
    return tmp_uint;
}
static inline void int16_to_data(int16_t val, uint8_t *data)
{
    data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
}
static inline int16_t data_to_int16(uint8_t *data)
{
    int16_t tmp_int16;
    *(((uint8_t*)(&tmp_int16)) + 0) = data[0];
    *(((uint8_t*)(&tmp_int16)) + 1) = data[1];
    return tmp_int16;
}
static inline void int_to_data(int val, uint8_t *data)
{
    data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
    data[2] = *(((uint8_t*)(&val)) + 2);
    data[3] = *(((uint8_t*)(&val)) + 3);
}
static inline int data_to_int(uint8_t *data)
{
    int tmp_int;
    *(((uint8_t*)(&tmp_int)) + 0) = data[0];
    *(((uint8_t*)(&tmp_int)) + 1) = data[1];
    *(((uint8_t*)(&tmp_int)) + 2) = data[2];
    *(((uint8_t*)(&tmp_int)) + 3) = data[3];
    return tmp_int;
}
static inline void float_to_data(float val, uint8_t *data)
{
    data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
    data[2] = *(((uint8_t*)(&val)) + 2);
    data[3] = *(((uint8_t*)(&val)) + 3);
}
static inline float data_to_float(uint8_t *data)
{
    float tmp_float;
    *(((uint8_t*)(&tmp_float)) + 0) = data[0];
    *(((uint8_t*)(&tmp_float)) + 1) = data[1];
    *(((uint8_t*)(&tmp_float)) + 2) = data[2];
    *(((uint8_t*)(&tmp_float)) + 3) = data[3];
    return tmp_float;
}

//不需主动调用
void byte2value()
{
    int value_index = 0;
    int byte_index = 0;
    while (1)
    {
        switch(data_list.type_data[value_index])
        {
        case 0:
        {
            data_list.value_data[value_index] = (float)data_to_float(&data_list.byte_data[byte_index]);
            byte_index = 4+byte_index;
            value_index++;
        }
        break;
        case 1:
        {
            data_list.value_data[value_index] = (float)data_to_uint16(&data_list.byte_data[byte_index]);
            byte_index = 2+byte_index;
            value_index++;
        }
        break;
        case 2:
        {
            data_list.value_data[value_index] = (float)data_to_int16(&data_list.byte_data[byte_index]);
            byte_index = 2+byte_index;
            value_index++;
        }
        break;
        case 3:
        {
            data_list.value_data[value_index] = (float)data_to_uint(&data_list.byte_data[byte_index]);
            byte_index = 4+byte_index;
            value_index++;
        }
        break;
        case 4:
        {
            data_list.value_data[value_index] = (float)data_to_int(&data_list.byte_data[byte_index]);
            byte_index = 4+byte_index;
            value_index++;
        }
        break;
        default:
            value_index++;
            break;
        }
        if((byte_index>=8)||(value_index>=data_list.length))
        {
            return;
        }
    }
}
//不需主动调用
void value2byte()
{
    int byte_index=0;
    int value_index=0;
    while(1)
    {
        if (data_list.type_data[value_index]==0)
        {
            float_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index += 1;
        }
        switch(data_list.type_data[value_index])
        {
        case 0:
        {
            float_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index += 1;
        }
        break;
        case 1:
        {
            uint16_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 2;
            value_index += 1;
        }
        break;
        case 2:
        {
            int16_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 2;
            value_index += 1;
        }
        break;
        case 3:
        {
            uint_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index += 1;
        }
        break;
        case 4:
        {
            int_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index += 1;
        }
        break;
        default:
            value_index++;
            break;
        }
        if((byte_index >=8)||(value_index>=data_list.length))
        {
            return;
        }
    }
}

//参数结构体指针，要做的操作（五种type转byte输入“encode”，byte转五种type输入“decode”）
void format_data( float *value_data, int *type_data,int length, char * str)
{
    data_list.length=length;
    for (int i = 0; i < length; i++)
    {
        data_list.value_data[i]= value_data[i];
        data_list.type_data[i] = type_data[i];
    }
    if (strcmp(str,"encode")==0)
    {
        value2byte();
    }
    if (strcmp(str,"decode")==0)
    {
        for (int i = 0; i < 8; i++)
        {
            data_list.byte_data[i]=rx_buffer[i];
        }
        byte2value();
    }
}

/**
 * @brief 单个一体化关节角度预设函数。
 * 预设指定一体化关节编号的一体化关节的目标角度，之后需要用mt或mq指令启动转动。
 *
 * @param id_num 需要设置的一体化关节ID编号,如果不知道当前一体化关节ID，可以用0广播，如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 * @param angle 一体化关节角度（-360~360）*n，支持大角度转动
 * @param t mode=0,无作用，直接给0即可; mode=1, 运动时间（s）; mode =2, 前馈转速（r/min)
 * @param param mode=0,角度输入滤波带宽（<300），mode=1,启动和停止阶段加转速（(r/min)/s）; mode =2, 前馈力矩（Nm)
 * @param mode 角度控制模式选择，一体化关节支持三种角度控制模式，
 *             mode = 0: 多个一体化关节轨迹跟踪模式，用于一般绕轴运动，特别适合多个轨迹点输入，角度输入带宽参数需设置为指令发送频率的一半。
 *             mode = 1: 多个一体化关节梯形轨迹模式，此时speed用运动时间t（s）表示，param为目标加转速（(r/min)/s）。
 *             mode = 2: 前馈控制模式，这种模式下的t为前馈转速，param为前馈力矩。前馈控制在原有PID控制基础上加入转速和力矩前馈，提高系统的响应特性和减少静态误差。
 * @note 在mode=1,梯形轨迹模式中，speed和accel都需要大于0.如果speed=0会导致一体化关节报motor error 并退出闭环控制模式，所以在这种模式下如果speed=0,会被用0.01代替。
          另外如果这种模式下accel=0，一体化关节以最快转速运动到angle,speed参数不再其作用。
 */
void preset_angle(uint8_t id_num, float angle, float t, float param, int mode)
{
    float factor = 0.01;
    if (mode == 0)
    {
        float f_angle = angle;
        int s16_time = (int)(fabs(t) / factor);
        if (param > 300)
            param = 300;
        int  s16_width = (int)(fabs(param / factor));

        float value_data[3]= {f_angle, s16_time, s16_width};
        int type_data[3]= {0,2,2};

        format_data(value_data,type_data,3,"encode");

        send_command(id_num,0x0C,data_list.byte_data,0);
    }
    else if(mode == 1)
    {
        float f_angle = angle;
        int s16_time = (int)(fabs(t) / factor);
        int s16_accel = (int)((fabs(param)) / factor);

        float value_data[3]= {f_angle,s16_time,s16_accel};
        int type_data[3]= {0,2,2};

        format_data(value_data,type_data,3,"encode");
        send_command(id_num,0x0C,data_list.byte_data,0);
    }
    else if(mode == 2)
    {
        float f_angle = angle;
        int s16_speed_ff = (int)((t) / factor);
        int s16_torque_ff = (int)((param) / factor);

        float value_data[3]= {f_angle,s16_speed_ff,s16_torque_ff};
        int type_data[3]= {0,2,2};

        format_data(value_data,type_data,3,"encode");
        send_command(id_num,0x0C,data_list.byte_data,0);
    }
}
/**
 * @brief 单个一体化关节转速预设函数。
 * 预设指定一体化关节编号的一体化关节的目标转速，之后需要用mv指令启动转动。
 *
 * @param id_num 需要设置的一体化关节ID编号,如果不知道当前一体化关节ID，可以用0广播，如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 * @param speed 目标转速（r/min）
 * @param param mode=0, 前馈力矩（Nm); mode!=0,或目标加转速（(r/min)/s）
 * @param mode 控制模式选择
 *             mode=0, 转速前馈控制模式，一体化关节将目标转速直接设为speed
 *             mode!=0,转速爬升控制模式，一体化关节将按照目标加转速变化到speed。
 * @note 在转速爬升模式下，如果目标加转速设置为0，则一体化关节转速将保持当前值不变。
 */
void preset_speed(uint8_t id_num, float speed, float param, int mode)
{
    float factor = 0.01;
    float f_speed = speed;
    if (mode == 0)
    {
        int  s16_torque = (int)((param) / factor);
        if (f_speed == 0)
            s16_torque = 0;
        int s16_input_mode = (int)(1 / factor);

        float value_data[3]= {f_speed, s16_torque, s16_input_mode};
        int type_data[3]= {0,2,2};

        format_data(value_data,type_data,3,"encode");
    }
    else
    {
        int s16_ramp_rate = (int)((param) / factor);
        int s16_input_mode = (int)(2 / factor);

        float value_data[3]= {f_speed, s16_ramp_rate, s16_input_mode};
        int type_data[3]= {0,2,2};

        format_data(value_data,type_data,3,"encode");
    }
    send_command(id_num, 0x0C,data_list.byte_data, 0);
}

/**
 * @brief 单个一体化关节力矩预设函数。
 * 预设指定一体化关节编号的一体化关节目标力矩（Nm）
 *
 * @param id_num 需要设置的一体化关节ID编号,如果不知道当前一体化关节ID，可以用0广播，如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 * @param torque 一体化关节输出（Nm)
 * @param param mode=0,改参数无意义；mode!=0,力矩上升速率（Nm/s）
 * @param mode 控制模式选择
 *             mode=0, 力矩直接控制模式，一体化关节将目标力矩直接设为torque
 *             mode!=0,力矩爬升控制模式，一体化关节将按照力矩上升速率（Nm/s）变化到torque。
 * @note  如果一体化关节转速超过您设置的 speed_limit ，一体化关节输出的力矩将会减小。
    	  可以设置 dr.controller.config.speed_limit = False 来禁止力矩减小。
          另外在力矩爬升控制模式下，如果点击力矩上升速率为0，则点击力矩将在当前值保持不变。
 */
void preset_torque(uint8_t id_num, float torque, float param, int mode)
{

    float factor = 0.01;
    float f_torque = torque;
    int s16_ramp_rate, s16_input_mode;
    if (mode == 0)
    {
        s16_input_mode = (int)(1 / factor);
        s16_ramp_rate = 0;
    }
    else
    {
        s16_input_mode = (int)(6 / factor);
        s16_ramp_rate =  (int)((param) / factor);
    }

    float value_data[3]= {f_torque, s16_ramp_rate, s16_input_mode};
    int type_data[3]= {0,2,2};

    format_data(value_data,type_data,3,"encode");

    send_command(id_num,0x0C,data_list.byte_data,0);
}

// 功能函数，用户使用

// 运动控制功能

/**
 * @brief 单个一体化关节角度控制函数。
 * 控制指定 ID 编号的一体化关节按照指定的转速转动到指定的角度（绝对角度，相对于用户设定的零点角度）。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID，可以用 0 广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 * @param angle 一体化关节绝对角度（°），支持-360°~360°的倍数大角度转动
 * @param speed 一体化关节转速（r/min），具体含义由 mode 的取值决定：
 *              - mode=1：表示目标转速；
 *              - mode=0/2：表示前馈转速
 * @param param 运动参数，由 mode 取值决定：
 *              - mode=0：角度输入滤波带宽（需小于300）；
 *              - mode=1：启动和停止阶段角加转速（r/min/s）；
 *              - mode=2：前馈力矩（Nm）
 * @param mode 角度控制模式，一体化关节支持三种角度控制模式：
 *              - mode=0：轨迹跟踪模式，适合多个轨迹点输入后进行平滑控制，角度输入滤波带宽参数需设置为指令发送频率的一半；
 *              - mode=1：梯形轨迹模式，这种模式下可以指定运动过程中的目标转速（speed）和启停加转速（param）；
 *              - mode=2：前馈控制模式，这种模式下的 speed 和 param（torque）分别为前馈控制量。前馈控制在原有 PID 控制基础上加入转速和力矩前馈，提高系统的响应特性和减少静态误差
 * @note 在 mode=1 梯形轨迹模式中，speed 和 param（accel）都要大于 0；若 speed=0 会导致关节报 motor error 并退出闭环控制模式，因此该模式下 speed=0 时会自动替换为 0.01；若 param（accel）=0，关节将以最快速度运动到目标角度，speed 参数不再起作用。mode=0 时 speed 参数不起作用。
 */
void set_angle(uint8_t id_num, float angle, float speed, float param, int mode)
{
    float factor = 0.01;
    if (mode == 0)
    {
        float f_angle = angle;
        int s16_speed = (int)((abs(speed)) / factor);
        if (param > 300)
            param = 300;
        int s16_width = (int)(abs(param / factor));

        float value_data[3]= {f_angle,s16_speed,s16_width};
        int type_data[3]= {0,2,2};

        format_data(value_data,type_data,3,"encode");

        send_command(id_num,0x19,data_list.byte_data,0);
    }
    else if (mode == 1)
    {
        if ((speed > 0)&&(param > 0))
        {
            float f_angle = angle;
            int s16_speed = (int)((abs(speed)) / factor);
            int s16_accel = (int)((abs(param)) / factor);

            float value_data[3]= {f_angle,s16_speed,s16_accel};
            int type_data[3]= {0,2,2};

            format_data(value_data,type_data,3,"encode");

            send_command(id_num,0x1A,data_list.byte_data,0);
        }
    }
    else if (mode == 2)
    {
        float f_angle = angle;
        int s16_speed_ff = (int)((speed) / factor);
        int s16_torque_ff = (int)((param) / factor);

        float value_data[3]= {f_angle,s16_speed_ff,s16_torque_ff};
        int type_data[3]= {0,2,2};

        format_data(value_data,type_data,3,"encode");

        send_command(id_num, 0x1B, data_list.byte_data, 0);
    }
}

/**
 * @brief 多个一体化关节绝对角度控制函数。
 * 控制多个编号的一体化关节按照一定转速转动到指定的角度。
 *
 * @param id_list 一体化关节 ID 编号组成的列表。
 * @param angle_list 目标角度（°）组成的列表。
 * @param speed 指定转速（r/min），由 mode 取值决定：
 *              - mode=1：多个一体化关节中转速最大者的转速；
 *              - mode=0/2：前馈转速。
 * @param param 运动参数，由 mode 取值决定：
 *              - mode=0：角度输入滤波带宽（需小于300）；
 *              - mode=1：启动和停止阶段加转速（r/min/s）；
 *              - mode=2：前馈力矩（Nm）。
 * @param n 数组长度
 * @param mode 控制模式选择，一体化关节支持三种角度控制模式，由 mode 取值决定：
 *              - mode=0：多个一体化关节轨迹跟踪模式，适合多个轨迹点输入后进行平滑控制，角度输入带宽参数需设置为指令发送频率的一半；
 *              - mode=1：多个一体化关节梯形轨迹模式，此时 speed 为这些一体化关节的最快转速（r/min），param 为目标加转速（r/min/s）；
 *              - mode=2：前馈控制模式，这种模式下的 speed 和 param（torque）分别为前馈控制量。前馈控制在原有 PID 控制基础上加入转速和力矩前馈，提高系统的响应特性和减少静态误差。
 */
void set_angles(uint8_t *id_list, float *angle_list, float speed, float param, int mode, size_t n)
{

#if ENABLE_INPUT_VALIDITY_CHECK
    if (id_list == NULL || angle_list == NULL) return;
#endif
    static int last_count = 0;
    static uint8_t *last_id_list = NULL;
    static float *current_angle_list = NULL;

    uint8_t state = 0;
    for (size_t i = 0; i < n; i++)
    {
        if (last_id_list[i] != id_list[i])
        {
            state = 1;
            break;
        }
    }

    if (last_count != n || state || set_angles_mode_1_flag == 0)
    {
        last_count = n;
        SERVO_FREE(last_id_list);
        SERVO_FREE(current_angle_list);
        last_id_list = SERVO_MALLOC(n * sizeof(uint8_t));
        current_angle_list = SERVO_MALLOC(n * sizeof(float));
        memcpy(last_id_list, id_list, n * sizeof(uint8_t));
        for (size_t i = 0; i < n; i++)
            current_angle_list[i] = get_state(id_list[i]).angle;
    }
    if (mode == 0)
    {
        for (size_t i = 0; i < n; i++)
            preset_angle(id_list[i],angle_list[i],speed,param,mode);
        unsigned char order_num = 0x10;

        float value_data[3]= {order_num,0,0};
        int type_data[3]= {3,1,1};

        format_data(value_data,type_data,3,"encode");

        send_command(0,0x08,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
    }
    else if (mode == 1)
    {
        if ((speed < 0) ||( param < 0)) return;
        float delta_angle = 0;
        float t = 0;
        float fabs_temp;
        for (size_t i = 0; i < n; i++)
        {
            if (delta_angle < (fabs_temp = fabs(angle_list[i] - current_angle_list[i])))
                delta_angle = fabs_temp;
        }
        if (delta_angle <= (6 * speed * speed / fabs(param)))
                t = 2 * sqrt(delta_angle / (6 * fabs(param)));
            else
                t = speed / fabs(param) + delta_angle / (6 * speed);
        for (size_t i = 0; i < n; i++)
            preset_angle(id_list[i],angle_list[i],t,param,mode);
        preset_angle(255,0,t,param,mode); // 解决最大 id 号响应滞后问题
        unsigned char order_num = 0x11;

        float value_data[3]= {order_num,0,0};
        int type_data[3]= {3,1,1};
        format_data(value_data,type_data,3,"encode");
        send_command(0,0x08,data_list.byte_data,0); // 需要用标准帧（数据帧）进行发送，不能用远程帧
	set_angles_mode_1_flag = 1;
    }
    else if( mode == 2)
    {
        for (size_t i = 0; i < n; i++)
            preset_angle(id_list[i], angle_list[i],speed,param,mode);
        unsigned char order_num = 0x12;

        float value_data[3]= {order_num,0,0};
        int type_data[3]= {3,1,1};
        format_data(value_data,type_data,3,"encode");
        send_command(0,0x08,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
    }
    memcpy(current_angle_list, angle_list, n * sizeof(float)); 
}

/**
 * @brief 单个一体化关节相对角度控制函数。
 * 控制指定编号的一体化关节按照指定的转速相对转动指定的角度（相对角度，相对于发送该指令时的角度）。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID，可以用 0 广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 * @param angle 一体化关节相对角度（°）。
 * @param speed 指定转速（r/min），由 mode 取值决定：
 *              - mode=1：表示目标转速；
 *              - mode=0/2：表示前馈转速。
 * @param param 运动参数，由 mode 取值决定：
 *              - mode=0：角度输入滤波带宽（需小于300）；
 *              - mode=1：启动和停止阶段加转速（r/min/s）；
 *              - mode=2：前馈力矩（Nm）。
 * @param mode 角度控制模式，一体化关节支持三种角度控制模式：
 *              - mode=0：轨迹跟踪模式，适合多个轨迹点输入后进行平滑控制，角度输入滤波带宽参数需设置为指令发送频率的一半；
 *              - mode=1：梯形轨迹模式，这种模式下可以指定运动过程中的目标转速和启停加转速；
 *              - mode=2：前馈控制模式，这种模式下的 speed 和 param（torque）分别为前馈控制量，前馈控制在原有 PID 控制基础上加入转速和力矩前馈，提高系统的响应特性和减少静态误差。
 * @note 在 mode=1 梯形轨迹模式中，speed 和 param（accel）都要大于 0；mode=0 时 speed 不起作用。
 */
void step_angle(uint8_t id_num, float angle, float speed, float param, int mode)
{
    float factor = 0.01;
    if (mode == 0)
    {
        float f_angle = angle;
        int s16_speed = (int)((abs(speed)) / factor);
        if (param > 300)
            param = 300;
        int s16_width = (int)(abs(param / factor));

        float value_data[3]= {f_angle,s16_speed,s16_width};
        int type_data[3]= {0,2,2};
        format_data(value_data,type_data,3,"encode");
        send_command(id_num,0x0C,data_list.byte_data,0);

        float value_data_[3]= {0x10, 1, 0};
        int type_data_[3]= {3, 2, 1};
        format_data(value_data_, type_data_, 3, "encode");
        send_command(id_num,0x08,data_list.byte_data,0);
    }
    else if (mode == 1)
    {
        if ((speed > 0)&&(param > 0))
        {
            float f_angle = angle;
            int s16_speed = (int)((abs(speed)) / factor);
            int s16_accel = (int)((abs(param)) / factor);

            float value_data[3]= {f_angle,s16_speed,s16_accel};
            int type_data[3]= {0,2,2};
            format_data(value_data,type_data,3,"encode");
            send_command(id_num,0x0C,data_list.byte_data,0);

            float value_data_[3]= {0x11, 1, 0};
            int type_data_[3]= {3, 2, 1};
            format_data(value_data_, type_data_, 3, "encode");
            send_command(id_num,0x08,data_list.byte_data,0);
        }
    }
    else if (mode == 2)
    {
        float f_angle = angle;
        int s16_speed_ff = (int)((speed) / factor);
        int s16_torque_ff = (int)((param) / factor);

        float value_data[3]= {f_angle,s16_speed_ff,s16_torque_ff};
        int type_data[3]= {0,2,2};
        format_data(value_data,type_data,3,"encode");
        send_command(id_num, 0x0C, data_list.byte_data, 0);

        float value_data_[3]= {0x12, 1, 0};
        int type_data_[3]= {3, 2, 1};
        format_data(value_data_, type_data_, 3, "encode");
        send_command(id_num,0x08,data_list.byte_data,0);
    }
}

/**
 * @brief 多个一体化关节相对角度控制函数。
 * 控制多个一体化关节按照指定的时间相对转动给定角度。
 *
 * @param id_list 一体化关节 ID 编号组成的列表。
 * @param angle_list 相对目标角度组成的列表。
 * @param speed 指定转速（r/min），由 mode 取值决定：
 *              - mode=1：多个一体化关节中转速最大者的转速；
 *              - mode=0/2：前馈转速。
 * @param param 运动参数，由 mode 取值决定：
 *              - mode=0：角度输入滤波带宽（需小于300）；
 *              - mode=1：启动和停止阶段加转速（r/min/s）；
 *              - mode=2：前馈力矩（Nm）。
 * @param n 数组长度
 * @param mode 控制模式选择，一体化关节支持三种角度控制模式，由 mode 取值决定：
 *              - mode=0：多个一体化关节轨迹跟踪模式，适合多个轨迹点输入后进行平滑控制，角度输入带宽参数需设置为指令发送频率的一半；
 *              - mode=1：多个一体化关节梯形轨迹模式，此时 speed 为这些一体化关节的最快转速（r/min），param 为目标加转速（r/min/s）；
 *              - mode=2：前馈控制模式，这种模式下的 speed 和 param（torque）分别为前馈控制量。前馈控制在原有 PID 控制基础上加入转速和力矩前馈，提高系统的响应特性和减少静态误差。
 */
void step_angles(uint8_t *id_list, float *angle_list, float speed, float param, int mode, size_t n)
{
#if ENABLE_INPUT_VALIDITY_CHECK
    if (id_list == NULL || angle_list == NULL) return;
#endif
    if (mode == 0)
    {
        for (size_t i = 0; i < n; i++)
            preset_angle(id_list[i], angle_list[i], speed, param, mode);
        unsigned char order_num = 0x10;

        float value_data[3]= {order_num,0,0};
        int type_data[3]= {3,1,1};
        format_data(value_data,type_data,3,"encode");

        send_command(0,0x08,data_list.byte_data,0); //需要用标准帧（数据帧）进行发送，不能用远程帧
    }
    else if( mode == 1)
    {
        if (speed <= 0 || param <= 0) return;
        float delta_angle = 0;
        float t = 0;
        float fabs_temp;
        for (size_t i = 0; i < n; i++)
        {
            if(delta_angle < (fabs_temp = fabs(angle_list[i])))
                delta_angle = fabs_temp;
        }
        if(delta_angle <= (6 * speed * speed / fabs(param)))
            t = 2 * sqrt(delta_angle / (6 * fabs(param)));
        else
            t = speed / fabs(param) + delta_angle / (6 * speed);
        for (size_t i = 0; i < n; i++)
            preset_angle(id_list[i], angle_list[i], t, param, mode);
        preset_angle(255,0,t,param,mode); // 解决最大 id 号响应滞后问题
        unsigned char order_num = 0x11;

        float value_data[3]= {order_num,2,0};
        int type_data[3]= {3,1,1};
        format_data(value_data,type_data,3,"encode");
        send_command(0, 0x08, data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
//        send_command(0, 0x08, data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
    }
    else if (mode == 2)
    {
        for (size_t i = 0; i < n; i++)
            preset_angle(id_list[i], angle_list[i], speed, param, mode);
        unsigned char order_num = 0x12;

        float value_data[3]= {order_num,0,0};
        int type_data[3]= {3,1,1};
        format_data(value_data,type_data,3,"encode");
        send_command(0,0x08,data_list.byte_data,0);  //需要用标准帧（数据帧）进行发送，不能用远程帧
    }
}

/**
 * @brief 单个一体化关节角度自适应函数。
 * 控制指定 ID 编号的一体化关节按照限定的转速和力矩转动到指定的角度（绝对角度，相对于用户设定的零点角度）。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID，可以用 0 广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 * @param angle 一体化关节角度（°）。
 * @param speed 限定转速值（r/min）。
 * @param torque 限定力矩值（Nm)。
 * @note 当设置的转速相对于力矩过大，或力矩相对于转速过小时，关节无法在短时间内提供足够的加速度使转速降至 0，
 *       此时若关节未遇阻力，会出现在目标角度过冲现象，此为物理规律，暂时没有好的解决办法。
 */
void set_angle_adaptive(uint8_t id_num, float angle, float speed, float torque)
{
    float factor = 0.01;
    float f_angle = angle;
    int s16_speed = (int)((abs(speed)) / factor);
    int s16_torque = (int)(abs(torque / factor));

    float value_data[3]= {f_angle,s16_speed,s16_torque};
    int type_data[3]= {0,2,2};

    format_data(value_data,type_data,3,"encode");
    send_command(id_num,0x0B,data_list.byte_data,0);
}

/**
 * @brief 多个一体化关节角度自适应函数。
 * 控制多个一体化关节按照限定的转速和力矩转动到指定的角度（绝对角度，相对于用户设定的零点角度）。
 *
 * @param id_list 一体化关节 ID 编号组成的列表。
 * @param angle_list 一体化关节角度（°）组成的列表。
 * @param speed_list 限定转速值（r/min）组成的列表。
 * @param torque_list 限定力矩值（Nm)组成的列表。
 * @param n 数组长度
 * @note 当设置的转速相对于力矩过大，或力矩相对于转速过小时，关节无法在短时间内提供足够的加速度使转速降至 0，
 *       此时若关节未遇阻力，会出现在目标角度过冲现象，此为物理规律，暂时没有好的解决办法。
 */
void set_angles_adaptive(uint8_t id_list[], float angle_list[], float speed_list[], float torque_list[], size_t n)
{
    if (id_list == NULL || angle_list == NULL || speed_list == NULL || torque_list == NULL) return;
    for (size_t i = 0; i < n; i++)
        preset_angle(id_list[i], angle_list[i], fabs(speed_list[i]), fabs(torque_list[i]), 1);
    preset_angle(255, angle_list[i], fabs(speed_list[i]), fabs(torque_list[i]), 1); // 解决最大 id 号响应滞后问题
    unsigned char order_num = 0x11;
    float value_data[3]= {order_num,3,1};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(0,0x08,data_list.byte_data,0);
}

/**
 * @brief 检查并等待单个一体化关节在梯形轨迹模式下是否运动到位函数。
 * 检查并等待单个一体化关节是否转动到指定角度。
 * 该函数支持以下三种角度控制模式：
 *                          梯形轨迹模式，mode=1
 *                          前馈模式，mode=2
 *                          自适应模式下，set_angle_adaptive()
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID，可以用 0 广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 */
void position_done(uint8_t id_num)
{
    int traj_done = 0;
    while(traj_done == 0 || READ_FLAG == -1)
    {
        traj_done = read_property(id_num, 32002,3);
    }
}

/**
 * @brief 检查并等待多个多个一体化关节在梯形轨迹模式下是否运动到位函数。
 * 检查并等待多个一体化关节是否全部转动到指定角度。
 * 该函数支持以下三种角度控制模式：
 *                          梯形轨迹模式，mode=1
 *                          前馈模式，mode=2
 *                          自适应模式下，set_angle_adaptive()
 *
 * @param id_list 一体化关节 ID 编号组成的列表。
 * @param n 数组长度
 */
void positions_done(uint8_t *id_list,size_t n)
{
    for (size_t i = 0; i < n; i++)
    {
        position_done(id_list[i]);
    }
}

/**
 * @brief 单个一体化关节转速控制函数。
 * 控制指定 ID 编号的一体化关节按照指定的转速连续整周转动（转动到关节支持的极限角度后自动停止）。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID，可以用 0 广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 * @param speed 目标转速（r/min）。
 * @param param 运动参数，由 mode 取值决定：
 *              - mode=0：前馈力矩（Nm)；
 *              - mode!=0：目标加转速（r/min/s）。
 * @param mode 控制模式选择，由 mode 取值决定：
 *              - mode=0：转速直接控制模式，将一体化关节目标转速直接设为 speed；
 *              - mode!=0：匀加速控制模式，一体化关节将按照目标角加速变化到 speed。
 * @note 在 mode!=0，即匀加速模式下，如果目标角加速度设置为 0，则一体化关节转速将保持当前值不变。
 */
void set_speed(uint8_t id_num, float speed, float param, int mode)
{
    float factor = 0.01;
    float f_speed = speed;
    if (mode == 0)
    {
        int s16_torque = (int)((param) / factor);
        if( f_speed == 0)
            s16_torque = 0;
        unsigned short u16_input_mode = 1;

        float value_data[3]= {f_speed,s16_torque,u16_input_mode};
        int type_data[3]= {0,2,1};
        format_data(value_data,type_data,3,"encode");
    }
    else
    {
        int s16_ramp_rate = (int)((param) / factor);
        unsigned short u16_input_mode = 2;

        float value_data[3]= {f_speed,s16_ramp_rate,u16_input_mode};
        int type_data[3]= {0,2,1};
        format_data(value_data,type_data,3,"encode");
    }
    send_command(id_num,0x1c,data_list.byte_data,0);
}

/**
 * @brief 多个一体化关节转速控制函数。
 * 控制多个一体化关节按照指定的转速连续整周转动（转动到关节支持的极限角度后自动停止）。
 *
 * @param id_list 一体化关节 ID 编号组成的列表。
 * @param speed_list 一体化关节目标转速（r/min）组成的列表。
 * @param param 运动参数，由 mode 取值决定：
 *              - mode=0：前馈力矩（Nm)；
 *              - mode!=0：目标加转速（r/min/s）。
 * @param mode 控制模式选择，由 mode 取值决定：
 *              - mode=0：转速直接控制模式，将一体化关节目标转速直接设为 speed；
 *              - mode!=0：匀加速控制模式，一体化关节将按照目标角加速变化到 speed。
 * @param n 数组长度
 * @note 在 mode!=0，即匀加速模式下，如果目标角加速度设置为 0，则一体化关节转速将保持当前值不变。
 */
void set_speeds(uint8_t *id_list, float *speed_list, float param, float mode, size_t n)
{
    if (id_list == NULL || speed_list == NULL) return;
    for (size_t i = 0; i < n; i++)
        preset_speed(id_list[i], speed_list[i], param, mode);
    preset_speed(255, speed_list[i], param, mode); // 解决最大 id 号响应滞后问题
    unsigned char order_num = 0x13;
    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(0, 0x08, data_list.byte_data, 0); //需要用标准帧（数据帧）进行发送，不能用远程帧
}

/**
 * @brief 单个一体化关节力矩（电流）闭环控制函数。
 * 控制指定 ID 编号的一体化关节输出指定的力矩（Nm），若阻力不足以抵抗该力矩，则关节会持续转动（转动到关节支持的极限角度后自动停止）。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID，可以用 0 广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 * @param torque 目标力矩（Nm）。
 * @param param 运动参数，由 mode 取值决定：
 *              - mode=0：参数不起作用；
 *              - mode!=0：力矩在单位时间内的增量（Nm/s）。
 * @param mode 控制模式选择，由 mode 取值决定：
 *              - mode=0：力矩直接控制模式，将一体化关节目标力矩直接设为 torque；
 *              - mode!=0：力矩匀速增加模式，一体化关节将按照指定的单位时间内的增量匀速变化到 torque。
 * @note 在 mode!=0，即力矩匀速增加模式下，如果目标单位时间增量设置为 0，则一体化关节输出力矩将保持当前值不变。
 */
void set_torque(uint8_t id_num, float torque, float param, int mode)
{
    float factor = 0.01;
    int u16_input_mode,s16_ramp_rate;
    float f_torque = torque;
    if (mode == 0)
    {
        u16_input_mode = 1;
        s16_ramp_rate = 0;
    }
    else
    {
        u16_input_mode = 6;
        s16_ramp_rate = (int)((param) / factor);
    }
    float value_data[3]= {f_torque, s16_ramp_rate,u16_input_mode};
    int type_data[3]= {0,2,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num,0x1d,data_list.byte_data,0);
}

/**
 * @brief 多个一体化关节力矩控制函数。
 * 控制多个一体化关节输出指定的力矩，若阻力不足以抵抗该力矩，则关节会持续转动（转动到关节支持的极限角度后自动停止）。
 *
 * @param id_list 一体化关节 ID 编号组成的列表。
 * @param torque_list 一体化关节目标力矩（Nm）组成的列表。
 * @param param 运动参数，由 mode 取值决定：
 *              - mode=0：该参数不起作用；
 *              - mode!=0：力矩在单位时间内的增量（Nm/s）。
 * @param mode 控制模式选择，由 mode 取值决定：
 *              - mode=0：力矩直接控制模式，将一体化关节目标力矩直接设为 torque；
 *              - mode!=0：力矩匀速增加模式，一体化关节将按照指定的单位时间内的增量匀速变化到 torque。
 * @param n 数组长度
 * @note 在 mode!=0，即力矩匀速增加模式下，如果目标单位时间增量设置为 0，则一体化关节输出力矩将保持当前值不变。
 */
void set_torques(uint8_t *id_list, float *torque_list, float param, int mode, size_t n)
{
    if (id_list == NULL || torque_list == NULL) return;
    for (size_t i = 0; i < n; i++)
        preset_torque(id_list[i], torque_list[i], param, mode);
    preset_torque(255, torque_list[i], param, mode); // 解决最大 id 号响应滞后问题
    unsigned char order_num = 0x14;
    float value_data[3]= {order_num};
    int type_data[3]= {3};
    format_data(value_data,type_data,1,"encode");
    send_command(0, 0x08, data_list.byte_data, 0);  //需要用标准帧（数据帧）进行发送，不能用远程帧
}

/**
 * @brief 单个一体化关节阻抗控制函数。
 * 对指定 ID 编号的一体化关节进行阻抗控制。该函数执行结束后关节会停在目标角度 angle，并对外表现出一定柔性。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID，可以用 0 广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 * @param angle 一体化关节目标角度（°）。
 * @param speed 一体化关节目标转速（r/min）。
 * @param tff 前馈力矩（Nm)。
 * @param kp 角度刚度系数（Nm/°），需大于 0。
 * @param kd 转速阻尼系数（Nm/(r/min)），需大于 0。
 * @param mode 模式选择，等于 1 则以角度为控制目标，等于 0 则以力矩为控制目标。
 * @note 该函数直接控制关节输出力矩，其目标输出力矩计算公式如下：
 *       torque = kp*( angle – angle_) + tff + kd*(speed – speed_)
 *       其中 angle_ 和 speed_ 分别为输出轴当前实际角度（°）和当前实际转速（r/min），kp 和 kd 为刚度系数和阻尼系数。
 */
void impedance_control(uint8_t id_num, float pos, float vel, float tff, float kp, float kd, int mode)
{
    float factor = 0.01;
    float angle_set = 0;
    kp = fabs(kp);
    kd = fabs(kd);
    if (kp > 20){
    	kp = 20; // 限制系数，否则带载时容易震动
    }
    if (kd > 20){
    	kd = 20;
    }
    if (mode == 1){
    	if (kp != 0){
    		angle_set = (- kd * vel - tff) / kp + pos;
    	}
    	else{
//    		printf("机器人中关节不允许不间断连续旋转 \n");
    		return;
    	}
    }
    else{
    	angle_set = pos;
    }
    preset_angle(id_num,angle_set,vel, tff, 2);
    unsigned char order_num = 0x15;
    float value_data[3]= {order_num,(int)(kp / factor),(int)(kd / factor)};
    int type_data[3]= {3,2,2};
    format_data(value_data,type_data,3,"encode");
    send_command(0, 0x08,data_list.byte_data,0);//需要用标准帧（数据帧）进行发送，不能用远程帧
}

/**
 * @brief 多个一体化关节阻抗控制函数。
 * 对多个一体化关节进行阻抗控制。该函数执行结束后被指定的各个关节会停在目标角度，并对外表现出一定柔性。
 *
 * @param id_list 一体化关节 ID 编号组成的列表。
 * @param angle_list 一体化关节目标角度组成的列表（°）。
 * @param speed_list 一体化关节目标转速组成的列表（r/min）。
 * @param tff_list 前馈力矩组成的列表（Nm）。
 * @param kp_list 角度刚度系数组成的列表（Nm/°），每个元素均需大于 0。
 * @param kd_list 转速阻尼系数组成的列表（Nm/(r/min)），每个元素均需大于 0。
 * @param mode 模式选择，等于 1 则以角度为控制目标，等于 0 则以力矩为控制目标。
 * @param n 数组长度
 * @note 该函数直接控制关节输出力矩，其目标输出力矩计算公式如下：
 *       torque = kp_list[i] * (angle_list[i] – angle_[i]) + tff_list[i] + kd_list[i] * (speed_list[i] – speed_[i])
 *       其中 angle_[i] 和 speed_[i] 分别为对应关节输出轴当前实际角度（°）和当前实际转速（r/min），
 *       kp_list[i] 和 kd_list[i] 为刚度系数和阻尼系数。
 */
void impedance_control_multi(uint8_t id_list[], float angle_list[], float speed_list[], float tff_list[], float kp_list[], float kd_list[], int mode, size_t n)
{
    if (id_list == NULL || angle_list == NULL || speed_list == NULL || tff_list == NULL || kp_list == NULL || kd_list == NULL) return;
    float factor = 0.001;
    float angle_set_list[n];
    for (size_t i = 0; i < n; i++)
    {
    	kp_list[i] = fabs(kp_list[i]);
    	kd_list[i] = fabs(kd_list[i]);
        if (kp_list[i] > 0){ kp_list[i] = 10;}
        if (kd_list[i] > 0){ kd_list[i] = 10;}
        if (mode == 1){
        	if (kp_list[i] != 0){
        		angle_set_list[i] = (- kd_list[i] * speed_list[i] - tff_list[i]) / kp_list[i] + angle_list[i];
        	}
        	else{
//        		printf("机器人中关节不允许不间断连续旋转 \n");
        		return;
        	}
        }
        else{
        	angle_set_list[i] = angle_list[i];
        }
        preset_angle(id_list[i],angle_set_list[i],speed_list[i], tff_list[i], 2);
        unsigned char order_num = 0x16;
        float value_data[3]= {order_num,(int)abs(kp_list[i] / factor),(int)abs(kd_list[i] / factor)};
        int type_data[3]= {3,2,2};
        format_data(value_data,type_data,3,"encode");
        send_command(id_list[i], 0x08,data_list.byte_data,0);//需要用标准帧（数据帧）进行发送，不能用远程帧
        send_command(255,0x06,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
        rt_thread_mdelay(1);  /* RT-Thread 延时, 让出 CPU */
    }

    unsigned char order_num = 0x17;
    float value_data_[3]= {order_num,1,1};
    int type_data_[3]= {3,0,0};
    format_data(value_data_,type_data_,3,"encode");
    send_command(0, 0x08,data_list.byte_data,0);//需要用标准帧（数据帧）进行发送，不能用远程帧
    send_command(255,0x06,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
}

/**
 * @brief 单个一体化关节运动跟随与助力函数。
 * 指定 ID 编号的一体化关节进行运动跟随与助力。
 * 当关节在停止状态下检测到角度差 angle_err 和转速差 speed_err 时，向目标角度 angle 方向提供力矩大小为 torque 的助力，
 * 并在到达 angle 后停止并保持位置。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID，可以用 0 广播，此时总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 * @param angle 目标角度（°），即助力目标角度，该值减去关节当前角度即为助力行程，目标角度范围为 -300~300°。
 * @param speed 限定转速（r/min），即助力的限定转速，防止助力力矩引起的持续加速导致转速过快。
 * @param angle_err 角度差值（°），表示运动跟随与助力的角度灵敏度。
 * @param speed_err 转速差值（r/min），表示运动跟随与助力的转速灵敏度。
 * @param torque 助力力矩（Nm)。
 * @note
 *       a、当助力与外部驱动力之和大于阻力时，关节会持续转动；
 *       b、当助力与外部驱动力之和小于阻力时，关节开始减速，当转速小于 2 倍转速差值 speed_err 时，关节停止输出助力；
 *       c、一般情况下，该功能用于人工助力，强烈建议用户将助力力矩设置在人力所能及的范围内（即人力可使关节停止转动）；
 *       d、若必须设置超出人力的力矩，则必须在合理位置设置牢固的机械限位，以避免超出运动范围给人或物体带来损伤。
 */
void motion_aid(uint8_t id_num, float angle, float speed, float angle_err, float speed_err, float torque)
{
    float factor = 0.01;
    if (angle < -300 || angle > 300) return;
    float value_data[4]= {(int)(angle/factor), (int)(angle_err/factor), (int)(speed_err/factor), (int)(torque/factor)};
    int type_data[4]= {2, 1, 1, 2};
    format_data(value_data,type_data,4,"encode");
    send_command(id_num,0x0D,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
    set_speed_adaptive(id_num,speed);
    set_speed_adaptive(id_num,speed);
}

/**
 * @brief 多个一体化关节运动跟随与助力函数。
 * 指定多个一体化关节进行运动跟随与助力。
 * 当关节在停止状态下检测到角度差 angle_er_list[i] 和转速差 speed_err_list[i] 时，向目标角度 angle_list[i] 方向提供力矩大小为
 * torque_list[i] 的助力，并在到达 angle_list[i] 后停止并保持位置。
 *
 * @param id_list 一体化关节 ID 编号组成的列表。
 * @param angle_list 目标角度组成的列表（°），即助力目标角度，该值减去关节当前角度即为助力行程。
 * @param speed_list 限定转速组成的列表（r/min），即助力的限定转速，防止助力力矩引起的持续加速导致转速过快。
 * @param angle_err_list 角度差值（°）组成的列表，表示运动跟随与助力的角度灵敏度。
 * @param speed_err_list 转速差值（r/min）组成的列表，表示运动跟随与助力的转速灵敏度。
 * @param torque_list 助力力矩（Nm)组成的列表。
 * @param n 数组长度
 * @note
 *       a、当助力与外部驱动力之和大于阻力时，关节会持续转动；
 *       b、当助力与外部驱动力之和小于阻力时，关节开始减速，当转速小于 2 倍转速差值 speed_err_list[i] 时，关节停止输出助力；
 *       c、一般情况下，该功能用于人工助力，强烈建议用户将助力力矩设置在人力所能及的范围内（即人力可使关节停止转动）；
 *       d、若必须设置超出人力的力矩，则必须在合理位置设置牢固的机械限位，以避免超出运动范围给人或物体带来损伤。
 */
void motion_aid_multi(uint8_t id_list[], float angle_list[], float speed_list[], float angle_err_list[], float speed_err_list[], float torque_list[], size_t n)
{
    if (id_list == NULL || angle_list == NULL || speed_list == NULL || angle_err_list == NULL || speed_err_list == NULL || torque_list == NULL) return;
    float factor = 0.01;
    for (size_t i = 0; i < n; i++)
    {
        if (angle_list[i] < -300 || angle_list[i] > 300) return;
    }
    for (size_t i = 0; i < n; i++)
    {
        float value_data[4]= {(int)(angle_list[i]/factor), (int)(angle_err_list[i]/factor), (int)(speed_err_list[i]/factor), (int)(torque_list[i]/factor)};
        int type_data[4]= {2, 1, 1, 2};
        format_data(value_data,type_data,4,"encode");
        send_command(id_list[i],0x06,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
        send_command(255,0x06,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
    }
    unsigned char order_num = 0x11;
    float value_data_[3]= {order_num,4,1};
    int type_data_[3]= {3,1,0};
    format_data(value_data_,type_data_,3,"encode");
    send_command(0, 0x08,data_list.byte_data,0);//需要用标准帧（数据帧）进行发送，不能用远程帧
    SERVO_DELAY(100); // 延时 0.1s
    for (size_t i = 0; i < n; i++)
    {
        set_speed_adaptive(id_list[i], speed_list[i]);
    }
}

/**
 * @brief 设置一体化关节自适应转速函数。
 * 设置一体化关节转速限制 speed_adaptive（r/min），此后关节自适应转速绝对值不超过 speed_adaptive。
 * 注意：
 *     1. 该函数执行完转速限制在本次开机运行期间有效，关机或重启后将失效。
 *     2. 若想关机重启前取消该限制，只需设置一个非常大的数值，比如令 speed_adaptive = 100000。
 *     3. 本函数需在运动指令之后运行。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0 广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 * @param speed_adaptive 自适应转速限制（r/min）（必须大于 0）。
 */
void set_speed_adaptive(uint8_t id_num, float speed_adaptive)
{
    if (speed_adaptive <=0) return;
    preset_angle(id_num, fabs(speed_adaptive), 0, 0, 1);
    preset_angle(id_num, fabs(speed_adaptive), 0, 0, 1);
    unsigned char order_num = 0x20;
    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num,0x08,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
    send_command(id_num,0x08,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
}

/**
 * @brief 设置一体化关节自适应力矩函数。
 * 设置一体化关节力矩限制 torque_adaptive（Nm），此后关节自适应力矩绝对值不超过 torque_adaptive。
 * 注意：
 *     1. 该函数执行完力矩限制在本次开机运行期间有效，关机或重启后将失效。
 *     2. 若想关机重启前取消该限制，只需设置一个非常大的数值，比如令 torque_adaptive = 100000。
 *     3. 本函数需在运动指令之后运行。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0 广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 * @param torque_adaptive 力矩限制（Nm）（必须大于 0）。
 */
void set_torque_adaptive(uint8_t id_num, float torque_adaptive)
{
    if (torque_adaptive <=0) return;
    preset_angle(id_num, fabs(torque_adaptive), 0, 0, 1);
    preset_angle(id_num, fabs(torque_adaptive), 0, 0, 1);
    unsigned char order_num = 0x21;
    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num,0x08,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
    send_command(id_num,0x08,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
}

/**
 * @brief 设置一体化关节控制环 PID 函数。
 * 设置一体化关节控制环的位置增益 P、转速增益 D、积分增益 I，以便实现调整关节控制性能的目的。
 * 该函数执行完 PID 的值在本次开机运行期间有效，关机或重启后将失效。
 * 如用户决定永久使用某组 PID 则可在使用该函数设置 PID 后，请紧接着使用 save_config 函数。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0 广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 * @param P 位置增益（必须大于 0）。
 * @param I 积分增益（必须大于 0）。
 * @param D 转速增益（必须大于 0）。
 */
void set_pid(uint8_t id_num, float P, float I, float D)
{
    if (P <= 0 || I <= 0 || D <= 0) return;
    write_property(id_num, 32102, 0, P);
    write_property(id_num, 32104, 0, I);
    write_property(id_num, 32103, 0, D);
}

/**
 * @brief 急停函数
 * 控制一体化关节紧急停止。如果想控制多个关节同时同时急停，则可使 id_num=0。
 *
 * @param id_num 一体化关节 ID 编号。如果不知道当前一体化关节 ID，可以用 0 广播，此时如果总线上有多个一体化关节，则多个多个一体化关节都会都会执行该操作。
 */
void estop(uint8_t id_num)
{
    unsigned char order_num = 0x06;
    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num,0x08,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
}
// 参数设置功能

/**
 * @brief 设置一体化关节 ID 编号。
 * 改变一体化关节 ID 编号，一次设定（关机后依然保存）。
 * 注：
 *     (1) 使用一体化关节前请先将其设置独有的ID号，以免在总线中出现相同ID号的多个关节，造成通信混乱。
 *     (2) 该函数最好在正式使用关节之前使用，预先设置ID以便确定控制目标。
 *     (3) 建议在无负载的情况下执行此命令，否则可能造成关节短暂卸载。
 *
 * @param id_num 需要重新设置 ID 编号的一体化关节的 ID 编号，如果不知道不知道当前一体化关节 ID 编号，可以用 0 广播。
 *               但是这时总线上只能连一个一体化关节，否则多个一体化关节会被设置成相同编号。
 * @param new_id 新一体化关节编号，一体化关节 ID 号范围为 1~64 内整数。
 */
void set_id(uint8_t id_num, int new_id)
{
    write_property(id_num,31001,3,new_id);
    save_config(new_id);
}

/**
 * @brief 设置一体化关节 CAN 波特率。
 * 设置 CAN 波特率（关机后依然保存）。
 * 注：建议在无负载的情况下执行此命令，否则可能造成关节短暂卸载。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID，可以用 0 广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 * @param baud_rate CAN 波特率，支持 125k、250k、500k、1M 中任意一种，修改成功后需手动将上位机 CAN 波特率也修改为相同值。
 */
void set_can_baud_rate(uint8_t id_num, int baud_rate)
{
    write_property(id_num,21001,3,baud_rate);
    save_config(id_num);
}

/**
 * @brief 设置一体化关节模式。
 * 设置一体化关节进入不同的控制模式。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID，可以用 0 广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 * @param mode 一体化关节模式编号，
 *             mode=1：待机模式，一体化关节卸载
 *             mode=2：闭环控制模式，运动控制函数必须在闭环控制模式下才能进行控制。（一体化关节上电后的默认模式）
 */
void set_mode(uint8_t id_num, int mode)
{
    if (mode == 1)
    {
        write_property(id_num,30003,3,1);
    }
    else if (mode == 2)
    {
    	rt_thread_mdelay(100);  /* RT-Thread 延时, 让出 CPU */
        write_property(id_num,30003,3, 2);
    }
}

/**
 * @brief 设置一体化关节零点角度函数。
 * 设置当前角度为一体化关节输出轴零点，设置完后当前角度为 0 度，重启后保存。
 * 注：
 *     (1) 最好在修改其他参数前使用该函数，因该函数中包含 save_config() 函数，会将其他参数一并保存。
 *     (2) 建议在无负载的情况下执行此命令，否则可能造成关节短暂卸载。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID，可以用 0 广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 */
void  set_zero_position(uint8_t id_num)
{

    unsigned char order_num = 0x05;

    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num, 0x08, data_list.byte_data, 0); //需要用标准帧（数据帧）进行发送，不能用远程帧
    save_config(id_num); // 保存后掉电不丢失
}

/**
 * @brief 设置一体化关节零点角度函数，当次启动有效，重启后失效。
 * 设置当前角度为一体化关节输出轴零点，设置完后当前角度为 0 度，重启后丢失。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID，可以用 0 广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 */
void set_zero_position_temp(uint8_t id_num)
{
    unsigned char order_num = 0x23;

    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num, 0x08, data_list.byte_data, 0); //需要用标准帧（数据帧）进行发送，不能用远程帧
}

/**
 * @brief 设置一体化关节运行过程中的极限角度，设置成功后一体化关节的可控制的转动角度将限定在[angle_min, angle_max]范围内。关机重启后失效。
 * 注：
 *     a、使用该函数时输出轴角度必须在[angle_min, angle_max]范围内，否则将设置失败；
 *     b、该功能只在本次开机运行过程中有效，对应地将在关节重启后失效；
 *     c、关节本身还有一个极限角度属性，默认生效，范围为[-180.5°, 180.5°]，
 *        该属性不受该函数影响，且每次开机重启均有效，如需重新设置或取消该属性，请使用
 *        set_angle_range_config() 和 disable_angle_range_config()，详见对应函数说明。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0 广播，如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 * @param angle_min 最小限位角度（°）。
 * @param angle_max 最大限位角度（°）。
 */
int8_t set_angle_range(uint8_t id_num, float angle_min, float angle_max)
{

    float pos_vel = get_state(id_num).angle;
    if (READ_FLAG == 1)
    {
        if (pos_vel < angle_min || pos_vel > angle_max || READ_FLAG != 1) return -1;
        write_property(id_num, 38004, 0, angle_min);
        write_property(id_num, 38005, 0, angle_max);
        write_property(id_num, 38006, 3, 1);
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief 取消本次运行期间一体化关节运行过程中的角度限位。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0 广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 */
int8_t disable_angle_range(uint8_t id_num)
{
    write_property(id_num, 38006, 3, 0);
    if (read_property(id_num, 38006, 3) == 0)
        return 1;
    else
        return 0;
}

/**
 * @brief 设置一体化关节极限角度属性，设置成功后一体化关节的可控制的转动角度将限定在[angle_min, angle_max]范围内，每次开机重启均默认有效。
 * 注：
 *     a、使用该函数时输出轴角度必须在[angle_min, angle_max]范围内，否则将设置失败；
 *     b、限位范围设置成功后，再执行 save_config() 函数，每次开机重启后均有效；
 *     c、如需取消该属性影响，请使用 disable_angle_range_config() 函数将该属性关闭，则本次开机该属性不起作用；
 *        若随后使用 save_config() 则该属性将永久失去，如需找回该属性，则再次使用本函数即可。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0 广播，如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 * @param angle_min 最小限位角度（°）。
 * @param angle_max 最大限位角度（°）。
 */
int8_t set_angle_range_config(uint8_t id_num, float angle_min, float angle_max)
{
    float angle = get_angle(id_num);
    if (READ_FLAG == 1)
    {
        if (angle < angle_min || angle > angle_max || READ_FLAG != 1) return -1;
        write_property(id_num, 31202, 0, angle_min);
        write_property(id_num, 31203, 0, angle_max);
        write_property(id_num, 31201, 3, 1);
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief 取消一体化关节角度限位属性。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0 广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 */
int8_t disable_angle_range_config(uint8_t id_num)
{
    write_property(id_num, 31201, 3, 0);
    if (read_property(id_num, 31201, 3) == 0)
        return 1;
    else
        return 0;
}

/**
 * @brief 设置一体化关节转速限制函数。
 * 设置一体化关节转速限制 speed_limit （r/min），此后关节转速绝对值不超过 speed_limit。
 * 注意：
 *     1. 该函数执行完转速限制在本次开机运行期间有效，关机或重启后将失效。
 *     2. 若想关机重启前取消该限制，只需设置一个非常大的数值，比如令 speed_limit = 100000。
 *     3. 如用户决定永久保持转速限制 speed_limit，请紧接着使用 save_config 函数（慎用）。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0 广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 * @param speed_limit 转速限制（r/min）（必须大于 0）。
 */
void set_speed_limit(uint8_t id_num, float speed_limit)
{
    if (speed_limit <= 0) return;
    preset_angle(id_num, fabs(speed_limit), 0, 0, 1);
    unsigned char order_num = 0x18;

    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num, 0x08, data_list.byte_data, 0); //需要用标准帧（数据帧）进行发送，不能用远程帧
}

/**
 * @brief 设置一体化关节力矩限制函数。
 * 设置一体化关节力矩限制 torque_limit（Nm），此后关节力矩绝对值不超过 torque_limit。
 * 注意：
 *     1. 该函数执行完力矩限制在本次开机运行期间有效，关机或重启后将失效。
 *     2. 若想关机重启前取消该限制，只需设置一个非常大的数值，比如令 torque_limit = 100000。
 *     3. 如用户决定永久保持力矩限制 torque_limit，请紧接着使用 save_config 函数（慎用）。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0 广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 * @param torque_limit 力矩限制（Nm）（必须大于 0）。
 */
void set_torque_limit(uint8_t id_num, float torque_limit)
{
    if (torque_limit <= 0) return;
    preset_angle(id_num, fabs(torque_limit), 0, 0, 1);
    unsigned char order_num = 0x19;

    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num, 0x08, data_list.byte_data, 0); //需要用标准帧（数据帧）进行发送，不能用远程帧
}

/**
 * @brief 修改一体化关节属性参数。
 * 修改一体化关节属性参数，这里的属性参数为一体化关节控制参数，存放于 interface_enums.py 文件。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0 广播，此时如果总线上有多个一体化关节，则多个一体化关节都会会执行该操作。
 * @param property 需要设置的属性参数名称，例如 "dr.voltage"，"dr.config.can_id"等，
 *                 具体参数名称见 parameter_interface.py 文件里 property_address 字典里的键值。
 * @param value 对应参数的目标值。
 */
void write_property(uint8_t id_num,unsigned short param_address,int8_t param_type,float value)
{
    float value_data[3]= {param_address,param_type,value};
    int type_data[3]= {1,1,param_type};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num, 0x1F,data_list.byte_data,0);  //需要用标准帧（数据帧）进行发送，不能用远程帧
}

/**
 * @brief 读取一体化关节 ID 编号。
 * 读取一体化关节 ID 编号。注意使用该函数时总线上只能接 1 个一体化关节。
 *
 * @return id：关节 ID 编号。
 * @note 函数无参数。
 */
uint8_t get_id(uint8_t id_num)
{
    return read_property(id_num,31001,3);
}

/**
 * @brief 读取一体化关节当前角度（°）。
 * 读取一体化关节当前角度，单位为度（°）。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0 广播。
 *               但此时如果总线上有多个一体化关节，会造成总线通信干扰，不可使用 0 广播。
 * @return angle：当前关节角度（°）。
 */
float get_angle(uint8_t id_num)
{
    return read_property(id_num,38001,0);
}

/**
 * @brief 读取一体化关节当前转速（r/min）。
 * 读取一体化关节当前转速，单位为转每分钟（r/min）。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0 广播。
 *               但此时如果总线上有多个一体化关节，会造成总线通信干扰，不可使用 0 广播。
 * @return speed：关节当前转速（r/min）。
 */
float get_speed(uint8_t id_num)
{
    return read_property(id_num,38002,0);
}

/**
 * @brief 读取一体化关节当前力矩（Nm）。
 * 读取一体化关节当前输出力矩，单位为牛米（Nm）。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0 广播。
 *               但此时如果总线上有多个一体化关节，会造成总线通信干扰，不可使用 0 广播。
 * @return torque：关节当前输出力矩（Nm）。
 */
float get_torque(uint8_t id_num)
{
    return read_property(id_num,38003,0);
}

/**
 * @brief 开启角度、转速、力矩实时反馈函数。
 * 开启角度、转速、力矩实时反馈。
 * 特别提醒：运行任何其他回读参数的函数前，须先运行 disable_angle_speed_torque_state() 关闭实时反馈。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0 广播。
 *               但此时如果总线上有多个一体化关节，会将所有关节都开启实时反馈。
 */
void enable_angle_speed_torque_state(uint8_t id_num)
{
	write_property(id_num, 22001, 3, 1);
}

/**
 * @brief 设置角度、转速、力矩状态实时反馈时间间隔函数，单位 ms，默认为 2。
 * 设置角度、转速、力矩状态实时反馈时间间隔，单位 ms，默认为 2。
 * 特别提醒：当总线中不同 ID 号关节数量为 n 时，建议将所有关节的该值统一设置为 2n。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0 广播。
 *               但此时如果总线上有多个一体化关节，会将所有关节都设置实时反馈时间间隔。
 * @param n_ms 角度、转速、力矩状态实时反馈时间间隔，单位 ms，当总线中不同 ID 号关节数量为 n 时，请将该值设置为 2n。
 */
void set_state_feedback_rate_ms(uint8_t id_num, uint32_t n_ms)
{
	write_property(id_num, 31002, 3, n_ms);
}

/**
 * @brief 读取一体化关节角度、转速、力矩实时状态反馈的函数。
 * 读取一体化关节角度 (angle °)、转速(speed r/min)、力矩(torque Nm)实时状态反馈。
 * 特别提醒：
 *         1. 由于总线实时发送数据，读取的第一组数据可能出错，因此该函数连续运行第二次之后才能保证数据有效性
 *         2. 运行任何其他回读参数的函数前，须先运行 disable_angle_speed_torque_state() 关闭实时反馈
 *
 * @param id_num 一体化关节 ID 编号，注意使用该函数时最好已经将总线中的关节设置为 1~63 号，并且没有相同 ID 号的关节。
 * @return angle_speed_torque：[angle, speed, torque]，分别表示角度(°)、转速(r/min)、力矩(Nm)
 */
struct angle_speed_torque angle_speed_torque_state(uint8_t id_num)
{
	READ_FLAG=0;
	struct angle_speed_torque angle_speed_torque = {0, 0, 0};
	while ((id_num != (uint8_t)(((can_id & 0x07E0) >> 5)&0xFF)))
	{
		receive_data();
	}
	if (id_num == ((uint8_t)((can_id & 0x07E0) >> 5)&0xFF))
	{
		float factor = 0.01f;
		float value_data[3]= {0,0,0};
		int type_data[3]= {0,2,2};
		format_data(value_data,type_data,3,"decode");
		angle_speed_torque.angle = data_list.value_data[0];
		angle_speed_torque.speed = data_list.value_data[1]*factor;
		angle_speed_torque.torque = data_list.value_data[2]*factor;
	}
	return angle_speed_torque;
}

/**
 * @brief 取消角度、转速、力矩实时反馈函数。
 * 取消角度、转速、力矩实时反馈。
 * 特别提醒：运行任何其他回读参数的函数前，须先运行 disable_angle_speed_torque_state() 关闭实时反馈。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0 广播。
 *               但此时如果总线上有多个一体化关节，会将所有关节都取消实时反馈。
 */
void disable_angle_speed_torque_state(uint8_t id_num)
{
	write_property(id_num, 22001, 3, 0);
}

/**
 * @brief 读取一体化关节控制环 PID 函数。
 * 读取一体化关节控制环的位置增益 P、转速增益 D、积分增益 I。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道不知道当前一体化关节 ID 编号，可以用 0 广播。
 *               但此时如果总线上有多个一体化关节，会造成总线通信干扰，不可使用 0 广播。
 * @return [P, I, D]：[位置增益 P, 积分增益 I, 转速增益 D]。
 */
struct PID get_pid(uint8_t id_num)
{
	struct PID pid = {0,0,0};
	pid.P = read_property(id_num,32102,0);
	pid.D = read_property(id_num,32103,0);
	pid.I =  read_property(id_num,32104,0);
    return pid;
}

/**
 * @brief 同时读取一体化关节当前角度（°）和转速（r/min）。
 * 读取一体化关节当前角度和转速。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0 广播。
 *               但此时如果总线上有多个一体化关节，会造成总线通信干扰，不可使用 0 广播。
 * @return [angle, speed]：当前关节角度（°）和转速（r/min）组成的列表。
 */
struct servo_state get_state(uint8_t id_num)
{
    struct servo_state state = {0, 0};
    state.angle = read_property(id_num,38001,0);
    state.speed = read_property(id_num,38002,0);
    return state;
}

/**
 * @brief 读取一体化关节的当前电压和电流。
 * 读取一体化关节当前电压和q轴电流，单位分别为伏（V）和安（A）。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0 广播。
 *               但此时如果总线上有多个一体化关节，会造成总线通信干扰，不可使用 0 广播。
 * @return [vol, cur]：电压（V）和电流（A）组成的列表。
 */
struct servo_volcur get_volcur(uint8_t id_num)
{
    struct servo_volcur volcur = {0, 0};
    volcur.vol  = read_property(id_num, 1, 0);
    if(READ_FLAG==1) {
        volcur.cur  = read_property(id_num, 33201,0);
    }
    else READ_FLAG=-1;
    return volcur;
}

/**
 * @brief 读取一体化关节属性参数。
 * 读取一体化关节属性参数，这里的属性参数为一体化关节控制参数，存放于 interface_enums.py 文件。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0 广播。
 *               但此时如果总线上有多个一体化关节，会造成总线通信干扰，不可使用 0 广播。
 * @param property 需要设置的属性参数名称，例如 "dr.voltage"，"dr.config.can_id"等，
 *                 具体参数名称见 parameter_interface.py 文件里 property_address 字典里的键值。
 * @return value：返回对应对应属性参数的值。
 */
float read_property(uint8_t id_num,int param_address,int param_type)
{
    float value_data[3]= {param_address,param_type,0};
    int type_data[3]= {1,1,3};
    format_data(value_data,type_data,3,"encode");
    READ_FLAG=0;
    send_command(id_num,0x1E,data_list.byte_data,0);// 需要用标准帧（数据帧）进行发送，不能用远程帧

    receive_data();

    if (READ_FLAG == 1)
    {
        float value_data[3]= {0,0,0};
        int type_data[3]= {1,1,param_type};
        format_data(value_data,type_data,3,"decode");

        float value=data_list.value_data[2];
        return value;
    }
    else
    {
        READ_FLAG=-1;
        return 0;
    }
}

/*
其他系统辅助函数，一般情况下无需使用
*/


/**
 * @brief 保存配置函数。
 * 保存用户修改的一体化关节属性参数，这里的属性参数为一体化关节控制参数，存放于 interface_enums.py 文件。
 * 正常情况下，通过 write_property 函数修改的属性一体化关节关机或重启之后，会恢复为修改前的值；
 * 如果想永久保存，则需要用 save_config 函数将相关参数保存到 flash 中，关机或重启后不丢失。
 * 注：建议在无负载的情况下执行此命令，否则可能造成关节短暂卸载。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0 广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 */
void save_config(uint8_t id_num)
{

    unsigned char order_num = 0x01;

    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num,0x08, data_list.byte_data,0);  //需要用标准帧（数据帧）进行发送，不能用远程帧
    SERVO_DELAY(2000);
}

/**
 * @brief 一体化关节重启函数。
 * 设置一体化关节软件重启，效果与重新上电类似。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0 广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 */
void reboot(uint8_t id_num)
{

    unsigned char order_num = 0x03;

    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num,0x08, data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧

}

/**
 * @brief 恢复出厂设置。
 * 恢复出厂时的参数配置，不改变用户设置的 ID 号。
 * 建议在无负载的情况下执行此命令，否则可能造成关节短暂卸载。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID，可以用 0 广播，如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 */
void init_config(uint8_t id_num)
{
    float value_data[3]= {id_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num,0x0E, data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
}

/**
 * @brief 一体化关节清除错误函数。
 * 清除一体化关节软件报错。
 *
 * @param id_num 一体化关节 ID 编号，如果不知道当前一体化关节 ID 编号，可以用 0 广播，此时如果总线上有多个一体化关节，则多个一体化关节都会执行该操作。
 */
void clear_error(uint8_t id_num)
{
    unsigned char order_num = 0x04;

    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num,0x08, data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧

}


