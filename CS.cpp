//#include <Eigen/Dense>
#ifdef __cplusplus

extern "C"
{
#endif

#include "wit_c_sdk.h"
#include "REG.h"
#include "minmea.h"
#include "seriald.h"
#include "transform.h"
#include "time.h"

#ifdef __cplusplus
}
#endif

#include <iostream>
#include <check.h>
#include "Python.h"


#define ACC_UPDATE        0x01
#define GYRO_UPDATE        0x02
#define ANGLE_UPDATE    0x04
#define MAG_UPDATE        0x08
#define READ_UPDATE       0x80

#define GPSDATALEN       0X80////GPS接收数据buff长度

ssize_t readline(int fd, char *buf, ssize_t maxlen);

static int fd, fs;
static volatile char s_cDataUpdate = 0;


static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);


void kalman_init();

uint8_t gps_refresh();

uint8_t imu_refresh(float *acc, float_t *time);

void kalman_main(float *acc, float_t *time_imu_t);

using namespace std;
time_t tmpcal_ptr;
PyObject *pfunc_return;

PyObject *la_py;
PyObject *lo_py;
PyObject *args_init;
PyObject *time_gps_py;
PyObject *pFunc;
PyObject *main_func;

struct minmea_sentence_gga frame_gga;
struct minmea_sentence_rmc frame_rmc;
double_t la, lo;


PyObject *args_main;

float_t fAcc[3];//加速度
float_t time_imu[3];//
timeval tv;

int main(int argc, char *argv[])
{
    Py_Initialize();

    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('/home/root123/project/cs/python')");
    // PyRun_SimpleString("print(\"pppppp:\",sys.path)");

    PyObject *pModule = PyImport_ImportModule("hello");//kalman  kalman_serial

    pFunc = PyObject_GetAttrString(pModule, "init_data");
    main_func = PyObject_GetAttrString(pModule, "main");


    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitRegisterCallBack(SensorDataUpdata);
    serial_close(fd);
    serial_close(fs);
    fs = serial_open((unsigned char *) argv[2], 115200);//imu
    fd = serial_open((unsigned char *) argv[1], 115200);//gps
    if (fd == -1)
    {
        printf("尝试切换GPS串口...\n");
        serial_close(fd);
        if (serial_open((unsigned char *) "/dev/ttyUSB1", 115200) == -1)
        {
            printf("gps open failed~!\n");
            return -1;
        }
    }
    if (fs == -1)
    {
        printf("尝试切换IMU串口...\n");
        serial_close(fs);
        if (serial_open((unsigned char *) "/dev/ttyACM0", 115200) == -1)
        {
            printf("imu open failed~!\n");
            return -1;
        }

    }

    kalman_init();
    while (true)
    {

        gps_refresh();
        if (imu_refresh(fAcc, time_imu))
        {
            kalman_main(fAcc, time_imu);
        }


    }
}

void kalman_main(float *acc, float_t *time_imu_t)
{
    PyObject *gps_lat;
    PyObject *gps_lon;
    PyObject *abs_north_acc_py;
    PyObject *abs_east_acc_py;
    PyObject *time_imu_py;
    tm *tmp_ptr;

    time(&tmpcal_ptr);
    tmp_ptr = gmtime(&tmpcal_ptr);
    gettimeofday(&tv, NULL);

    args_main = PyTuple_New(6);

    gps_lat = Py_BuildValue("d", la);
    gps_lon = Py_BuildValue("d", lo);
    la = 0;
    lo = 0;
    abs_north_acc_py = Py_BuildValue("f", acc[1]);
    abs_east_acc_py = Py_BuildValue("f", acc[0]);
    //time_imu_py = Py_BuildValue("[i,i,i]", time_imu_t[0] + 8, time_imu_t[1], time_imu_t[2]);
    time_imu_py = Py_BuildValue("[i,i,f]", tmp_ptr->tm_hour + 8, tmp_ptr->tm_min, (float_t)tmp_ptr->tm_sec+(float_t)tv.tv_usec/100000.0);///获取的系统时间
    if (pfunc_return != nullptr)
    {
        PyTuple_SetItem(args_main, 0, pfunc_return);
        PyTuple_SetItem(args_main, 1, gps_lat);
        PyTuple_SetItem(args_main, 2, gps_lon);
        PyTuple_SetItem(args_main, 3, abs_north_acc_py);
        PyTuple_SetItem(args_main, 4, abs_east_acc_py);
        PyTuple_SetItem(args_main, 5, time_imu_py);
        PyObject_CallObject(main_func, args_main);
        Py_INCREF(pfunc_return);
    }
}

uint8_t imu_refresh(float *acc, float_t *time)
{
    uint8_t cBuff[1];
    uint8_t state = 0;
    // float  fGyro[3], fAngle[3];

    if (serial_read_data(fs, cBuff, 1))
    {
        WitSerialDataIn(cBuff[0]);
        if (s_cDataUpdate & ACC_UPDATE)
        {
            for (uint8_t i = 0; i < 3; i++)
            {
                acc[i] = sReg[AX + i] / 32768.0f * 16.0f;

                //fGyro[i] = sReg[GX + i] / 32768.0f * 2000.0f;
                //fAngle[i] = sReg[Roll + i] / 32768.0f * 180.0f;
            }

            time[0] = ((sReg[DDHH] >> 8) & 0xff);//时
            time[1] = (sReg[MMSS] & 0xff);//分
            time[2] = ((sReg[MMSS] >> 8) & 0xff);//秒
            s_cDataUpdate &= ~ACC_UPDATE;
            state = 1;
        }
    }
    //                if (s_cDataUpdate & ACC_UPDATE)
//                {
    //printf("acc:%.3f %.3f %.3f\r\n", fAcc[0], fAcc[1], fAcc[2]);
//                    s_cDataUpdate &= ~ACC_UPDATE;
//                }
//                if (s_cDataUpdate & GYRO_UPDATE)
//                {
//                    printf("gyro:%.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);
//                    s_cDataUpdate &= ~GYRO_UPDATE;
//                }
//                if (s_cDataUpdate & ANGLE_UPDATE)
//                {
//                    printf("angle:%.3f %.3f %.3f\r\n", fAngle[0], fAngle[1], fAngle[2]);
//                    s_cDataUpdate &= ~ANGLE_UPDATE;
//                }
//                if (s_cDataUpdate & MAG_UPDATE)
//                {
//                    printf("mag:%d %d %d\r\n", sReg[HX], sReg[HY], sReg[HZ]);
//                    s_cDataUpdate &= ~MAG_UPDATE;
//                }
    return state;
}


uint8_t gps_refresh()
{
    uint8_t cs[GPSDATALEN] = {0};
    uint8_t state = 0;
    if (readline(fd, (char *) cs, sizeof(cs)))
    {

        if (minmea_parse_gga(&frame_gga, (char *) cs))
        {
            //printf("\r\n");
            //printf(INDENT_SPACES "$xxGGA: latitude: %d\n", frame_gga.latitude.value);
        }

        if (minmea_parse_rmc(&frame_rmc, (char *) cs))
        {
            //printf("时间：%d:%d:%d 微秒:%d\r\n", frame_rmc.time.hours + 8, frame_rmc.time.minutes, frame_rmc.time.seconds,frame_rmc.time.microseconds);
            // printf(INDENT_SPACES "$xxRMC: ԭʼ������ٶ�?: (%d/%d,%d/%d) %d/%d\n",
            //        frame_rmc.latitude.value, frame_rmc.latitude.scale,
            //        frame_rmc.longitude.value, frame_rmc.longitude.scale,
            //        frame_rmc.speed.value, frame_rmc.speed.scale);
            // printf(INDENT_SPACES "$xxRMC ����������ٶȾ�ȷ��С�������λ: (%d,%d) %d\n",
            //        minmea_rescale(&frame_rmc.latitude, 1000),
            //        minmea_rescale(&frame_rmc.longitude, 1000),
            //        minmea_rescale(&frame_rmc.speed, 1000));
            // printf(INDENT_SPACES "$xxRMC �����������ٶ�: (%f,%f) %f\n",
            //        minmea_tocoord(&frame_rmc.latitude),
            //        minmea_tocoord(&frame_rmc.longitude),
            //        minmea_tofloat(&frame_rmc.speed));
            wgs2gcj(minmea_tocoord(&frame_rmc.latitude), minmea_tocoord(&frame_rmc.longitude), &la, &lo);
            state = 1;
        }

    }
    return state;
}

void kalman_init()
{
    uint8_t cs[GPSDATALEN] = {0};
    double_t la_init;
    double_t lo_init;

    while (!minmea_parse_rmc(&frame_rmc, (char *) cs))
    {
        if (readline(fd, (char *) cs, sizeof(cs)))
        {
            if (minmea_parse_gga(&frame_gga, (char *) cs))
            {
                //printf(INDENT_SPACES "$xxGGA: latitude: %d\n", frame_gga.latitude.value);
            }

            if (minmea_parse_rmc(&frame_rmc, (char *) cs))
            {
                wgs2gcj(minmea_tocoord(&frame_rmc.latitude), minmea_tocoord(&frame_rmc.longitude), &la_init, &lo_init);
            }
        }
    }
    la_py = Py_BuildValue("d", la_init);//35.30840333333333
    lo_py = Py_BuildValue("d", lo_init);//-80.74369333333334
    time_gps_py = Py_BuildValue("[i,i,i]", frame_rmc.time.hours + 8, frame_rmc.time.minutes, frame_rmc.time.seconds);

    args_init = PyTuple_New(3);
    // printf("lo,la:%f,%f\r\n", lo, la);
    PyTuple_SetItem(args_init, 0, la_py);
    PyTuple_SetItem(args_init, 1, lo_py);
    PyTuple_SetItem(args_init, 2, time_gps_py);
    pfunc_return = PyObject_CallObject(pFunc, args_init);
    //Py_INCREF(args_init);
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
    int i;
    for (i = 0; i < uiRegNum; i++)
    {
        switch (uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
                s_cDataUpdate |= ACC_UPDATE;
                break;
//            case GX:
//            case GY:
            case GZ:
                s_cDataUpdate |= GYRO_UPDATE;
                break;
//            case HX:
//            case HY:
            case HZ:
                s_cDataUpdate |= MAG_UPDATE;
                break;
//            case Roll:
//            case Pitch:
            case Yaw:
                s_cDataUpdate |= ANGLE_UPDATE;
                break;
            default:
                s_cDataUpdate |= READ_UPDATE;
                break;
        }
        uiReg++;
    }
}


ssize_t readline(int fd_read, char *buf, ssize_t maxlen)
{
    ssize_t count = 0;
    memset(buf, '\0', sizeof((char *) buf));

    char *ptr = buf;
    char tmp = 0;
    while (true)
    {
        read(fd_read, &tmp, sizeof(char));
        if (tmp == '\n' || tmp == '\0') break;

        (*ptr) = tmp;
        ptr++;
        count++;
        if (count >= maxlen)
        {
            printf("Out of buffer!\n");
            return -1;
        }
    }
    return count;
}