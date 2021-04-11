#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fstream>
#include <ctime>
#include "HgDataParser.h"
#include "Serial.h"
#include "filter_function.h"
#define BUFFER_SIZE 2048
#define HG4930

using namespace std;

int step = 1;
int clear = 1;

int no_const = 1; // 1 : GPS, 2 : GPS + GLONASS, 3 : GPS + GLONASS + GALILEO
int no_state = 15;

double dualheading;

int main(int argc, char **argv)
{
    double state[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //attitude[3],velocity[3],position[3],IMU bias[6],clock bias and clock drift

    double C_b_e[3][3] = {
            {-0.7766, 0.0141, -0.6299},
            {0.0175, 0.9998, 0.0009},
            {0.6298, -0.0103, -0.7767}};

    int i, j;

    double **P = new double *[no_state]; //define a row of covariance double pointer

    for (int i = 1; i < no_state + 1; i++)
    {
        P[i - 1] = new double[no_state];			 //define column for each row - P[no_state][no_state]
        memset(P[i - 1], 0, sizeof(int) * no_state); //initialize the memory space as zero
    }

    for (int i = 1; i < 4; i++)
    {
        P[i - 1][i - 1] = 3.0462e-04;
        P[i + 2][i + 2] = 0.01;
        P[i + 5][i + 5] = 100;
        P[i + 8][i + 8] = 9.6170e-05;
        P[i + 11][i + 11] = 2.3504e-09;
    }

    char comPort[32] = "/dev/cu.usbserial-14120";
    int baudrate = 1000000;

    int status = 0;
    //Connect to defined serial port
    int SerialHandle;
    status = serial_init(&SerialHandle, comPort);
    if (status != 0)
    {
        printf("failed to open Com port:%s\n", comPort);
        return status;
    }
    status = serial_configure(SerialHandle, baudrate, SER_PARITY_NO, 1, 8);
    if (status != 0)
    {
        printf("failed to configure Com port:%s\tstatus:%d\n", comPort, status);
        return status;
    }

    printf("Input COM:\t%s\t%d\n", comPort, baudrate);

    //Allocate the read buffer
    UINT8 ReadBuffer[BUFFER_SIZE * 2] = {0};
    memset(ReadBuffer, 0, BUFFER_SIZE * 2);
    UINT8 *ReadBufferAct = ReadBuffer;
    UINT8 *ReadBufferEnd = ReadBuffer;

    int BytesRead = 0;
    //UINT x = 0;
    status = 2;
#ifdef HG4930
#define INERTIAL_MESSAGE_LEN 44
#endif

#ifdef HG4930
    HgDataParser::HG4930InertialMessage Message;
#endif
    bool IMU_health;
    bool Gyro_health;
    bool Gyro_health2;

    Message.ZeroMessage();
    ///////////////////////////////////////////////////////// IMU Reading Part

    double tor = 0.01;
    double IMU[6];
    double IMU_data[6] = {0, 0, 0, 0, 0, 0};
    double xyz[3];
    double lla[3];

    int num_ins = 0;
    int num_gnss = 0;

    while (1){

        fstream imudataFile;
        fstream covFile;
        fstream stateFile;

        string buffer;
        imudataFile.open("Output_imu.txt", ios::app);
        covFile.open("Covariance.txt", ios::app);
        stateFile.open("State.txt", ios::app);
        ///*
        memcpy(ReadBuffer, ReadBufferAct, (ReadBufferEnd - ReadBufferAct));
        ReadBufferEnd = ReadBuffer + (ReadBufferEnd - ReadBufferAct);
        ReadBufferAct = ReadBuffer;

        if (!serial_read(SerialHandle, (char *)ReadBufferEnd, BUFFER_SIZE, &BytesRead))
        {
            printf("\nFailed to read Data!\n");
        }
        else
        {
            // Move the pointer to the end of the Read buffer
            ReadBufferEnd += (int)BytesRead;

            while (ReadBufferAct <= ReadBufferEnd - INERTIAL_MESSAGE_LEN)
            {
                if (*(UINT8 *)ReadBufferAct == 0x0E)
                {
                    // Switch Message Code - Call appropriate .dll functions
                    auto s = *(UINT8 *)(ReadBufferAct + 1);
                    switch (*(UINT8 *)(ReadBufferAct + 1))
                    {
#ifdef HG4930
                        case 0x0100:
                        {
                            status = HgDataParser::GetHG4930X01ControlMessage(ReadBufferAct, 0, &Message.ControlMessage);
                            ReadBufferAct += 19;

                            break;
                        }
                        case 0x0200:
                        {
                            status = HgDataParser::GetHG4930X02InertialMessage(ReadBufferAct, 0, &Message);
                            ReadBufferAct += 43;

                            break;
                        }
#endif
                        default:
                        {
                            status = 2;
                        }
                    }
                    IMU[0] = -Message.ControlMessage.AngularRate[1]; ////////////////// 1 2 0 - + -
                    IMU[1] = Message.ControlMessage.AngularRate[2];
                    IMU[2] = -Message.ControlMessage.AngularRate[0];
                    IMU[3] = -Message.ControlMessage.LinearAcceleration[1]; ///// cahnge need?????
                    IMU[4] = Message.ControlMessage.LinearAcceleration[2];
                    IMU[5] = -Message.ControlMessage.LinearAcceleration[0];
                    IMU_health = Message.ControlMessage.StatusWord.AccelerometerHealth;
                    Gyro_health = Message.ControlMessage.StatusWord.GyroHealth;
                    Gyro_health2 = Message.ControlMessage.StatusWord.GyroHealth2;

                    if (!IMU_health && !Gyro_health && !Gyro_health2)
                    {
                        num_ins++;

                        IMU_data[0] = IMU[0] - state[13];
                        IMU_data[1] = IMU[1] - state[12];
                        IMU_data[2] = IMU[2] - state[14];
                        IMU_data[3] = IMU[3] - state[9];
                        IMU_data[4] = IMU[4] - state[10];
                        IMU_data[5] = IMU[5] - state[11];

                        tor = 0.00000001;

                        if (num_ins > 1 && num_gnss > 0)
                        {

                            tor = 0.00166666666666666666666666666666;
                            System_Update(IMU_data, state, P, C_b_e, tor, dualheading);
                        }

                        for (i = 0; i < 6; i++)
                        {
                            imudataFile << IMU[i];
                            imudataFile << ",";
                        }

                        covFile << "IMU";
                        for (i = 0; i < 15; i++)
                        {
                            covFile << ",";
                            covFile << setprecision(4) << P[i][i];
                        }

                        stateFile << "IMU,";
                        for (i = 0; i < 15; i++)
                        {
                            if (i > 5 && i < 9)
                            {
                                stateFile << setprecision(12) << state[i];
                                stateFile << ",";
                            }
                            else if (i < 3)
                            {
                                stateFile << setprecision(6) << state[i] / M_PI * 180;
                                stateFile << ",";
                            }
                            else
                            {
                                stateFile << setprecision(4) << state[i];
                                stateFile << ",";
                            }
                        }
                        for (i = 0; i < 3; i++)
                        {
                            stateFile << setprecision(4) << C_b_e[i][i];
                            stateFile << ",";
                        }
                        stateFile << tor;
                        stateFile << ",";

                        for (i = 0; i < 3; i++)
                            xyz[i] = state[i + 6];
                        //ConvertECEF2LLA(xyz, lla);

                        for (i = 0; i < 3; i++)
                        {
                            stateFile << setprecision(12) << lla[i];
                            stateFile << ",";
                        }

                        // stateFile << setprecision(12) << NavSys.sTimeCurrent;
                        stateFile << setprecision(12) << time(NULL);

                        imudataFile << "\n";
                        covFile << "\n";
                        stateFile << "\n";

                        printf("\n Rawdata : %f %f %f %f %f %f %d %f",IMU[0],IMU[1],IMU[2],IMU[3],IMU[4],IMU[5],num_ins,tor);
                        printf("\n IMU_x : %f",IMU[3]);
                        printf("\n IMUdata : %f %f %f %f %f %f %d %f",IMU_data[0],IMU_data[1],IMU_data[2],IMU_data[3],IMU_data[4],IMU_data[5],num_ins,tor);
                        printf("\n system : Velocity, Position : %f  %f  %f  %f  %f  %f",state[3],state[4],state[5],state[6],state[7],state[8]);
                        printf("\n %f  %f  %f  %f  %f  %f %d %f\n",state[9],state[10],state[11],state[12],state[13],state[14],num_ins,tor);
                    }
                }

                //Move the read buffer pointer
                ReadBufferAct++;
            } // !ReadFile

            //Sleep(1);
        }
        imudataFile.close();
        covFile.close();
        stateFile.close();

        //TODO gnss
        if (num_gnss > 10){
           break;
        }
    }
}

