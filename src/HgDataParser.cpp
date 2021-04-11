//
// Created by Yancheng Dong on 2021/4/11.
//

#include "HgDataParser.h"

namespace HgDataParser
{
    /*-----------------------------------------------------------------------------------------------------------------------------*/
    //COMMON
    /*-----------------------------------------------------------------------------------------------------------------------------*/
    // Deserialize Data Overloaded functions definition
    int Deserialize(UINT8 *buffer, int startOffset, struct HG4930ControlMessage *Message, UINT8 MsgType);
    int Deserialize(UINT8 *buffer, int startOffset, struct HG4930InertialMessage *Message, UINT8 MsgType);

    /// <summary>Calculate HG IMU standartized Checksum</summary>
    /// <param name="buffer">Buffer containing binary/hex data</param>
    /// <param name="startOffset">Byte containing the IMU Address (0 based)</param>
    /// <param name="byteLength">MSG Byte length - 2 (excluding checksum from message)</param>
    /// <returns>0 - OK, 1 - Incorrect</returns>
    int HgChecksum(UINT8 *buffer, int startOffset, int byteLength)
    {

        UINT16 u16sum = 0;
        UINT8* pb = &buffer[startOffset];

        for (int i = 0; i < byteLength; i = i + 2)
        {
            u16sum += *(UINT16 *)(pb + i);
        }
        UINT16 Checksum = *(UINT16 *)(pb + byteLength);

        if (Checksum == u16sum)
            return 0;
        else
            return 1;
    }

    /*-----------------------------------------------------------------------------------------------------------------------------*/
    //HG4930
    /*-----------------------------------------------------------------------------------------------------------------------------*/

    /// <summary>Fill HG4930 Control Message structure from raw byte array containing 0x01 Control Message</summary>
    /// <param name="buffer">Buffer containing binary/hex data</param>
    /// <param name="startOffset">Byte containing the IMU Address (0 based)</param>
    /// <param name="Message">container structure</param>
    /// <returns>(inherited from deserialize function) 0 - OK | -1 - incorrect start offset | -2 - incorrect message type | 1 - Incorrect Checksum</returns>
    int GetHG4930X01ControlMessage(UINT8 *buffer, int startOffset, struct HG4930ControlMessage *Message)
    {
        return Deserialize(buffer, startOffset, Message, 0x01);
    }
    /*-----------------------------------------------------------------------------------------------------------------------------*/

    /// <summary>Fill HG4930 Inertial Message structure from raw byte array containing 0x02 Inertial Message</summary>
    /// <param name="buffer">Buffer containing binary/hex data</param>
    /// <param name="startOffset">Byte containing the IMU Address (0 based)</param>
    /// <param name="Message">container structure</param>
    /// <returns>(inherited from deserialize function) 0 - OK | -1 - incorrect start offset | -2 - incorrect message type | 1 - Incorrect Checksum</returns>
    int GetHG4930X02InertialMessage(UINT8 *buffer, int startOffset, struct HG4930InertialMessage *Message)
    {
        return Deserialize(buffer, startOffset, Message, 0x02);
    }

    /*-----------------------------------------------------------------------------------------------------------------------------*/

    /// <summary>HG4930 Control Message from raw byte array Deserialize overloaded function</summary>
    /// <param name="buffer">Buffer containing binary/hex data</param>
    /// <param name="startOffset">Byte containing the IMU Address (0 based)</param>
    /// <param name="Message">container structure</param>
    /// <param name="MsgType">0x definition of message type</param>
    /// <returns>0 - OK | -1 - incorrect start offset | -2 - incorrect message type | 1 - Incorrect Checksum</returns>
    int Deserialize(UINT8 *buffer, int startOffset, struct HG4930ControlMessage *Message, UINT8 MsgType)
    {
        if (startOffset < 0)
            return -1;

        UINT8 *pb = &buffer[startOffset];

        Message->IMUAddress = pb[0];
        Message->MessageID = pb[1];

        if (MsgType == 0x01)
        {
            // rad/s
            Message->AngularRate[0] = *(short*)(pb + 2) * 0.0005722046f;
            Message->AngularRate[1] = *(short*)(pb + 4) * 0.0005722046f;
            Message->AngularRate[2] = *(short*)(pb + 6) * 0.0005722046f;
            // m/s2
            Message->LinearAcceleration[0] = *(short*)(pb + 8) * 0.01116211f;
            Message->LinearAcceleration[1] = *(short*)(pb + 10) * 0.01116211f;
            Message->LinearAcceleration[2] = *(short*)(pb + 12) * 0.01116211f;
        }
        else
        {
            return -2;
        }
        Message->MultiplexedCounter = (UINT8)((pb[14] >> 0) & 0x03);
        Message->ModeIndicator = (UINT8)((pb[14] >> 2) & 0x03);
        Message->StatusWord.IMU = (pb[14] & 0x10) != 0;
        Message->StatusWord.Gyro = (pb[14] & 0x20) != 0;
        Message->StatusWord.Accelerometer = (pb[14] & 0x40) != 0;
        Message->StatusWord.GyroOK = (pb[14] & 0x80) != 0;
        Message->StatusWord.GyroXValidity = (pb[15] & 0x01) == 0 /*false fail*/;
        Message->StatusWord.GyroYValidity = (pb[15] & 0x02) == 0 /*false fail*/;
        Message->StatusWord.GyroZValidity = (pb[15] & 0x04) == 0 /*false fail*/;
        Message->StatusWord.IMUOK = (pb[15] & 0x80) != 0;
        Message->StatusWord2A2BFlag = ((pb[17] & 0x80) != 0) ? (UINT8)1 : (UINT8)0;
        if (Message->StatusWord2A2BFlag == 0)
            Message->SoftwareVersionNumber = (*(short*)(pb + 16));

        if (Message->StatusWord2A2BFlag == 1)
            Message->Temperature = ((INT8)pb[16]);

        Message->StatusWord.GyroHealth = (pb[17] & 0x01) != 0;
        Message->StatusWord.StartDataFlag = (pb[17] & 0x02) != 0;
        Message->StatusWord.ProcessTest = (pb[17] & 0x04) != 0;
        Message->StatusWord.MemoryTest = (pb[17] & 0x08) != 0;
        Message->StatusWord.ElectronicsTest = (pb[17] & 0x10) != 0;
        Message->StatusWord.GyroHealth2 = (pb[17] & 0x20) != 0;
        Message->StatusWord.AccelerometerHealth = (pb[17] & 0x40) != 0;
        Message->Checksum = (*(short*)(pb + 18));

        return HgChecksum(buffer, startOffset, 18);
    }

    /// <summary>HG4930 Inertial Message from raw byte array Deserialize overloaded function</summary>
    /// <param name="buffer">Buffer containing binary/hex data</param>
    /// <param name="startOffset">Byte containing the IMU Address (0 based)</param>
    /// <param name="Message">container structure</param>
    /// <param name="MsgType">0x definition of message type</param>
    /// <returns>0 - OK | -1 - incorrect start offset | -2 - incorrect message type | 1 - Incorrect Checksum</returns>
    int Deserialize(UINT8 *buffer, int startOffset, struct HG4930InertialMessage *Message, UINT8 MsgType)
    {
        if (startOffset < 0)
            return -1;

        int status = 0;
        UINT8 *pb = &buffer[startOffset];
        if (MsgType == 0x02)
        {
            status = GetHG4930X01ControlMessage(buffer, startOffset, &Message->ControlMessage);
            // rad
            Message->DeltaAngle[0] = *(INT32*)(pb + 18) * 1.164153E-10f;
            Message->DeltaAngle[1] = *(INT32*)(pb + 22) * 1.164153E-10f;
            Message->DeltaAngle[2] = *(INT32*)(pb + 26) * 1.164153E-10f;
            // m/s
            Message->DeltaVelocity[0] = *(INT32*)(pb + 30) * 2.270937E-09f;
            Message->DeltaVelocity[1] = *(INT32*)(pb + 34) * 2.270937E-09f;
            Message->DeltaVelocity[2] = *(INT32*)(pb + 38) * 2.270937E-09f;
        }
        else
        {
            return -2;
        }
        Message->ControlMessage.Checksum = (*(short*)(pb + 42));

        if (status >= 0)
            return HgChecksum(buffer, startOffset, 42);
        else
            return status;

    }

}