// *****************************************************************************
//
//     Copyright (c) 2009, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PVDEVICESERIALPORT_H__
#define __PVDEVICESERIALPORT_H__


#include <PvSerialLib.h>
#include <IPvDeviceAdapter.h>


namespace PvSerialLib
{
    class DeviceSerialPort;
}


typedef enum 
{
    PvDeviceSerialInvalid = -1,
    PvDeviceSerial0 = 0,
    PvDeviceSerial1 = 1,
    PvDeviceSerialBulk0 = 2,
    PvDeviceSerialBulk1 = 3,
    PvDeviceSerialBulk2 = 4,
    PvDeviceSerialBulk3 = 5,
    PvDeviceSerialBulk4 = 6,
    PvDeviceSerialBulk5 = 7,
    PvDeviceSerialBulk6 = 8,
    PvDeviceSerialBulk7 = 9

} PvDeviceSerial;


class PV_SERIAL_API PvDeviceSerialPort
{
public:

    PvDeviceSerialPort();
    virtual ~PvDeviceSerialPort();

    PvResult Open( IPvDeviceAdapter *aDevice, PvDeviceSerial aPort );
    PvResult Close();
    bool IsOpened();

    PvResult Write( const uint8_t *aBuffer, uint32_t aSize, uint32_t &aBytesWritten );
    PvResult Read( uint8_t *aBuffer, uint32_t aBufferSize, uint32_t &aBytesRead, uint32_t aTimeout = 0 );

    PvResult FlushRxBuffer();
    PvResult GetRxBytesReady( uint32_t &aBytes ); 
    PvResult GetRxBufferSize( uint32_t &aSize );
    PvResult SetRxBufferSize( uint32_t aSize );

    static bool IsSupported( IPvDeviceAdapter *aDevice, PvDeviceSerial aPort );

private:

    // Not implemented
	PvDeviceSerialPort( const PvDeviceSerialPort & );
	const PvDeviceSerialPort &operator=( const PvDeviceSerialPort & );

    PvSerialLib::DeviceSerialPort * mThis;
};


#endif // __PVDEVICESERIALPORT_H__

