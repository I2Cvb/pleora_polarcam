// *****************************************************************************
//
//     Copyright (c) 2013, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __IPV_DEVICE_ADAPTER_H__
#define __IPV_DEVICE_ADAPTER_H__

#include <PvTypes.h>
#include <PvString.h>
#include <PvResult.h>

#include <PvStringList.h>
#include <IPvDeviceEventSink.h>


class IPvDeviceAdapter
{
public:

    virtual bool IsGenIntegerInNodeMap( const PvString &aParameterName ) = 0;
    virtual bool IsGenEnumInNodeMap( const PvString &aParameterName ) = 0;
    virtual bool IsGenRegisterInNodeMap( const PvString &aParameterName ) = 0;
    virtual bool IsGenReadable( const PvString &aParameterName ) = 0;
    virtual bool IsGenWritable( const PvString &aParameterName ) = 0;
    virtual bool IsGenEnumEntryAvailable( const PvString &aParameterName, const PvString &aEnumEntry ) = 0;

    virtual PvResult GetGenIntegerValue( const PvString &aParameterName, int64_t &aValue ) = 0;
    virtual PvResult GetGenEnumEntriesAvailable( const PvString &aParameterName, PvStringList &aList ) = 0;
    virtual PvResult GetGenEnumValue( const PvString &aParameterName, PvString &aEnumEntry ) = 0;
    virtual PvResult SetGenEnumValue( const PvString &aParameterName, const PvString &aEnumEntry ) = 0;
    virtual PvResult GetGenStringValue( const PvString &aParameterName, PvString &aValue ) = 0;
    virtual PvResult GetGenRegisterLength( const PvString &aParameterName, int64_t &aLength ) = 0;
    virtual PvResult GetGenRegisterData( const PvString &aParameterName, uint8_t *aDataBuffer, int64_t aByteCount ) = 0;
    virtual PvResult SetGenRegisterData( const PvString &aParameterName, const uint8_t *aDataBuffer, int64_t aByteCount ) = 0;

    virtual PvResult RegisterGenInvalidator( const PvString &aParameterName ) = 0;
    virtual PvResult UnregisterGenInvalidator( const PvString &aParameterName ) = 0;

    virtual PvResult WriteRegister( int64_t aAddress, uint32_t aValue ) = 0;
    virtual PvResult ReadRegister( int64_t aAddress, uint32_t &aValue ) = 0;
	virtual PvResult WriteMemory( const uint8_t *aBuffer, int64_t aAddress, int64_t aLength ) = 0;
	virtual PvResult ReadMemory( uint8_t *aBuffer, int64_t aAddress, int64_t aLength ) = 0;

    virtual PvResult WaitForMessagingChannelIdle( uint32_t aTimeout ) = 0;

    virtual bool IsConnected() = 0;
    virtual bool IsPleoraPowered() = 0;
    virtual bool IsGigEVision() = 0;
    virtual bool IsUSB3Vision() = 0;

    virtual PvResult RegisterEventSink( IPvDeviceEventSink *aEventSink ) = 0;
    virtual PvResult UnregisterEventSink( IPvDeviceEventSink *aEventSink ) = 0;

protected:

private:

};


#endif // __IPV_DEVICE_ADAPTER_H__



