// *****************************************************************************
//
//     Copyright (c) 2013, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __IPV_DEVICE_EVENT_SINK_H__
#define __IPV_DEVICE_EVENT_SINK_H__

#include <PvTypes.h>
#include <PvString.h>
#include <PvResult.h>

#include <PvStringList.h>


class IPvDeviceEventSink
{
public: 

    virtual void NotifyEvent( 
        uint16_t aEventID, uint16_t aChannel, uint64_t aBlockID, uint64_t aTimestamp, 
        const void *aData, uint32_t aDataLength ) = 0;

    virtual void NotifyInvalidatedGenParameter( const PvString &aParameterName ) = 0;

};


#endif // __IPV_DEVICE_EVENT_SINK_H__



