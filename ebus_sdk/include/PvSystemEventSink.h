// *****************************************************************************
//
//     Copyright (c) 2013, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PV_SYSTEM_EVENT_SINK_H__
#define __PV_SYSTEM_EVENT_SINK_H__

#include <PvDeviceLib.h>


class PvInterface;
class PvDeviceInfo;


class PV_DEVICE_API PvSystemEventSink
{
public:

    PvSystemEventSink();
    virtual ~PvSystemEventSink();

    virtual void OnDeviceFound( 
        const PvInterface *aInterface, const PvDeviceInfo *aDeviceInfo, 
        bool &aIgnore );

};



#endif // __PV_SYSTEM_EVENT_SINK_H__


