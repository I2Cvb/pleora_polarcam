// *****************************************************************************
//
//     Copyright (c) 2011, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PV_VIRTUAL_DEVICE_GEV_H__
#define __PV_VIRTUAL_DEVICE_GEV_H__

#include <PvVirtualDeviceLib.h>

namespace PvVirtualDeviceLib
{
    class VirtualDeviceGEV;
};

class PV_VIRTUAL_DEVICE_API PvVirtualDeviceGEV
{
public:

    PvVirtualDeviceGEV();
    ~PvVirtualDeviceGEV();
    PvResult StartListening( PvString aInfo );
    void StopListening();

    uint32_t GetDevicePortThreadPriority() const;
    PvResult SetDevicePortThreadPriority( uint32_t aPriority );

private:

    PvVirtualDeviceLib::VirtualDeviceGEV* mThis;

};

#endif //__PV_VIRTUAL_DEVICE_GEV_H__

