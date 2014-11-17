// *****************************************************************************
//
//     Copyright (c) 2008, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PV_SYSTEM_H__
#define __PV_SYSTEM_H__

#include <PvDeviceLib.h>
#include <PvSystemEventSink.h>
#include <PvNetworkAdapter.h>
#include <PvUSBHostController.h>


namespace PvDeviceLib
{
    class System;
};


class PV_DEVICE_API PvSystem
{
public:

    PvSystem();
    virtual ~PvSystem();

    PvResult Find();
    PvResult FindDevice( const PvString &aDeviceToFind, const PvDeviceInfo **aDeviceInfo );

    uint32_t GetDetectionThreadsPriority() const;
    PvResult SetDetectionThreadsPriority( uint32_t aPriority );

    void SetDetectionTimeout( uint32_t aTimeout );
    uint32_t GetDetectionTimeout() const;

    uint32_t GetGEVSupportedVersion() const;
    uint32_t GetU3VSupportedVersion() const;

    PvResult RegisterEventSink( PvSystemEventSink *aEventSink );
    PvResult UnregisterEventSink( PvSystemEventSink *aEventSink );

    uint32_t GetInterfaceCount() const;
    const PvInterface *GetInterface( uint32_t aIndex ) const;

    uint32_t GetDeviceCount() const;
    const PvDeviceInfo *GetDeviceInfo( uint32_t aIndex ) const;

protected:

private:

	 // Not implemented
	PvSystem( const PvSystem & );
	const PvSystem &operator=( const PvSystem & );

    PvDeviceLib::System *mThis;

};


#ifdef PV_INTERNAL_HEADERS
    #include <PvDeviceLib/System.h>
#endif // PV_INTERNAL_HEADERS


#endif // __PV_SYSTEM_H__

