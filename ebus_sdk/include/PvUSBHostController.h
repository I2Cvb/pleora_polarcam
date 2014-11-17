// *****************************************************************************
//
//     Copyright (c) 2013, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PVUSBHOSTCONTROLLER_H__
#define __PVUSBHOSTCONTROLLER_H__

#include <PvDeviceLib.h>
#include <PvInterface.h>
#include <PvDeviceInfoU3V.h>


class PV_DEVICE_API PvUSBHostController : public PvInterface
{
public:

    uint32_t GetVendorID() const;
    uint32_t GetDeviceID() const;
    uint32_t GetSubsystemID() const;

    uint32_t GetRevision() const;

    PvUSBSpeed GetSpeed() const;

protected:

    PvUSBHostController( PvDeviceLib::Interface *aThis );
    virtual ~PvUSBHostController();

private:

	 // Not implemented
	PvUSBHostController( const PvUSBHostController & );
	const PvUSBHostController&operator=( const PvUSBHostController & );

    const uint32_t *mVendorID;
    const uint32_t *mDeviceID;
    const uint32_t *mSubsystemID;
    const uint32_t *mRevision;
    const PvUSBSpeed *mSpeed;

};


#ifdef PV_INTERNAL_HEADERS
    #include <PvDeviceLib/USBHostController.h>
#endif // PV_INTERNAL_HEADERS


#endif // __PVUSBHOSTCONTROLLER_H__

