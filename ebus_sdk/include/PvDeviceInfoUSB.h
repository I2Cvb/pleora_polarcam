// *****************************************************************************
//
//     Copyright (c) 2013, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PVDEVICEINFOUSB_H__
#define __PVDEVICEINFOUSB_H__

#include <PvDeviceInfo.h>


class PV_DEVICE_API PvDeviceInfoUSB : public PvDeviceInfo
{
public:

	virtual ~PvDeviceInfoUSB();

    PvUSBStatus GetStatus() const;

protected:

	PvDeviceInfoUSB( PvDeviceLib::DeviceInfo *aThis );

    friend class PvDeviceUSB;

private:

    const PvUSBStatus *mStatus;

	 // Not implemented
    PvDeviceInfoUSB( const PvDeviceInfoUSB & );
	const PvDeviceInfoUSB &operator=( const PvDeviceInfoUSB & );

};


#ifdef PV_INTERNAL_HEADERS
    #include <PvDeviceLib/DeviceInfoUSB.h>
#endif // PV_INTERNAL_HEADERS


#endif // __PVDEVICEINFOUSB_H__

