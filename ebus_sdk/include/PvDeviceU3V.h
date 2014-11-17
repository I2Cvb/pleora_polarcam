// *****************************************************************************
//
//     Copyright (c) 2012, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PV_DEVICE_U3V_H__
#define __PV_DEVICE_U3V_H__

#include <PvDevice.h>


class PV_DEVICE_API PvDeviceU3V : public PvDevice
{
public:

	PvDeviceU3V();
	virtual ~PvDeviceU3V();

	PvResult Connect( const PvDeviceInfo *aDeviceInfo );

protected:
    
private:

	 // Not implemented
	PvDeviceU3V( const PvDeviceU3V & );
	const PvDeviceU3V &operator=( const PvDeviceU3V & );

};


#endif // __PV_DEVICE_U3V_H__


