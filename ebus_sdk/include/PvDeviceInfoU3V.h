// *****************************************************************************
//
//     Copyright (c) 2013, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PVDEVICEINFOU3V_H__
#define __PVDEVICEINFOU3V_H__

#include <PvDeviceInfoUSB.h>
#include <PvUSBHostController.h>


class PvDeviceU3V;


class PV_DEVICE_API PvDeviceInfoU3V : public PvDeviceInfoUSB
{
public:

	virtual ~PvDeviceInfoU3V();

    uint32_t GetGenCPVersion() const;
    uint32_t GetU3VVersion() const;

    PvString GetDeviceGUID() const;
    PvString GetFamilyName() const;
    PvString GetU3VSerialNumber() const;

	bool IsLowSpeedSupported() const;
    bool IsFullSpeedSupported() const;
    bool IsHighSpeedSupported() const;
    bool IsSuperSpeedSupported() const;

    PvUSBSpeed GetSpeed() const;

    uint32_t GetMaxPower() const;

    bool IsPleoraDriverInstalled() const;

protected:

	PvDeviceInfoU3V( PvDeviceLib::DeviceInfo *aThis );

    friend class PvDeviceU3V;

private:

	 // Not implemented
    PvDeviceInfoU3V( const PvDeviceInfoU3V & );
	const PvDeviceInfoU3V &operator=( const PvDeviceInfoU3V & );

    const uint32_t *mGenCPVersion;
    const uint32_t *mU3VVersion;

    const std::string *mDeviceGUID;
    const std::string *mFamilyName;
    const std::string *mU3VSerialNumber;

	const bool *mLowSpeedSupported;
    const bool *mFullSpeedSupported;
    const bool *mHighSpeedSupported;
    const bool *mSuperSpeedSupported;

    const PvUSBSpeed *mSpeed;
    const uint32_t *mMaxPower;
    const bool *mPleoraDriverInstalled;

};


#ifdef PV_INTERNAL_HEADERS
    #include <PvDeviceLib/DeviceInfoU3V.h>
#endif // PV_INTERNAL_HEADERS


#endif // __PVDEVICEINFOU3V_H__

