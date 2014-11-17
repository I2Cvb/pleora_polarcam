// *****************************************************************************
//
//     Copyright (c) 2013, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PVDEVICEINFOGEV_H__
#define __PVDEVICEINFOGEV_H__

#include <PvDeviceInfo.h>


class PvDeviceGEV;


class PV_DEVICE_API PvDeviceInfoGEV : public PvDeviceInfo
{
public:

    PvString GetMACAddress() const;

    PvString GetIPAddress() const;
    PvString GetDefaultGateway() const;
    PvString GetSubnetMask() const;

    uint32_t GetGEVVersion() const;

protected:

	PvDeviceInfoGEV( PvDeviceLib::DeviceInfo *aThis );
	virtual ~PvDeviceInfoGEV();

    friend class PvDeviceGEV;

private:

	 // Not implemented
    PvDeviceInfoGEV( const PvDeviceInfoGEV & );
	const PvDeviceInfoGEV &operator=( const PvDeviceInfoGEV & );

    const std::string *mIPAddress;
    const std::string *mMACAddress;
    const std::string *mDefaultGateway;
    const std::string *mSubnetMask;
    const uint32_t *mGEVVersion;

};


#ifdef PV_INTERNAL_HEADERS
    #include <PvDeviceLib/DeviceInfoGEV.h>
#endif // PV_INTERNAL_HEADERS


#endif // __PVDEVICEINFOGEV_H__

