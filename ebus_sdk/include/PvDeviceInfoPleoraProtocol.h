// *****************************************************************************
//
//     Copyright (c) 2013, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PVDEVICEINFOPLEORAPROTOCOL_H__
#define __PVDEVICEINFOPLEORAPROTOCOL_H__

#include <PvDeviceInfo.h>


class PV_DEVICE_API PvDeviceInfoPleoraProtocol : public PvDeviceInfo
{
public:

    PvString GetMACAddress() const;
    PvString GetIPAddress() const;
    PvString GetSubnetMask() const;
    PvString GetDefaultGateway() const;

    uint8_t GetDeviceID() const;
    uint8_t GetModuleID() const;
    uint8_t GetSubID() const;
    uint8_t GetVendorID() const;
    uint8_t GetSoftwareMajor() const;
    uint8_t GetSoftwareMinor() const;

protected:

	PvDeviceInfoPleoraProtocol( PvDeviceLib::DeviceInfo *aThis );
	virtual ~PvDeviceInfoPleoraProtocol();

private:

	 // Not implemented
    PvDeviceInfoPleoraProtocol( const PvDeviceInfoPleoraProtocol & );
	const PvDeviceInfoPleoraProtocol &operator=( const PvDeviceInfoPleoraProtocol & );

    const std::string *mIPAddress;
    const std::string *mMACAddress;
    const std::string *mSubnetMask;
    const std::string *mDefaultGateway;
    const uint8_t *mDeviceID;
    const uint8_t *mModuleID;
    const uint8_t *mSubID;
    const uint8_t *mVendorID;
    const uint8_t *mSoftwareMajor;
    const uint8_t *mSoftwareMinor;

};


#ifdef PV_INTERNAL_HEADERS
    #include <PvDeviceLib/DeviceInfoPleoraProtocol.h>
#endif // PV_INTERNAL_HEADERS


#endif // __PVDEVICEINFOPLEORAPROTOCOL_H__

