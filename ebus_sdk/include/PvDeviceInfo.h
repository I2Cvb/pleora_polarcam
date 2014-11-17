// *****************************************************************************
//
//     Copyright (c) 2007, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PV_DEVICEINFO_H__
#define __PV_DEVICEINFO_H__

#include <PvDeviceLib.h>
#include <PvDeviceEnums.h>

#include <string>


namespace PvDeviceLib
{
    class DeviceInfo;
    class Interface;

}; // namespace PvDeviceLib


class PvInterface;
class PvDevice;


class PV_DEVICE_API PvDeviceInfo
{
public:
	virtual ~PvDeviceInfo();

    PvDeviceInfoType GetType() const;

    PvString GetVendorName() const;
    PvString GetModelName() const;
    PvString GetVersion() const;
    PvString GetManufacturerInfo() const;
    PvString GetSerialNumber() const;
    PvString GetUserDefinedName() const;
    
    PvString GetDisplayID() const;
    PvString GetUniqueID() const;
    PvString GetConnectionID() const;
    
    const PvInterface *GetInterface() const;

    bool IsConfigurationValid() const;
    bool IsLicenseValid() const;

    PvDeviceClass GetClass() const;

protected:

	PvDeviceInfo( PvDeviceLib::DeviceInfo *aThis );

    PvDeviceLib::DeviceInfo *mThis;

    friend class PvDeviceLib::Interface;
    friend class PvDevice;

private:

    const PvInterface *mInterface;
    const std::string *mVendorName;
    const std::string *mModelName;
    const std::string *mVersion;
    const std::string *mManufacturerInfo;
    const std::string *mSerialNumber;
    const std::string *mUserDefinedName;
    const PvDeviceClass *mClass;

	 // Not implemented
    PvDeviceInfo( const PvDeviceInfo & );
	const PvDeviceInfo &operator=( const PvDeviceInfo & );

};


#ifdef PV_INTERNAL_HEADERS
    #include <PvDeviceLib/DeviceInfo.h>
#endif // PV_INTERNAL_HEADERS


#endif // __PV_DEVICEINFO_H__

