// *****************************************************************************
//
//     Copyright (c) 2013, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PVNETWORKADAPTER_H__
#define __PVNETWORKADAPTER_H__

#include <PvDeviceLib.h>
#include <PvInterface.h>
#include <PvDeviceInfoGEV.h>
#include <PvDeviceInfoPleoraProtocol.h>


class PV_DEVICE_API PvNetworkAdapter : public PvInterface
{
public:

    PvString GetMACAddress() const;
    PvString GetIPAddress() const;
    PvString GetSubnetMask() const;
    PvString GetDefaultGateway() const;
    PvString GetDescription() const;

    bool IsPleoraDriverInstalled() const;

protected:

    PvNetworkAdapter( PvDeviceLib::Interface *aThis );
    virtual ~PvNetworkAdapter();

private:

	 // Not implemented
	PvNetworkAdapter( const PvNetworkAdapter & );
	const PvNetworkAdapter&operator=( const PvNetworkAdapter & );

    const std::string *mMAC;
    const std::string *mDescription;

    const std::string *mIP;
    const std::string *mSubnetMask;
    const std::string *mGateway;

    const bool *mValidIP;
    const bool *mDriverInstalled;

};


#ifdef PV_INTERNAL_HEADERS
    #include <PvDeviceLib/NetworkAdapter.h>
#endif // PV_INTERNAL_HEADERS


#endif // __PVNETWORKADAPTER_H__
