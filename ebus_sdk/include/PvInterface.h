// *****************************************************************************
//
//     Copyright (c) 2008, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PV_INTERFACE_H__
#define __PV_INTERFACE_H__

#include <PvDeviceLib.h>
#include <PvDeviceInfo.h>


class PvSystem;

namespace PvDeviceLib
{
    class Interface;
    class System;
};


class PV_DEVICE_API PvInterface
{
public:

    PvInterfaceType GetType() const;

    PvString GetName() const;
    PvString GetDisplayID() const;
    PvString GetUniqueID() const;
    
    uint32_t GetDeviceCount() const;
    const PvDeviceInfo *GetDeviceInfo( uint32_t aIndex ) const;

protected:

    PvInterface( PvDeviceLib::Interface *aThis );
    virtual ~PvInterface();

    PvDeviceLib::Interface *mThis;

    friend class PvDeviceLib::System;

private:

	 // Not implemented
	PvInterface( const PvInterface & );
	const PvInterface&operator=( const PvInterface & );

    const std::string *mName;
    const std::string *mDisplayID;
    const std::string *mUniqueID;

};


#ifdef PV_INTERNAL_HEADERS
    #include <PvDeviceLib/Interface.h>
#endif // PV_INTERNAL_HEADERS


#endif // __PV_INTERFACE_H__


