// *****************************************************************************
//
//     Copyright (c) 2013, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PVSTREAMGEV_H__
#define __PVSTREAMGEV_H__

#include <PvStream.h>



class PV_STREAM_API PvStreamGEV : public PvStream
{
public:
	
	PvStreamGEV();
	virtual ~PvStreamGEV();

	PvResult Open(
        const PvString & aIPAddress,
        uint16_t aLocalPort = 0,
        uint16_t aChannel = 0,
        const PvString & aLocalIpAddress = PvString(), 
        uint32_t aBuffersCapacity = 64 );

    PvResult Open(
        const PvString & aIPAddress,
        const PvString & aMulticastAddr,
        uint16_t aDataPort,
        uint16_t aChannel = 0,
        const PvString & aLocalIPAddress = PvString(), 
        uint32_t aBuffersCapacity = 64 );

    PvResult FlushPacketQueue();
    bool GetWaitForFirstPacketOfBlockToStart() const;
    PvResult SetWaitForFirstPacketOfBlockToStart( bool aWaitForFirstPacketOfBlockToStart );

    uint16_t GetLocalPort();
    PvString GetLocalIPAddress();
    PvString GetMulticastIPAddress();

    uint32_t GetUserModeDataReceiverThreadPriority() const;
    PvResult SetUserModeDataReceiverThreadPriority( uint32_t aPriority );

    static PvResult IsDriverInstalled( PvString &aIPAddress, bool &aInstalled, const PvString & aLocalIPAddress = PvString() );

protected:

private:

	 // Not implemented
	PvStreamGEV( const PvStreamGEV & );
    const PvStreamGEV &operator=( const PvStreamGEV & );
   

};


#endif // __PVSTREAMGEV_H__

