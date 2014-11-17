// *****************************************************************************
//
//     Copyright (c) 2012, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef _PV_GVSP_TRANSMITTER_RAW_H_
#define _PV_GVSP_TRANSMITTER_RAW_H_

#include <PvTransmitterLib.h>
#include <PvBuffer.h>


namespace PvTransmitterLib
{
    class TransmitterGEV;
};


class PV_TRANSMITTER_API PvTransmitterGEV
{
public:

    PvTransmitterGEV();
    virtual ~PvTransmitterGEV();

    PvResult Open( PvString aDestinationIPAddress, uint16_t aDestinationPort, 
        PvString aSourceIPAddress = "", uint16_t aSourcePort = 0, bool aDontFrag = true,
        bool aExtendedID = false, uint32_t aBuffersCapacity = 64, bool aTimestampWhenSending = false );
    PvResult Close();
    bool IsOpen() const;
    PvResult LoadBufferPool( PvBuffer** aBuffers, uint32_t aBufferCount );
    PvResult QueueBuffer( PvBuffer* aBuffer );
    PvResult RetrieveFreeBuffer( PvBuffer ** aBuffer, uint32_t aTimeout = 0xFFFFFFFF );
    PvResult AbortQueuedBuffers( uint32_t aTimeout = 0xFFFFFFFF, bool* aPartialTransmission = NULL );

    uint32_t GetQueuedBufferCount();
    uint32_t GetPacketSize();
    PvResult SetPacketSize( uint32_t aPacketSize );

    float GetMaxPayloadThroughput();
    PvResult SetMaxPayloadThroughput( float aMaxPayloadThroughput );
    uint16_t GetSourcePort();
    uint16_t GetDestinationPort();

    PvString GetDestinationIPAddress();
    PvString GetSourceIPAddress();
    
    void ResetStats();
    uint64_t GetBlocksTransmitted() const;
    uint64_t GetSamplingTime() const;
    uint64_t GetPayloadBytesTransmitted() const;
    float GetInstantaneousPayloadThroughput() const;
    float GetAveragePayloadThroughput() const;
    float GetInstantaneousTransmissionRate() const;
    float GetAverageTransmissionRate() const;

    uint32_t GetUserModeTransmitterThreadPriority() const;
    PvResult SetUserModeTransmitterThreadPriority( uint32_t aPriority );
    uint32_t GetBufferPoolThreadPriority() const;
    PvResult SetBufferPoolThreadPriority( uint32_t aPriority );

private:

    PvTransmitterLib::TransmitterGEV *mThis;

};


#endif //_PV_GVSP_TRANSMITTER_RAW_H_
