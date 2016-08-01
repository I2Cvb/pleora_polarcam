// *****************************************************************************
//
//     Copyright (c) 2007, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PVSTREAM_H__
#define __PVSTREAM_H__

#include <PvTypes.h>
#include <PvResult.h>
#include <PvString.h>
#include <PvBuffer.h>
#include <PvStreamEventSink.h>
#include <PvGenParameterArray.h>

#include <PvStreamLib.h>


namespace PvStreamLib
{
    class Stream;
};


class PvConfigurationWriter;
class PvConfigurationReader;
class PvPipeline;


typedef enum
{
    PvStreamTypeUnknown = -1,
    PvStreamTypeGEV = 0,
    PvStreamTypeU3V = 1

} PvStreamType;


class PV_STREAM_API PvStream
{
public:
	
    virtual ~PvStream();

    uint32_t GetQueuedBufferCount() const;
    uint32_t GetQueuedBufferMaximum() const;

    static PvStream *CreateAndOpen( const PvString &aInfo, PvResult *aResult );
    static void Free( PvStream *aStream );

    PvResult Close();

    PvStreamType GetType() const;

    PvResult AbortQueuedBuffers();
    PvResult QueueBuffer( PvBuffer * aBuffer );
    PvResult RetrieveBuffer( PvBuffer ** aBuffer, PvResult * aOperationResult, uint32_t aTimeout = 0xFFFFFFFF );

    uint16_t GetChannel();
    bool IsOpen() const;

    PvResult RegisterEventSink( PvStreamEventSink *aEventSink );
    PvResult UnregisterEventSink( PvStreamEventSink *aEventSink );

    PvGenParameterArray *GetParameters();



protected:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              

	PvStream();

    PvStreamLib::Stream *mThis;

private:

	 // Not implemented
	PvStream( const PvStream & );
    const PvStream &operator=( const PvStream & );

    friend class PvPipeline;
    friend class PvConfigurationWriter;
    friend class PvConfigurationReader;

};


#ifdef PV_INTERNAL_HEADERS
    #include <PvStreamLib/Stream.h>
#endif // PV_INTERNAL_HEADERS


#endif // __PVSTREAM_H__

