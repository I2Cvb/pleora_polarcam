// *****************************************************************************
//
//     Copyright (c) 2008, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PVSTREAM_IPIPELINESTREAM_H__
#define __PVSTREAM_IPIPELINESTREAM_H__

#include <PtUtilsLib/Result.h>
#include <PtUtilsLib/Types.h>

namespace PvBufferLib
{
    class Buffer;
};

namespace PvStreamLib
{

class PV_STREAM_API IPipelineStream
{
public:
    IPipelineStream() {}
    virtual ~IPipelineStream() {}

    virtual uint32_t GetQueuedBufferCount() const = 0;
    virtual uint32_t GetQueuedBufferMaximum() const = 0;
    virtual PtUtilsLib::Result AbortQueuedBuffers() = 0;
    virtual PtUtilsLib::Result QueueBuffer( PvBufferLib::Buffer * aBuffer ) = 0;
    virtual PtUtilsLib::Result RetrieveBuffer( PvBufferLib::Buffer ** aBuffer, PtUtilsLib::Result * aOperationResult, uint32_t aTimeout = 0 ) = 0;
    virtual void AddPipelineImageDropped() = 0;
    virtual void ResetStats() = 0; 
};

}; // namespace PvStreamLib

#endif // __PVSTREAM_IPIPELINESTREAM_H__
