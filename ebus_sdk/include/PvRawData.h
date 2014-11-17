// *****************************************************************************
//
//     Copyright (c) 2010, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PVRAWDATA_H__
#define __PVRAWDATA_H__

#include <PvResult.h>
#include <PvTypes.h>
#include <PvPixelType.h>


namespace PvBufferLib
{
    class RawData;
    class Buffer;
}


class PV_BUFFER_API PvRawData
{
public:


protected:

	PvRawData( PvBufferLib::RawData *aRawData );
    virtual ~PvRawData();

public:

    uint64_t GetPayloadLength() const;

    PvResult Alloc( uint64_t aPayloadLength );
	void Free();

    PvResult Attach( void * aRawBuffer, uint64_t aPayloadLength );
	uint8_t *Detach();

private:

	friend class PvBufferLib::Buffer;

	// Not implemented
	PvRawData( const PvRawData & );
	const PvRawData &operator=( const PvRawData & );

    PvBufferLib::RawData *mThis;
};


#ifdef PV_INTERNAL_HEADERS
    #include <PvBufferLib/Buffer.h>
#endif // PV_INTERNAL_HEADERS


#endif // __PVRAWDATA_H__

