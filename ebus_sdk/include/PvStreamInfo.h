// *****************************************************************************
//
//     Copyright (c) 2012, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PV_STREAM_INFO_H__
#define __PV_STREAM_INFO_H__

#include <PvAppUtilsLib.h>
#include <PvStream.h>


namespace PvAppUtilsLib
{
    class StreamInfo;

}; // namespace PvAppUtilsLib


class PV_APPUTILS_API PvStreamInfo
{
public:

    PvStreamInfo( PvStream *aStream );
	~PvStreamInfo();

	PvString GetStatistics( uint32_t aDisplayFrameRate );
	PvString GetErrors();
	PvString GetWarnings( bool aPipelineReallocated );

private:

    PvAppUtilsLib::StreamInfo *mThis;

	 // Not implemented
	PvStreamInfo( const PvStreamInfo & );
	const PvStreamInfo &operator=( const PvStreamInfo & );

};


#endif //__PV_STREAM_INFO_H__

