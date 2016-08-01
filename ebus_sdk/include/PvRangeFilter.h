// *****************************************************************************
//
//     Copyright (c) 2015, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PV_RANGE_FILTER_H__
#define __PV_RANGE_FILTER_H__

#include <PvAppUtilsLib.h>

#include <PvBuffer.h>
#include <PvConfigurationReader.h>
#include <PvConfigurationWriter.h>


namespace PvAppUtilsLib
{
    class RangeFilter;
}


class PV_APPUTILS_API PvRangeFilter
{
public:

    PvRangeFilter();
	~PvRangeFilter();

    PvBuffer *Process( PvBuffer *aBuffer );
	PvResult GetHistogram( uint32_t *aHistogramPtr, uint32_t aHistogramSize, uint32_t &aMaxValue );

    void Reset();
    void AutoConfigure();

    bool IsEnabled() const;
    uint8_t GetDark() const;
    uint8_t GetLight() const;

    void SetDark( uint8_t aValue );
    void SetLight( uint8_t aValue );
    void SetEnabled( bool aEnabled );

    PvResult Load( PvConfigurationReader *aReader );
    PvResult Save( PvConfigurationWriter *aWriter );

private:

    PvAppUtilsLib::RangeFilter *mThis;

	 // Not implemented
	PvRangeFilter( const PvRangeFilter & );
	const PvRangeFilter &operator=( const PvRangeFilter & );

};


#endif //__PV_RANGE_FILTER_H__

