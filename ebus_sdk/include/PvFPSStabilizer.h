// *****************************************************************************
//
//     Copyright (c) 2012, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PV_FPS_STABILIZER_H__
#define __PV_FPS_STABILIZER_H__

#include <PvAppUtilsLib.h>
#include <PvTypes.h>


namespace PvAppUtilsLib
{
    class FPSStabilizer;

}; // namespace PvAppUtilsLib


class PV_APPUTILS_API PvFPSStabilizer
{
public:

    PvFPSStabilizer();
    ~PvFPSStabilizer();

	bool IsTimeToDisplay( uint32_t aTargetFPS );
    uint32_t GetAverage();

	void Reset();

private:

    PvAppUtilsLib::FPSStabilizer *mThis;

	 // Not implemented
	PvFPSStabilizer( const PvFPSStabilizer & );
	const PvFPSStabilizer &operator=( const PvFPSStabilizer & );

};


#endif //__PV_FPS_STABILIZER_H__


