// *****************************************************************************
//
//     Copyright (c) 2009, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PV_GENREGISTER_H__
#define __PV_GENREGISTER_H__

#include <PvGenICamLib.h>
#include <PvGenParameter.h>


class PvGenRegister : public PvGenParameter
{
public:

    PV_GENICAM_API PvResult Set( const uint8_t *aBuffer, int64_t aLength );
    PV_GENICAM_API PvResult Get( uint8_t *aBuffer, int64_t aLength ) const;

    PV_GENICAM_API PvResult GetLength( int64_t &aLength ) const;

protected:

	PvGenRegister();
	virtual ~PvGenRegister();

private:

    // Not implemented
	PvGenRegister( const PvGenRegister & );
	const PvGenRegister &operator=( const PvGenRegister & );
};


#endif // __PV_GENREGISTER_H__
