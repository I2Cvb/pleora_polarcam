#ifndef ImagingFilterRGB_h__
#define ImagingFilterRGB_h__

#include "ImagingFilter.h"

namespace SimpleImagingLib {

class SIMPLEIMAGINGLIB_API ImagingFilterRGB : public ImagingFilter
{
public:
	ImagingFilterRGB();

	int8_t GetGainX() const;

    void SetGainR( int8_t aValue );

	int8_t GetGainR() const;

    void SetGainG( int8_t aValue );

	int8_t GetGainG() const;

    void SetGainB( int8_t aValue );

	int8_t GetGainB() const;

protected:
    void ProcessScanLine( uint8_t* aBuffer, uint32_t aImageWidth );

private:
	XBGR mGains;
	
};

}  // namespace SimpleImagingLIb

#endif // ImagingFilterRGB_h__
