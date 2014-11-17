#ifndef ImagingFilter_h__
#define ImagingFilter_h__

#include "ImagingBuffer.h"
#include <utility>

namespace SimpleImagingLib {

#pragma pack( push, 1 )
struct XBGR
{
	uint8_t B;
	uint8_t G;
	uint8_t R;
	uint8_t X;

};
#pragma pack( pop )

class SIMPLEIMAGINGLIB_API ImagingFilter
{
public:

	ImagingFilter()
	{
	}

	virtual ~ImagingFilter()
	{
	}

	void Apply( ImagingBuffer* aImage )
	{
		Apply( aImage->GetTopPtr(), aImage->GetWidth(), aImage->GetHeight(), aImage->GetStride() );
	}

	void Apply( uint8_t* aBuffer, uint32_t aImageWidth, uint32_t aImageHeight, int32_t aStride )
	{
		uint8_t* lLine = aBuffer;
		for( uint32_t y = 0; y < aImageHeight; y++ )
		{
			
			ProcessScanLine( lLine, aImageWidth );
			lLine += aStride;
		}
	}

protected:

	virtual void ProcessScanLine( uint8_t* aBuffer, uint32_t aWidth ){}


};

template <typename T> T 
    Clip( const T& ioValue, const T& aLower, const T& aUpper ) 
{
	using namespace std;
	return max( aLower, min( ioValue, aUpper ) );
}

}  // namespace SimpleImagingLib

#endif // ImagingFilter_h__
