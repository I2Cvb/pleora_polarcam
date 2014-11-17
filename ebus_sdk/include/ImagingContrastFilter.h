#ifndef ImagingContrastFilter_h__
#define ImagingContrastFilter_h__

#include "ImagingFilter.h"

namespace SimpleImagingLib {

class SIMPLEIMAGINGLIB_API ImagingContrastFilter : public ImagingFilter
{
public:

	static const int32_t MAX_LEVEL;
	static const int32_t MIN_LEVEL;

	ImagingContrastFilter();
	~ImagingContrastFilter(void);

    void SetLevel( int32_t aLevel );

	int32_t GetLevel() const;

protected:
    void ProcessScanLine( uint8_t* aBuffer, uint32_t aImageWidth );

    int ApplyContrast( float aPixel );

private:
	int32_t mLevel;
	float   mConstantLevel;
};

}  // namespace SimpleImagingLib

#endif // ImagingContrastFilter_h__

