#ifndef IMAGING_BUFFER_H_
#define IMAGING_BUFFER_H_

#include <string>
#include "SimpleImagingLib.h"
#include "ImagingBufferAllocator.h"

namespace SimpleImagingLib {

class SIMPLEIMAGINGLIB_API ImagingBuffer
{
public:

    ImagingBuffer(void);
    ImagingBuffer( uint32_t aWidth, uint32_t aHeight );

    ~ImagingBuffer(void);

    void AllocateImage( uint32_t aWidth, uint32_t aHeight, uint32_t aPixelDepth );

    uint32_t GetWidth() const;
    uint32_t GetHeight() const;

    uint8_t* GetFirstScanLinePtr();
    uint8_t* GetTopPtr();

    uint32_t GetPixelDepth() const;
    int32_t GetStride() const;

    void FillBlack();

    uint32_t GetBufferSize();
    bool BottomUp() const;

private:
    
	void AllocateImage();

private:
    
	uint32_t mWidth;
	uint32_t mHeight;
	uint32_t mPixelDepth;
	uint8_t* mBuffer;
	SimpleImagingLib::ImagingBufferAllocator* mAllocator;
	int32_t  mStride;
	uint32_t mBufferSize;
	bool     mBottomUp;
    
};

} // end namesapce

#endif

