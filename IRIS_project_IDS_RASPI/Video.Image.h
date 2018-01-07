#pragma once
#include <memory>

#include "TimeStamp.h"


//Adaptation for GLobal use (not only  in §fusion)
#define LOG_ADD_TO_CALLSTACK
#define LOG_TRACE_THIS_METHOD
#define LOG_INFO(x)  {std::cout<< x << std::endl;}
#define LOG_ERROR(x) {std::cout<< x << std::endl;}
#define LOG_DEBUG(x) {std::cout<< x << std::endl;}
#define LOG_TRACE(x) {std::cout<< x << std::endl;}




// log rect , level should be TRACE, DEBUG, INFO, ...
#define LOG_RECT(level, desc,rect)  LOG_##level(desc<< " (left,top,width,height) = (" << (rect).Left<< ", "<< (rect).Top<< ", "<< (rect).Width<< ", "<<  (rect).Height<< ") ")


namespace Video{

typedef struct
{
    unsigned short Left;
    unsigned short Top;
    unsigned short Width;
    unsigned short Height;
    int            RectID = -1; // used to identify rects / rois
} Rect;


class Image
{
public:
    Image(Rect rect, unsigned char bytes_per_pixel, int image_id = 0);
    Image(char* already_allocated_buffer, Rect rect, unsigned char bytes_per_pixel, int image_id = 0);

    virtual ~Image();

    const Rect      GetRect() const;
    unsigned char   GetBytesPerPixel() const;
    int             GetImageID() const;
    int             GetRectID() const;

    bool            CopyFrom(const Image& src);

    unsigned char * GetBufferUnsafe();
    const unsigned char* GetConstBuffer() const;

    TimeStamp&      get_timestamp();
    void            update_timestamp(TimeStamp& ts);

private:
    unsigned char*              buffer_;
    bool                        is_memory_allocated_locally_;

    TimeStamp                   acquisition_timestamp_;

    Rect                        rect_;
    unsigned char               bytes_per_pixel_;
    int                         image_ID_;
};

}
