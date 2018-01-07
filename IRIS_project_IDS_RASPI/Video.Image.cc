
#include "Video.Image.h"
//#include "Log/Log.h"

#include <cstring>
#include <iostream>

// LOG_DEFINE_LOGGER(Video::Image::, "Video.Image")

namespace Video
{

Image::~Image()
{
    if (is_memory_allocated_locally_)
    {
        delete[] buffer_;
    }
}

Image::Image(Rect rect, unsigned char bytes_per_pixel,int image_id)
{
        LOG_DEBUG("Allocate new Image with internal image buffer");
        //LOG_DEBUG_RECT("\tInput Rect ", rect);
        LOG_RECT(DEBUG,"\tInput Rect ", rect);
        LOG_DEBUG("\tInput Bytes per pixel " << +bytes_per_pixel);
        LOG_DEBUG("\tInput ID " << image_id);
    rect_ = rect;
    bytes_per_pixel_ = bytes_per_pixel;
    buffer_ = new unsigned char[rect.Width*rect.Height*bytes_per_pixel];
    is_memory_allocated_locally_ = true;
    image_ID_ = image_id;
}

Image::Image(char* already_allocated_buffer, Rect rect, unsigned char bytes_per_pixel, int image_id)
{
    rect_ = rect;
    bytes_per_pixel_ = bytes_per_pixel;
    buffer_ = reinterpret_cast<unsigned char*>(already_allocated_buffer);
    is_memory_allocated_locally_ = false;
    image_ID_ = image_id;
}

unsigned char   Image::GetBytesPerPixel() const
{
    return bytes_per_pixel_;
}

unsigned char *Image::GetBufferUnsafe()
{
    return buffer_;
}

const unsigned char* Image::GetConstBuffer() const
{
    return const_cast<unsigned char*>(buffer_);
}

const Rect      Image::GetRect() const
{
    return rect_;
}

int Image::GetImageID() const
{
    return image_ID_;
}

int  Image::GetRectID() const
{
    return rect_.RectID;
}

bool Image::CopyFrom(const Image& src)
{
    //LOG_DEBUG("Copy frame from source");
    //LOG_RECT(DEBUG,"\tDest Rect ", rect_);
    // this is ensured by design in the service as camera cquires englobing rect of all rois
    int dst_line_byte_width = rect_.Width*bytes_per_pixel_;
    int src_line_byte_width = src.rect_.Width*src.bytes_per_pixel_;
    int src_line_byte_offset = (rect_.Top - src.rect_.Top) *src_line_byte_width;
    int src_column_byte_offset = (rect_.Left - src.rect_.Left) * src.bytes_per_pixel_;
    //LOG_DEBUG(dst_line_byte_width << "    ----    " <<  src_line_byte_width <<"," <<src_line_byte_offset << ","<< src_column_byte_offset);
    for (int line = 0; line < rect_.Height; line++)
    {
        std::memcpy(&(buffer_[line*dst_line_byte_width]),
                    &(src.buffer_[line*src_line_byte_width+src_line_byte_offset+src_column_byte_offset]),
                    dst_line_byte_width);
    }
    return true;
}

TimeStamp&      Image::get_timestamp()
{
    return acquisition_timestamp_;
}

void            Image::update_timestamp(TimeStamp& ts)
{
    acquisition_timestamp_ = ts;
}

}
