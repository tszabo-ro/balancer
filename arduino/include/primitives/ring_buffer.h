#ifndef RingBuffer_h
#define RingBuffer_h

#include <math.h>

template<unsigned long buffer_size, typename T>
class RingBuffer
{
public:
    RingBuffer(T initial_value)
    : start_(0)
    {
        for (unsigned long i=0; i < buffer_size; ++i)
        {
            data_[i] = initial_value;
        }
    }

    RingBuffer() : RingBuffer(0) {}

    constexpr unsigned long size() const
    {
        return buffer_size;
    }

    void push_back(T value)
    {
        data_[start_++] = value;

        if (start_ >= buffer_size)
        {
            start_ = 0;
        }
    }

    T& operator[](unsigned long index)
    {
        return data_[indexToPos(index)];
    }

    const T& operator[](unsigned long index) const
    {
        return data_[indexToPos(index)];
    }

private:
    inline unsigned long indexToPos(unsigned long index) const
    {
        return fmod(start_ + index, buffer_size);
    }

private:
    T data_[buffer_size];
    unsigned long start_;
};

#endif