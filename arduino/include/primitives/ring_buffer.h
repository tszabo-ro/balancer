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

    T push_back(T value)
    {
        T last_val = data_[start_];
        data_[start_++] = value;

        if (start_ >= buffer_size)
        {
            start_ = 0;
        }
        return last_val;
    }

    T& operator[](unsigned long index)
    {
        return data_[indexToPos(index)];
    }

    const T& operator[](unsigned long index) const
    {
        return data_[indexToPos(index)];
    }

    T& front()
    {
      return data_[start_];
    }
    const T& front() const
    {
      return data_[start_];
    }

    T& back()
    {
      return data_[indexiToPos(buffer_size - 1)];
    }
    const T& back() const
    {
      return data_[indexiToPos(buffer_size - 1)];
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