#ifndef moving_average_h
#define moving_average_h

#include "../primitives/ring_buffer.h"

template<unsigned int filter_size, class T>
class MovingAverage
{
public:
  MovingAverage(T default_value)
  : data_(default_value)
  , sum_(default_value)
  {
  }

  MovingAverage()
  : MovingAverage(0)
  {
  }

  inline T get() const
  {
    return static_cast<T>(sum_/filter_size);
  }

  inline T push(T value)
  {
    T last_val = data_.push_back(value);
    sum_ += value - last_val;

    return get();
  }

private:
  RingBuffer<filter_size, T> data_;
  double sum_;
};

#endif
