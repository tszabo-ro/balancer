#ifndef Array_h
#define Array_h

template<typename T, unsigned long array_size>
class Array
{
public:
    Array(T val)
    {
        for (unsigned long i=0; i < array_size; ++i)
        {
            data_[i] = val;
        }
    }
    Array(): Array(0) {}

    constexpr unsigned long size() const
    {
        return array_size;
    }

    T& operator[](unsigned long i) { return data_[i]; }
    const T& operator[](unsigned long i) const { return data_[i]; }

private:
    T data_[array_size];
};


#endif