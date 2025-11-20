#pragma once

#include <cstdint>
#include <array>

namespace apm_bridge {

template<typename T, std::uint32_t N = 1>
class EMAFilter
{
private:
    const T alpha;
    std::array<T, N> data;
    std::array<bool, N> set;
public:
    explicit EMAFilter(int n)
        : alpha(2.0/(n+1))
    {
        reset();
    }
    ~EMAFilter() = default;
    void reset() 
    {
        data.fill(0);
        set.fill(false);
    }
    T operator[](int index) const
    {
        return data[index];
    }
    bool check(int index) const
    {
        return set[index];
    }
    T filter(T val, int index = 0)
    {
        if (!set[index])
        {
            data[index] = val;
            set[index] = true;
        }
        else
        {
            data[index] = alpha * val + (1-alpha)*data[index];
        }
        return data[index];
    }

    void filter(const T *val, T *result)
    {
        for(auto i=0u; i<N; ++i)
            result[i] = filter(val[i], i);
    }
};

} // namespace kestrel
