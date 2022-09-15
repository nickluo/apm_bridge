#pragma once

#include <cstdint>
#include <array>

namespace apm_bridge {

template<std::uint32_t N = 1>
class EMAFilter
{
private:
    const double alpha;
    std::array<double, N> data;
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
    double operator[](int index) const
    {
        return data[index];
    }
    bool check(int index) const
    {
        return set[index];
    }
    double filter(double val, int index = 0)
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

    void filter(const double *val, double *result)
    {
        for(auto i=0u; i<N; ++i)
            result[i] = filter(val[i], i);
    }
};

} // namespace kestrel
