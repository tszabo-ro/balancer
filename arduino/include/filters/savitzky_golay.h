#ifndef SavitzkyGolay_H
#define SavitzkyGolay_H

#include "../primitives/array.h"
#include "../primitives/ring_buffer.h"

#include <stdio.h>

// Implementation based on http://www.personal.psu.edu/users/m/r/mrh318/Gorry-AC-1990.pdf
namespace
{
  double gramPoly(int i, int m, int k, int s)
  {
    if (k > 0)
    {
      return (4. * k - 2.) / (k * (2. * m - k + 1.)) * (i * gramPoly(i, m, k - 1, s) + s * gramPoly(i, m, k - 1, s - 1)) - ((k - 1.) * (2. * m + k)) / (k * (2. * m - k + 1.)) * gramPoly(i, m, k - 2, s);
    }
    else
    {
      if (k == 0 && s == 0)
        return 1.;
      else
        return 0.;
    }
  }

  double generalizedFactorial(int a, int b)
  {
    double gf = 1.;

    for (int j = (a - b) + 1; j <= a; j++)
    {
      gf *= j;
    }
    return gf;
  }

  double computeWeight(int i, int t, int m, int n, int s)
  {
    double w = 0;
    for (int k = 0; k <= n; ++k)
    {
      w = w + (2 * k + 1) * (generalizedFactorial(2 * m, k) / generalizedFactorial(2 * m + k + 1, k + 1)) * gramPoly(i, m, k, 0) * gramPoly(t, m, k, s);
    }
    return w;
  }


  template<unsigned long m>
  static Array<double, (2*m+1)> computeWeights(const int t, const int n, const int s)
  {
    Array<double, (2*m+1)> weights;

    for (unsigned long i = 0; i < 2 * m + 1; ++i)
    {
      weights[i] = computeWeight(i - m, t, m, n, s);
    }
    return weights;
  }
}

template<typename T, unsigned int m_elements, unsigned int n_order, int t_th_pos>
class SavitzkyGolayFilter
{
public:
    SavitzkyGolayFilter(double dt):
    dt(dt)
    {
      // Compute weights for the time window 2*m+1, for the t'th least-square point

      for (unsigned int s = 0; s < n_order; ++s) // of the s'th derivative
      {
        coefficients[s] = computeWeights<m_elements>(t_th_pos, n_order, s);
      }
    }

    void push(T val)
    {
      data.push_back(val);
    }

    T filter(unsigned int diff_order = 0)
    {
      T result = 0;
      Array<double, 2*m_elements + 1>& coeff = coefficients[diff_order];

      for (unsigned long i = 0; i < data.size(); ++i)
      {
        result += coeff[i] * data[i];
      }

      return result / pow(dt, diff_order);
    }

private:
    const double dt;

    Array<Array<double, 2*m_elements + 1>, n_order - 1> coefficients;
    RingBuffer<2*m_elements+1, T> data;
};

#endif