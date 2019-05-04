/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     savitzky_golay.h
* \author   Collin Johnson
* 
* Definition of functions implementing Savitzky-Golay filtering for smoothing data or calculating smoothed derivatives:
* 
*   - savitzky_golay_smooth       : smooth using Savitzky-Golay
*   - savitzky_golay_first_deriv  : smoothed first derivative of the data
*   - savitzky_golay_second_deriv : smoothed second derivative of the data
*/

#include <math/savitzky_golay.h>
#include <array>
#include <iostream>
#include <iterator>
#include <numeric>
#include <cassert>

namespace vulcan
{
namespace math
{
    
template <int N>
using Coeffs = std::array<double, N>;
    
    
template <int Window>
void savitzky_golay_filter(DataConstIter         begin, 
                           DataConstIter         end, 
                           DataIter              output, 
                           const Coeffs<Window>& coefficients,
                           double                normalizer);
    

void savitzky_golay_smooth(DataConstIter  begin, 
                           DataConstIter  end, 
                           DataIter       output, 
                           SGWindowRadius radius,
                           SGPolynomial   polynomial)
{
    static Coeffs<5> kRadTwoQuadCoeffs    = { -3, 12, 17, 12, -3 };
    static Coeffs<7> kRadThreeQuadCoeffs  = { -2, 3, 6, 7, 6, 3, -2 };
    static Coeffs<9> kRadFourQuadCoeffs   = { -21, 14, 39, 54, 59, 54, 39, 14, -31};
    static Coeffs<7> kRadThreeQuartCoeffs = { 5, -30, 75, 131, 75, -30, 5 };
    static Coeffs<9> kRadFourQuartCoeffs  = { 15 -55, 30, 135, 179, 135, 30, -55, 15 };
    
    const double kRadTwoQuadNormalizer    = 35;
    const double kRadThreeQuadNormalizer  = 21;
    const double kRadFourQuadNormalizer   = 231;
    const double kRadThreeQuartNormalizer = 231;
    const double kRadFourQuartNormalizer  = 429;
    
    if((radius == SGWindowRadius::two) && (polynomial == SGPolynomial::quadratic))
    {
        savitzky_golay_filter<5>(begin, end, output, kRadTwoQuadCoeffs, kRadTwoQuadNormalizer);
    }
    else if((radius == SGWindowRadius::three) && (polynomial == SGPolynomial::quadratic))
    {
        savitzky_golay_filter<7>(begin, end, output, kRadThreeQuadCoeffs, kRadThreeQuadNormalizer);
    }
    else if((radius == SGWindowRadius::four) && (polynomial == SGPolynomial::quadratic))
    {
        savitzky_golay_filter<9>(begin, end, output, kRadFourQuadCoeffs, kRadFourQuadNormalizer);
    }
    else if((radius == SGWindowRadius::two) && (polynomial == SGPolynomial::quartic))
    {
        std::cerr << "WARNING: savitzky_golay_smooth: Radius = 2, Polynomial = quartic is not supported.\n";
    }
    else if((radius == SGWindowRadius::three) && (polynomial == SGPolynomial::quartic))
    {
        savitzky_golay_filter<7>(begin, end, output, kRadThreeQuartCoeffs, kRadThreeQuartNormalizer);
    }
    else if((radius == SGWindowRadius::four) && (polynomial == SGPolynomial::quartic))
    {
        savitzky_golay_filter<9>(begin, end, output, kRadFourQuartCoeffs, kRadFourQuartNormalizer);
    }
}


void savitzky_golay_first_deriv(DataConstIter  begin, 
                                DataConstIter  end, 
                                DataIter       output, 
                                SGWindowRadius radius,
                                SGPolynomial   polynomial)
{
    static Coeffs<5> kRadTwoQuadCoeffs    = { -2, -1, 0, 1, 2 };
    static Coeffs<7> kRadThreeQuadCoeffs  = { -3, -2, -1, 0, 1, 2, 3 };
    static Coeffs<9> kRadFourQuadCoeffs   = { -4, -3, -2, -1, 0, 1, 2, 3, 4 };
    static Coeffs<5> kRadTwoQuartCoeffs   = { 1, -8, 0, 8, -1 };
    static Coeffs<7> kRadThreeQuartCoeffs = { 22, -67, -58, 0, 58, 67, -22 };
    static Coeffs<9> kRadFourQuartCoeffs  = { 86, -142, -193, -126, 0, 126, 193, 142, -86 };
    
    const double kRadTwoQuadNormalizer    = 10;
    const double kRadThreeQuadNormalizer  = 28;
    const double kRadFourQuadNormalizer   = 60;
    const double kRadTwoQuartNormalizer   = 12;
    const double kRadThreeQuartNormalizer = 252;
    const double kRadFourQuartNormalizer  = 1188;
    
    if((radius == SGWindowRadius::two) && (polynomial == SGPolynomial::quadratic))
    {
        savitzky_golay_filter<5>(begin, end, output, kRadTwoQuadCoeffs, kRadTwoQuadNormalizer);
    }
    else if((radius == SGWindowRadius::three) && (polynomial == SGPolynomial::quadratic))
    {
        savitzky_golay_filter<7>(begin, end, output, kRadThreeQuadCoeffs, kRadThreeQuadNormalizer);
    }
    else if((radius == SGWindowRadius::four) && (polynomial == SGPolynomial::quadratic))
    {
        savitzky_golay_filter<9>(begin, end, output, kRadFourQuadCoeffs, kRadFourQuadNormalizer);
    }
    else if((radius == SGWindowRadius::two) && (polynomial == SGPolynomial::quartic))
    {
        savitzky_golay_filter<5>(begin, end, output, kRadTwoQuartCoeffs, kRadTwoQuartNormalizer);
    }
    else if((radius == SGWindowRadius::three) && (polynomial == SGPolynomial::quartic))
    {
        savitzky_golay_filter<7>(begin, end, output, kRadThreeQuartCoeffs, kRadThreeQuartNormalizer);
    }
    else if((radius == SGWindowRadius::four) && (polynomial == SGPolynomial::quartic))
    {
        savitzky_golay_filter<9>(begin, end, output, kRadFourQuartCoeffs, kRadFourQuartNormalizer);
    }
}


void savitzky_golay_second_deriv(DataConstIter  begin, 
                                 DataConstIter  end, 
                                 DataIter       output, 
                                 SGWindowRadius radius,
                                 SGPolynomial   polynomial)
{
    static Coeffs<5> kRadTwoQuadCoeffs    = { 2, -1, -2, -1, 2 };
    static Coeffs<7> kRadThreeQuadCoeffs  = { 5, 0, -3, -4, -3, 0, 5 };
    static Coeffs<9> kRadFourQuadCoeffs   = { 28, 7, -8, -17, -20, -17, -8, 7, 28 };
    static Coeffs<5> kRadTwoQuartCoeffs   = { -1, 16, -30, 16, -1 };
    static Coeffs<7> kRadThreeQuartCoeffs = { -13, 67, -19, -70, -19, 67, -13 };
    static Coeffs<9> kRadFourQuartCoeffs  = { -126, 371, 151, -211, -370, -211, 151, 371, -126 };
    
    const double kRadTwoQuadNormalizer    = 7;
    const double kRadThreeQuadNormalizer  = 42;
    const double kRadFourQuadNormalizer   = 462;
    const double kRadTwoQuartNormalizer   = 12;
    const double kRadThreeQuartNormalizer = 132;
    const double kRadFourQuartNormalizer  = 1716;
    
    if((radius == SGWindowRadius::two) && (polynomial == SGPolynomial::quadratic))
    {
        savitzky_golay_filter<5>(begin, end, output, kRadTwoQuadCoeffs, kRadTwoQuadNormalizer);
    }
    else if((radius == SGWindowRadius::three) && (polynomial == SGPolynomial::quadratic))
    {
        savitzky_golay_filter<7>(begin, end, output, kRadThreeQuadCoeffs, kRadThreeQuadNormalizer);
    }
    else if((radius == SGWindowRadius::four) && (polynomial == SGPolynomial::quadratic))
    {
        savitzky_golay_filter<9>(begin, end, output, kRadFourQuadCoeffs, kRadFourQuadNormalizer);
    }
    else if((radius == SGWindowRadius::two) && (polynomial == SGPolynomial::quartic))
    {
        savitzky_golay_filter<5>(begin, end, output, kRadTwoQuartCoeffs, kRadTwoQuartNormalizer);
    }
    else if((radius == SGWindowRadius::three) && (polynomial == SGPolynomial::quartic))
    {
        savitzky_golay_filter<7>(begin, end, output, kRadThreeQuartCoeffs, kRadThreeQuartNormalizer);
    }
    else if((radius == SGWindowRadius::four) && (polynomial == SGPolynomial::quartic))
    {
        savitzky_golay_filter<9>(begin, end, output, kRadFourQuartCoeffs, kRadFourQuartNormalizer);
    }
}


template <int Window>
void savitzky_golay_filter(DataConstIter         begin, 
                           DataConstIter         end, 
                           DataIter              output, 
                           const Coeffs<Window>& coefficients,
                           double                normalizer)
{
    /*
    * To apply the Savitzky-Golay filter, a full window is needed. At the edges of the data, where the window doesn't
    * fit, the data is wrapped around so a full window can be used. The data used will then look like:
    * 
    *   . . . d_2 d_1 d_0 d_1 d_2 . . . d_n d_n-1 d_n-2 . . .
    * 
    * The special case for the beginning and ending elements are handled first and last.
    */
    
    const int kRadius = Window / 2;
    const int kDataSize = std::distance(begin, end);
    
    normalizer = 1.0 / normalizer;
    
    assert(kDataSize >= Window);
    
    // Handle the first [0, kRadius) elements
    for(int n = 0; n < kRadius; ++n)
    {
        // The wraparound elements run in reverse: [wraparound, 0)
        double value = 0.0;
        int wraparound = kRadius - n;
        value = std::inner_product(std::reverse_iterator<DataConstIter>(begin + wraparound + 1), 
                                   std::reverse_iterator<DataConstIter>(begin + 1),
                                   coefficients.begin(),
                                   value);
        // At this point |wraparound| have been processed. Now need to finish the remaining values in the window,
        // which consists of all values from [0, n+radius+1) because the filter is centered at n.
        *output++ = std::inner_product(begin, begin + n + kRadius + 1, coefficients.begin() + wraparound, value) *
            normalizer;
    }
    
    // Handle the elements in the range [kRadius, end-kRadius)
    for(int n = kRadius; n < kDataSize - kRadius; ++n)
    {
        *output++ = std::inner_product(begin + n - kRadius, begin + n + kRadius + 1, coefficients.begin(), 0.0) *
            normalizer;
    }
    
    // Handle the final [end-kRadius, end) elements
    for(int n = kDataSize - kRadius; n < kDataSize; ++n)
    {
        // The wraparound elements run in reverse: [wraparound, 0)
        int remaining = kDataSize - n - 1;
        int wraparound = kRadius - remaining;
        double value = 0.0;
        
        value = std::inner_product(begin + n - kRadius, end, coefficients.begin(), 0.0);
        *output++ = std::inner_product(std::reverse_iterator<DataConstIter>(end - 1),
                                       std::reverse_iterator<DataConstIter>(end - wraparound - 1),
                                       coefficients.begin() + kRadius + remaining + 1,
                                       value) * normalizer;
    }
}

} // namespace math
} // namespace vulcan
