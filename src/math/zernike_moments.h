/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     zernike_moments.h
 * \author   Collin Johnson
 *
 * Definition of ZernikeMoments class template.
 */

#ifndef MATH_ZERNIKE_MOMENTS_H
#define MATH_ZERNIKE_MOMENTS_H

#include "core/point.h"
#include "math/geometry/rectangle.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <cstring>

namespace vulcan
{
namespace math
{


/**
 * ZernikeMoments is an implementation of the Zernike moments for describing a shape using the pseudocode from:
 *
 *   "Algorithms for fast computation of Zernike moments and their numerical stability"
 *       Chandan Singh and Ekta Walia [1]
 *
 * The use of Zernike moments for shape classification is described in:
 *
 *   "Rotation invariant Image Recognition using Features Selected Via A Systematic Method"
 *       Alireza Khotanzad and Yaw Hua Hong  [2]
 *
 * The Zernike moments are complex. However, [2] notes that the moments are rotation invariant with respect to their
 * magnitude. Thus, the moments returned here are the magnitude of the Zernike moments. Because only the magnitude is
 * calculated,
 *
 * The fast method implemented caches many intermediate calculations. If the moments are going to be calculated for many
 * shapes, then the instance of ZernikeMoments should be saved between updates.
 *
 */
template <int N>
class ZernikeMoments
{
public:
    /**
     * Constructor for ZernikeMoments.
     */
    ZernikeMoments(void);

    /**
     * moments
     *
     * \param    begin           Start of the points in the shape
     * \param    end             End of the points in the shape
     * \param    center          Center of the points -- for translation invariance
     * \param    boundary        Bounding rectangle for the points
     * \return   Zernike moments up to order [0, N].
     */
    template <typename T>
    std::vector<double> moments(typename std::vector<Point<T>>::const_iterator begin,
                                typename std::vector<Point<T>>::const_iterator end,
                                const Point<double>& center,
                                double length) const;

    /**
     * numMoments retrieves the number of moments that will be calculated for the value of N.
     */
    std::size_t numMoments(void) const;

private:
    using HArray = int[N + 1][N + 1];
    using CacheArray = std::array<double, N + 1>;

    // Order-dependent, data-independent cached values
    HArray h1_;
    HArray h2_;
    HArray h3_;

    void computeRTable(double radius, CacheArray& rN) const;
    void computeCosAndSinTable(double x, double y, double radius, CacheArray& cosT, CacheArray& sinT) const;
};


template <int N>
ZernikeMoments<N>::ZernikeMoments(void)
{
    assert(N >= 0);

    // Calculate the H-values -- Eq (22)
    for (int n = 0; n <= N; ++n) {
        for (int m = n - 2; m >= 0; m -= 2) {
            h3_[n][m] = -(4 * (m + 2) * (m + 1)) / ((n + m + 2) * (n - m));
            h2_[n][m] = (h3_[n][n] * (n + m + 4) * (n - m - 2)) / (4 * (m + 3)) + (m + 2);
            h1_[n][m] =
              (((m + 4) * (m + 3)) / 2) - ((m + 4) * h2_[n][m]) + (h3_[n][m] * ((n + m + 6) * (n - m - 4) / 8));
        }
    }
}


template <int N>
template <typename T>
std::vector<double> ZernikeMoments<N>::moments(typename std::vector<Point<T>>::const_iterator begin,
                                               typename std::vector<Point<T>>::const_iterator end,
                                               const Point<double>& center,
                                               double length) const
{
    // Implementing the pseudo-code from Figure 3
    const double kLength = length;
    const double kDiameter = kLength * std::sqrt(2.0);
    //     const double kDiameter = 2.0 * std::max(distance_between_points(center, boundary.bottomLeft),
    //                                             distance_between_points(center, boundary.bottomRight));
    const double kArea = M_PI * kDiameter * kDiameter / 4.0;

    // Scale-normalization is used. This approach makes all shapes have the same m00, in this case
    // 800.0. The area of the shape is considered to be the number of cells it contains. However, all coordinates are
    // scaled by the diameter to put them in the range [-1, 1], therefore the area needs to be divided by diameter^2
    // to ensure proper scaling
    const double kScaleFactor = 10000.0;
    // Dividing denominator by D^2 = multiplying numerator by D^2
    const double kAreaScale = std::sqrt(kScaleFactor * kDiameter * kDiameter / std::distance(begin, end));

    const double kPointNormalizer = 1.0 / (kDiameter * kAreaScale);
    // If the scale factor is greater than the diameter
    //     assert(kScaleFactor > kDiameter/2);

    CacheArray rN;
    CacheArray cosT;
    CacheArray sinT;

    const int kMDimensions = N + 1;
    double mReal[kMDimensions][kMDimensions];
    double mImag[kMDimensions][kMDimensions];
    std::memset(&mReal, 0, kMDimensions * kMDimensions * sizeof(double));
    std::memset(&mImag, 0, kMDimensions * kMDimensions * sizeof(double));

    // Incrementally calculate the moments
    for (auto pointIt = begin; pointIt != end; ++pointIt) {
        // Scale the point coordinates so they are in the range [-1, 1] by shifting them to be centered at the
        // center of mass of the points and then dividing by the diameter of the bounding rectangle, which ensures
        // that they are in the correct range, regardless of how skewed the shape is.
        auto normalized =
          Point<double>((pointIt->x - center.x) * kPointNormalizer, (pointIt->y - center.y) * kPointNormalizer);
        auto pointRadius = std::sqrt(normalized.x * normalized.x + normalized.y * normalized.y);

        if (pointRadius > 1.0) {
            std::cout << "Point:" << *pointIt << " Center:" << center
                      << " Dist to center:" << distance_between_points(*pointIt, center) << " Normalized:" << normalized
                      << " Radius: " << pointRadius << " Diameter:" << kDiameter << " Scale:" << kAreaScale << '\n';
        }
        //         std::cout << "Point:" << *pointIt << " Center:" << center << " Dist to center:"
        //             << distance_between_points(*pointIt, center) << " Boundary:" << boundary
        //             << " Normalized:" << normalized << " Radius: " << pointRadius << " Diameter:" << kDiameter
        //             << " Scale:" << kAreaScale << " Contains:" << boundary.contains(*pointIt) << '\n';
        assert(pointRadius <= 1.0);

        // If the radius is 0, it doesn't contribute to the calculation and should be skipped
        if (pointRadius == 0.0) {
            continue;
        }

        computeRTable(pointRadius, rN);
        computeCosAndSinTable(normalized.x, normalized.y, pointRadius, cosT, sinT);

        // Intermediate values for the radial polynomial, R_nm, R_n,m+2, R_n,m+4
        double r_nm = 0.0, r_nmp2 = 0.0, r_nmp4 = 0.0;
        double oneOverRad = 1.0 / (pointRadius * pointRadius);

        for (int n = 0; n <= N; ++n) {
            double normalizer = (n + 1) / kArea;

            for (int m = n; m >= 0; m -= 2) {
                if (m == n) {
                    r_nm = rN[n];
                    r_nmp4 = rN[n];
                } else if (m == n - 2) {
                    r_nm = (n * rN[n]) - ((n - 1) * rN[n - 2]);
                    r_nmp2 = r_nm;
                } else {
                    r_nm = (h1_[n][m] * r_nmp4) + (h2_[n][m] + (h3_[n][m] * oneOverRad)) / r_nmp2;
                    r_nmp4 = r_nmp2;
                    r_nmp2 = r_nm;
                }

                mReal[n][m] += normalizer * r_nm * cosT[m];
                mImag[n][m] -= normalizer * r_nm * sinT[m];
            }
        }
    }

    // Calculate the magnitude for all valid moments
    std::vector<double> magnitudes;
    magnitudes.reserve(numMoments());
    for (int n = 0; n <= N; ++n) {
        // The moments are defined wherever n + m = even. When odd, start 1, otherwise start at even and jump along
        // by two to keep things even the whole time
        for (int m = (n % 2 == 0) ? 0 : 1; m <= n; m += 2) {
            double magSq = (mReal[n][m] * mReal[n][m]) + (mImag[n][m] * mImag[n][m]);
            magnitudes.push_back(magSq);
        }
    }

    std::transform(magnitudes.begin(), magnitudes.end(), magnitudes.begin(), ::sqrt);

    return magnitudes;
}


template <int N>
std::size_t ZernikeMoments<N>::numMoments(void) const
{
    /*
     * The number of moments for a given N can be inferred looking at the first few values of N.
     *
     *   0 : 1   (00)
     *   1 : 1   (11)
     *   2 : 2   (20, 22)
     *   3 : 2   (31, 33)
     *   4 : 3   (40, 42, 44)
     *   5 : 3   (51, 53, 55)
     *   6 : 4   (60, 62, 64, 66)
     *     etc.
     *
     * Thus, for a given n, there are  (n / 2) + 1 moments, where the division is integer division. The total can be
     * viewed as two acending sequences for the even and odd numbers, where the maximum number of moments in the
     * sequence is M = (N / 2) + 1. There are also (N / 2) + 1 even numbers in the sequence (0 counts). Thus, there are
     * (M * (M+1)) / 2 even moments and odd moments. The total moments is M * (M+1) if N is odd and M*(M+1)/2 +
     * M*(M-1)/2 if N is even.
     */

    constexpr int m = (N / 2) + 1;
    if (N % 2 == 0) {
        return ((m * (m + 1)) / 2) + ((m * (m - 1)) / 2);
    } else {
        return m * (m + 1);
    }
}


template <int N>
void ZernikeMoments<N>::computeRTable(double radius, CacheArray& rN) const
{
    rN[0] = radius;
    for (int n = 1; n <= N; ++n) {
        rN[n] = rN[n - 1] * radius;
    }
}


template <int N>
void ZernikeMoments<N>::computeCosAndSinTable(double x, double y, double radius, CacheArray& cosT, CacheArray& sinT)
  const
{
    double a = x / radius;
    double b = y / radius;

    cosT[0] = a;
    sinT[0] = b;

    for (int n = 1; n <= N; ++n) {
        cosT[n] = a * cosT[n - 1] - b * sinT[n - 1];
        sinT[n] = a * sinT[n - 1] + b * cosT[n - 1];
    }
}

}   // namespace math
}   // namespace vulcan

#endif   // MATH_ZERNIKE_MOMENTS_H
