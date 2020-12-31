/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "vision/distortion.h"
#include <iostream>
#include <opencv2/opencv.hpp>


using namespace vulcan;


void create_camera_matrix(const vision::camera_calibration_t& calibration, cv::Mat& matrix);
void create_coeffs_matrix(const vision::camera_calibration_t& calibration, cv::Mat& matrix);


std::istream& vulcan::vision::operator>>(std::istream& in, camera_calibration_t& calibration)
{
    in >> calibration.focalLengths[0] >> calibration.focalLengths[1] >> calibration.principalPoints[0]
      >> calibration.principalPoints[1] >> calibration.skew >> calibration.distortionCoeffs[0]
      >> calibration.distortionCoeffs[1] >> calibration.distortionCoeffs[2] >> calibration.distortionCoeffs[3];

    return in;
}


std::ostream& vulcan::vision::operator<<(std::ostream& out, camera_calibration_t& calibration)
{
    out << calibration.focalLengths[0] << ' ' << calibration.focalLengths[1] << '\n'
        << calibration.principalPoints[0] << ' ' << calibration.principalPoints[1] << '\n'
        << calibration.skew << '\n'
        << calibration.distortionCoeffs[0] << ' ' << calibration.distortionCoeffs[1] << ' '
        << calibration.distortionCoeffs[2] << ' ' << calibration.distortionCoeffs[3] << '\n';

    return out;
}


void vulcan::vision::undistort(const Image& distorted, const camera_calibration_t& calibration, Image& undistorted)
{
    cv::Mat camera;
    cv::Mat coeffs;

    create_camera_matrix(calibration, camera);
    create_coeffs_matrix(calibration, coeffs);

    // TODO: Fill in the blanks
}


void vulcan::vision::undistort(const std::vector<Point<int16_t>>& distorted,
                               const camera_calibration_t& calibration,
                               std::vector<Point<int16_t>>& undistorted)
{
    cv::Mat camera;
    cv::Mat coeffs;

    create_camera_matrix(calibration, camera);
    create_coeffs_matrix(calibration, coeffs);

    cv::Mat distortedPoints(distorted.size(), 1, CV_32FC2);
    std::vector<cv::Point2f> undistortedPoints(distorted.size());

    for (size_t n = 0, end = distorted.size(); n < end; ++n) {
        distortedPoints.at<cv::Vec2f>(n, 0)[0] = distorted[n].x;
        distortedPoints.at<cv::Vec2f>(n, 0)[1] = distorted[n].y;
    }

    undistortPoints(distortedPoints, undistortedPoints, camera, coeffs);

    undistorted.resize(distorted.size());

    for (size_t n = 0, end = undistorted.size(); n < end; ++n) {
        undistorted[n].x = (undistortedPoints[n].x * calibration.focalLengths[0]) + calibration.principalPoints[0];
        undistorted[n].y = (undistortedPoints[n].y * calibration.focalLengths[1]) + calibration.principalPoints[1];
    }
}


void create_camera_matrix(const vision::camera_calibration_t& calibration, cv::Mat& matrix)
{
    matrix.create(3, 3, CV_64FC1);
    matrix.zeros(3, 3, CV_64FC1);

    matrix.at<double>(0, 0) = calibration.focalLengths[0];
    matrix.at<double>(1, 1) = calibration.focalLengths[1];

    matrix.at<double>(0, 2) = calibration.principalPoints[0];
    matrix.at<double>(1, 2) = calibration.principalPoints[1];

    matrix.at<double>(2, 2) = 1;
}


void create_coeffs_matrix(const vision::camera_calibration_t& calibration, cv::Mat& matrix)
{
    matrix.create(4, 1, CV_64FC1);
    matrix.zeros(4, 1, CV_64FC1);

    matrix.at<double>(0, 0) = calibration.distortionCoeffs[0];
    matrix.at<double>(1, 0) = calibration.distortionCoeffs[1];
    matrix.at<double>(2, 0) = calibration.distortionCoeffs[2];
    matrix.at<double>(3, 0) = calibration.distortionCoeffs[3];
}
