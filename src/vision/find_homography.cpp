/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <cassert>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <utils/command_line.h>
#include <core/point.h>
#include <vision/distortion.h>
#include <vision/homography.h>
#include <homest.h>


using namespace vulcan;


const std::string HELP_LONG("help");
const std::string HELP_SHORT("h");
const std::string POINTS_FILE("points-file");
const std::string CALIBRATION_FILE("calibration-file");
const std::string OUTPUT_FILE("output-file");


struct homography_points_t
{
    size_t numPoints;
    
    std::vector<Point<int16_t>> pixels;
    std::vector<Point<float>>   world;
};

void display_help_if_needed(const utils::CommandLine& commandLine);

homography_points_t          load_homography_points_file(const std::string& filename);
vision::camera_calibration_t load_camera_calibration(const std::string& filename);

vision::homography_matrix_t calculate_homography_matrix(const homography_points_t& rawPoints);
void calculate_world_to_camera_homography(const homography_points_t& points, vision::homography_matrix_t& matrix);
void calculate_camera_to_world_homography(const homography_points_t& points, vision::homography_matrix_t& matrix);

void show_test_data_conversions(const homography_points_t& points, const vision::homography_matrix_t& matrix);

void save_matrix_to_file(const vision::homography_matrix_t& matrix, const std::string& filename);


/**
* find_homography uses the homest library to find the homography matrix between world and image coordinates. The
* points used for the calculation are loaded from a .txt file containing the following values on each row:
* 
*   image_x image_y world_x world_y
* 
* The calculated homography_matrix_t is saved to the specified output file, which should then be loaded by
* programs that need use of homographies.
* 
* The command-line arguments for find_homography are:
*   
*   -h/--help                       Display help message
*   --points-file      'filename'   File containing the raw points to use for the calculation
*   --calibration-file 'filename'   File containing the camera calibration parameters
*   --output-file      'filename'   File in which to store the matrix
*/
int main(int argc, char** argv)
{
    utils::CommandLine commandLine(argc, argv);
    
    display_help_if_needed(commandLine);
    
    homography_points_t          rawPoints   = load_homography_points_file(commandLine.argumentValue(POINTS_FILE));
    vision::camera_calibration_t calibration = load_camera_calibration(commandLine.argumentValue(CALIBRATION_FILE));
    
    // the homography has to be calculated with undistorted points because you need a plane->plane conversion
    vision::undistort(rawPoints.pixels, calibration, rawPoints.pixels);
    
    vision::homography_matrix_t  matrix = calculate_homography_matrix(rawPoints);
    
    show_test_data_conversions(rawPoints, matrix);
    
    save_matrix_to_file(matrix, commandLine.argumentValue(OUTPUT_FILE));
    
    return 0;
}


void display_help_if_needed(const utils::CommandLine& commandLine)
{
    bool needHelp = commandLine.argumentExists(HELP_SHORT)        ||
                    commandLine.argumentExists(HELP_LONG)         ||
                    !commandLine.argumentExists(POINTS_FILE)      ||
                    !commandLine.argumentExists(CALIBRATION_FILE) ||
                    !commandLine.argumentExists(OUTPUT_FILE);
                    
    if(needHelp)
    {
        std::cout<<"The command-line arguments for find_homography are:\n"
                 <<'\n'
                 <<"   -h/--help                       Display help message\n"
                 <<"   --points-file      'filename'   File containing the raw points to use for the calculation\n"
                 <<"   --calibration-file 'filename'   File containing the camera calibration parameters\n"
                 <<"   --output-file      'filename'   File in which to store the matrix\n"
                 <<std::endl;
                 
        exit(1);
    }
}


homography_points_t load_homography_points_file(const std::string& filename)
{
    const float TILE_WIDTH = 0.3048;
    
    std::ifstream in(filename.c_str());
    
    if(!in.is_open())
    {
        std::cerr<<"ERROR: Unable to open "<<filename<<" to load raw homography points"<<std::endl;
        assert(in.is_open());
    }
    
    homography_points_t homographyPoints;
    
    homographyPoints.numPoints = 0;
    
    std::cout<<"Loading image->world sample points:\n";
    
    Point<int16_t> pixel;
    Point<float>   world;
    
    while(!in.eof())
    {
        in>>pixel.x>>pixel.y>>world.x>>world.y;
        
        std::cout<<pixel<<"->"<<world<<'\n';
        
        world.x *= TILE_WIDTH;
        world.y *= TILE_WIDTH;
        
        homographyPoints.pixels.push_back(pixel);
        homographyPoints.world.push_back(world);
        
        ++homographyPoints.numPoints;
    }
    
    std::cout<<"Finished loading sample points.\n";
    
    return homographyPoints;
}


vision::camera_calibration_t load_camera_calibration(const std::string& filename)
{
    std::ifstream in(filename.c_str());
    
    if(!in.is_open())
    {
        std::cerr<<"ERROR: Unable to open "<<filename<<" to load camera calibration values"<<std::endl;
        assert(in.is_open());
    }
    
    vision::camera_calibration_t calibration;
    
    in>>calibration;
    
    std::cout<<"Camera calibration parameters:\n"<<calibration<<'\n';
    
    return calibration;
}


vision::homography_matrix_t calculate_homography_matrix(const homography_points_t& rawPoints)
{
    vision::homography_matrix_t matrix;
    
    calculate_world_to_camera_homography(rawPoints, matrix);
    calculate_camera_to_world_homography(rawPoints, matrix);
    
    return matrix;
}


void calculate_world_to_camera_homography(const homography_points_t& points, vision::homography_matrix_t& matrix)
{
    const double INL_PCENT = 0.7;
    
    double homography[9];
    int    numOutliers;
    
    double (*imagePoints)[2];
    double (*worldPoints)[2];
    // This slightly terrifying line comes from homest_demo.c. Not sure how I'd do the equivalent in C++.
    imagePoints=(double (*)[2])malloc(points.numPoints*sizeof(double[2]));
    worldPoints=(double (*)[2])malloc(points.numPoints*sizeof(double[2]));
    
    for(size_t n = 0; n < points.numPoints; ++n)
    {
        imagePoints[n][0] = points.pixels[n].x;
        imagePoints[n][1] = points.pixels[n].y;
        
        worldPoints[n][0] = points.world[n].x;
        worldPoints[n][1] = points.world[n].y;
        
        std::cout<<"("<<imagePoints[n][0]<<','<<imagePoints[n][1]<<")->("<<worldPoints[n][0]<<','<<worldPoints[n][1]<<")\n";
    }
    
    // NOTE: This call comes from homest_demo.c. Not sure how the different refine functions diff
    homest(worldPoints, imagePoints, points.numPoints, INL_PCENT, homography, true, HOMEST_SYM_XFER_ERROR, 0, &numOutliers, 1);
    
    matrix.wi00 = homography[0];
    matrix.wi01 = homography[1];
    matrix.wi02 = homography[2];
    matrix.wi10 = homography[3];
    matrix.wi11 = homography[4];
    matrix.wi12 = homography[5];
    matrix.wi20 = homography[6];
    matrix.wi21 = homography[7];
    matrix.wi22 = homography[8];
    
    std::cout<<"World->Camera homography matrix:\n"
             <<matrix.wi00<<' '<<matrix.wi01<<' '<<matrix.wi02<<'\n'
             <<matrix.wi10<<' '<<matrix.wi11<<' '<<matrix.wi12<<'\n'
             <<matrix.wi20<<' '<<matrix.wi21<<' '<<matrix.wi22<<'\n';
}


void calculate_camera_to_world_homography(const homography_points_t& points, vision::homography_matrix_t& matrix)
{
    const double INL_PCENT = 0.7;
    
    double homography[9];
    int    numOutliers = 0;
    
    double (*imagePoints)[2];
    double (*worldPoints)[2];
    // This slightly terrifying line comes from homest_demo.c. Not sure how I'd do the equivalent in C++.
    imagePoints=(double (*)[2])malloc(points.numPoints*sizeof(double[2]));
    worldPoints=(double (*)[2])malloc(points.numPoints*sizeof(double[2]));
    
    for(size_t n = 0; n < points.numPoints; ++n)
    {
        imagePoints[n][0] = points.pixels[n].x;
        imagePoints[n][1] = points.pixels[n].y;
        
        worldPoints[n][0] = points.world[n].x;
        worldPoints[n][1] = points.world[n].y;
    }
    
    // NOTE: This call comes from homest_demo.c. Not sure how the different refine functions diff
    homest(imagePoints, worldPoints, points.numPoints, INL_PCENT, homography, true, HOMEST_SYM_XFER_ERROR, 0, &numOutliers, 1);
    
    matrix.iw00 = homography[0];
    matrix.iw01 = homography[1];
    matrix.iw02 = homography[2];
    matrix.iw10 = homography[3];
    matrix.iw11 = homography[4];
    matrix.iw12 = homography[5];
    matrix.iw20 = homography[6];
    matrix.iw21 = homography[7];
    matrix.iw22 = homography[8];
    
    std::cout<<"Camera->World homography matrix:\n"
             <<matrix.iw00<<' '<<matrix.iw01<<' '<<matrix.iw02<<'\n'
             <<matrix.iw10<<' '<<matrix.iw11<<' '<<matrix.iw12<<'\n'
             <<matrix.iw20<<' '<<matrix.iw21<<' '<<matrix.iw22<<'\n';
}


void show_test_data_conversions(const homography_points_t& points, const vision::homography_matrix_t& matrix)
{
    // Idea is to do a conversion of the test points to the other frame to get a feel for how bad or good the conversion is
    std::cout<<"Test: Convert image->world for the sample points:\n";
    
    Point<float> converted;
    
    for(size_t n = 0; n < points.numPoints; ++n)
    {
        converted = vision::image_to_world_coordinates(points.pixels[n], matrix);
        
        std::cout<<"Image: "<<points.pixels[n]<<" World:"<<points.world[n]<<" Homography:"<<converted<<'\n';
    }
    
    std::cout<<"\nTest: Convert world->image for the sample points:\n";
    
    for(size_t n = 0; n < points.numPoints; ++n)
    {
        converted = vision::world_to_image_coordinates(points.world[n], matrix);
        
        std::cout<<"World: "<<points.world[n]<<" Image:"<<points.pixels[n]<<" Homography:"<<converted<<'\n';
    }
}


void save_matrix_to_file(const vision::homography_matrix_t& matrix, const std::string& filename)
{
    std::ofstream out(filename.c_str());
    
    if(!out.is_open())
    {
        std::cerr<<"ERROR: Unable to open "<<filename<<" to save calculate homography matrix"<<std::endl;
        assert(out.is_open());
    }
    
    out<<matrix;
}
