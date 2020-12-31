/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#define TWO_PI (2 * M_PI)   // 6.28318530718
#include <math.h>
/*
Utilities:
    Modular Arithmetic:
        angsub(a,b)         //returns mod((a-b+pi),2pi)-pi , the smallest signed angular difference between a and b
        C(a)                //returns the natural form of the angle, mod(a,2pi)
        mod(a,b)            //returns modulus, floor style(result same sign as divisor)
    sign(x)					//returns the whether x is >,=,or < 0 as +1,0,-1

    angTo(cell)				//returns the angle to a cell from the bot given by its linear index

*/
using namespace std;
using namespace cv;

inline double mod(double a, double b);
inline double C(double in);

inline double mod(double a, double b)
{
    return a < 0 ? fmod(a, b) + b : fmod(a, b);
}

inline double C(double in)
{
    return mod(in, TWO_PI);
}

inline double angdiff(double a, double b)
{
    return mod(a - b + M_PI, TWO_PI) - M_PI;
}

Point ind2sub(const Mat& m, int ind)
{
    return Point(ind % m.cols, ind / m.cols);   // x=ind%cols, y=ind/cols
}
Point ind2sub(Size s, int ind)
{
    return Point(ind % s.width, ind / s.width);   // x=ind%cols, y=ind/cols
}

double angTo(int ind)
{
    Point cell = ind2sub(size, ind);
    Point bottmp = Point(bot.x, bot.y);
    Point diff = cell - bottmp;
    return atan2(diff.y, diff.x);
}

int sign(double x)
{
    return (x > 0) - (x < 0);
}
