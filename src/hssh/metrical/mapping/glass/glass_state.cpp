/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/*State:
    Images:
        belief      //the amount that we believe that a cell is occupied right now
        flips       //the number of times a cell has changed from believed occupied to believed unoccupied(will be
   usefull for static occupancy calculation) seen        //the number of times we have seen a cell miss        //the
   number of times we have failed to see a cell from a known visible angle transp      //the number of times a cell has
   been transparent(we saw something farther away)

        phivis.lo   //the widest angular range we have seen the cell from
              .hi
              .valid
        phiseen.lo  //the recent angles where we have seen the cell
               .hi
               .valid
        phimiss.lo  //the range of angles from which we have not recently seen this cell and could(should?) have seen it
               .hi
               .valid
    Sets:
        active       //the cells that are actively needing processing this time around, kept updated to corespond to
   cells where belief >0 Sizes: size		 //the current size of the grid Temporary Images: label;       //the
   internal labels(transparent, unknown, unscannable, etc.) for all cells on the current scan.
        */
#include <set>
using namespace std;
using namespace cv;

// Images
Mat_<int> belief;   // the amount that we believe that a cell is occupied right now
Mat_<int> flips;    // the number of times a cell has changed from believed occupied to believed unoccupied(will be
                   // useful for static occupancy calculation)
Mat_<int> seen;     // the number of times we have seen a cell
Mat_<int> miss;     // the number of times we have failed to see a cell from a known visible angle
Mat_<int> transp;   // the number of times a cell has been transparent(we saw something farther away)

sector phivis;    // the lowest and highest angles we have seen the cell from
sector phiseen;   // the recent angles where we have seen the cell
sector phimiss;   // the range of angles from which we have not recently seen this cell and could(should?) have seen it

// Sets
set<int> active;   // the cells that are actively needing processing this time around, kept updated to corespond to
                   // cells where belief >0

// Size
Size size = Size(0, 0);   // the current grid size.

// Scratch work
Mat_<uint8_t> label;   // the internal labels for all cells on the current scan.
