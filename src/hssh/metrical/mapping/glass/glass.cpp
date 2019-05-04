/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "main.cpp"
#include "glass.h"
#include "glass_types.h"
#include "glass_state.cpp"
#include "glass_utils.cpp"
#include "glass_internal.cpp"


/*
Notes on this file/module
This program uses a blackboard model for all of its work.

Inputs (in glass_input.h):
laser.range		//the range to the target in meters
     .inten		//intensity of the return, >100 is reasonable intensity, >1000 is high confidence, >4000 is brighter than brightest white
     .ang       //the angle of the ray in world coordinates
     .lo		//lowest scan angle in the scan
	 .hi		//highest scan angle in the scan
     .uncert    //the uncertainty of the ray (not used right now, in theory should be)
     .x         //x in grid coords of ray tip (0,0 is upper left corner of upper left pixel)
     .y         //y in grid coords
     
bot.x        //x in grid coords
   .y        //y in grid coords
   .dir      //the direction of motion of the robot as an angle in world coords

gridsize				 //the size of the grid in pixels
     
State (in glass_state.h):
    Images:
        belief      //the amount that we believe that a cell is occupied right now
        flips       //the number of times a cell has changed from believed occupied to believed unoccupied(will be usefull for static occupancy calculation)
        seen        //the number of times we have seen a cell
        miss        //the number of times we have failed to see a cell from a known visible angle
        transp      //the number of times a cell has been transparent(we saw something farther away)
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
        active      //the cells that are actively needing processing this time around, kept updated to corespond to cells where belief >0
	Size:
		size		//the current size of the grid
	Temporary Images:
	 	label;       //the internal labels(transparent, unknown, unscannable, etc.) for all cells on the current scan.

Dependencies:
    Utility:
        angdiff(a,b)        //returns mod((a-b+pi),2pi)-pi , the smallest signed angular difference between a and b
        angTo(cell)			//returns the angle to a cell, given by its linear index, from the bot 
        C(a)                //returns the natural form of the angle, mod(a,2pi)
        mod(a,b)            //returns modulus, floor style(result same sign as divisor)
        
    Internal:
        knownVisible(cell)   //returns whether a cell is at known visible angle
        shouldSee(cell)      //returns whether a cell is unoccluded, scannable, knownvisible to better than can be explained by uncert
        isOverlapping        //used by shouldSee to figure out overlapping
        trackadd(ang,&lo,&hi,&valid,dir)//tries to expand the track defined by lo,hi,valid to include ang in the direction dir(+1=ccw,-1=cw)
        getTransparent       //returns the cells that are transparent(ie. closer than the laser observation in that direction). 
                             //  These are the cells that are normally decremened in a occ grid.
        calculateTransparent //Figures out what cells are seen 
        preprocessLaserScan  //cleans up the laser scan before use by removing points out of bounds, with negative range, too low intensity, etc.
        

        
        
        
The basic principles of the algorithm's behavior are:
    1. points are unknown until evidence otherwise
    2. any point seen right now is more likely than not to be occupied now (i.e.belief>0)
    3. if a cell has previously been occupied and is now not occupied, it requires multiple observations to be considered staticly occupied (this will probably not be explictly stated)
    4. not seeing a cell that has never been seen is evidence that the cell is free
    5. not seeing a cell that has been seen is only evidence for freeness if it is at an angle where we should see it

*/

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>




using namespace cv;  // The new C++ interface API is inside this namespace. Import it.
using namespace std;
//Function: updateFromLaserScan(laser)
void updateFromLaserScan(LaserScan& laser,Size& gridsize)
{
	static int run=0;
	cout<<"run: "<<run<<endl;
	run++;

	if (gridsize!=size) {
		resetState(gridsize);
		cout<<"Setting size."<<endl;
	}
	int ncell=size.width*size.height;//number of cells
	//update the buffer for our understanding of the world
	calculateLabels();
	
    //update state from laser ray information
    //  
    for (int n=0;n<laser.range.size();n++){
		if(std::isnan(laser.x[n])||!isfinite(laser.x[n])){//laser ray is invalid, don't use it
			continue;
		}
		
        int cell=&label((int)laser.y[n],(int)laser.x[n])-&label(0,0);//calculate linear index of cell
		
        //reward cells that were hit
        belief(0,cell)=(belief(0,cell)>0)?belief(0,cell)+1 : 1;//make sure belief is positive then add 1
        active.insert(cell);
        seen(0,cell)++;
        //update phiseen, phivis, phimiss. Might also want to update the neighbors here
        phimiss.valid(0,cell)=0;//not missing any more
        trackadd(laser.ang[n],phiseen,cell);//expand the seen sector

        if (C(phiseen.hi(0,cell)-phiseen.lo(0,cell))>C(phivis.hi(0,cell)-phivis.lo(0,cell))||!phivis.valid(0,cell)){//phiseen is now wider than phivis, replace
            phivis.hi(0,cell)=phiseen.hi(0,cell);
            phivis.lo(0,cell)=phiseen.lo(0,cell);
            phivis.valid(0,cell)=1;
        }
    }
    
	//debug
	if (run%100==0){
		//debug display
		/*flip(label,label,0);
		flip(belief,belief,0);
		flip(transp,transp,0);
		flip(phiseen.lo,phiseen.lo,0);
		flip(phiseen.hi,phiseen.hi,0);*/
		imshow("label Matrix",label*80);
		imshow("main",belief*1000+32767);
		imshow("transp",transp*255);
		imshow("seenlo",phiseen.lo/M_PI/2+.5);
		imshow("seenhi",phiseen.hi/M_PI/2+.5);
		imshow("diff",(phiseen.hi-phiseen.lo)/M_PI/2+.5);
		/*flip(label,label,0);
		flip(belief,belief,0);
		flip(transp,transp,0);
		flip(phiseen.lo,phiseen.lo,0);
		flip(phiseen.hi,phiseen.hi,0);*/
		cvWaitKey(1);
	}
	

	
    //take away from cells that were transparent (seen through)
	// The cells that we definitely should have seen but didn't are a subset of the transparent cells 
    
    for (int cell=0;cell<ncell;cell++){
		if(label(0,cell)!=1){
		continue;
		}

		//update phi's to represent a miss
        phiseen.valid(0,cell)=0;
        //add to number of times cell has been seen through
        transp(0,cell)++;
        //update the range of angles where we haven't seen the cell. could probably save some processing if we only update for active cells, but for debug better here
        trackadd(angTo(cell), phimiss, cell);
        
		//deal with cells we already thought were empty
        if (!phivis.valid(0,cell)){
            belief(0,cell)-=1;//a cell being missed that we don't believe is occupied is evidence against it, since it is more likely to be empty than any sort of obstacle
        }
		//cell was believed occupied, only mess with if we should definitely have seen it
		else{
            if (shouldSee(cell)){
			
                belief(0,cell)-=10;
                miss(0,cell)++;
                //deal with the cells that have deactivated (we no longer believe are occupied)
                if (belief(0,cell)<=0){
                    active.erase(cell);
                    phivis.valid(0,cell)=0;
                    flips(0,cell)++;
                }
            }
        } 
    }



    //deal with the cells we couldn't have seen
    for (auto cell: active ){
        if (label(0,cell)==1/*unscannable or occluded*/){
            phiseen.valid(0,cell)=0;
            phimiss.valid(0,cell)=0;
        }
    }

}




    


