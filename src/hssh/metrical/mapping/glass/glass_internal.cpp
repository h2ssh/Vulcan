/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <iostream>
/* Internal dependencies for glass.cpp
        knownVisible(cell)   //returns whether a cell is at known visible angle
        shouldSee(cell)      //returns whether a cell is unoccluded, scannable, knownvisible to better than can be explained by uncert
        isOverlapping        //used by shouldSee to figure out overlapping
        trackadd(ang,&lo,&hi,&valid,dir)//tries to expand the track defined by lo,hi,valid to include ang in the direction dir(+1=ccw,-1=cw)
        getTransparent       //returns the cells that are transparent(ie. closer than the laser observation in that direction). 
                             //  These are the cells that are normally decremened in a occ grid.
        preprocessLaserScan  //cleans up the laser scan before use by removing points out of bounds, with negative range, too low intensity, etc.
*/

/************************************************************/
//Function: resetState(s)
	//sets up the grid to the size requested by s
void resetState(Size s){
	//Images
	belief.create(s)	;      //the amount that we believe that a cell is occupied right now
	flips.create(s)	    ;      //the number of times a cell has changed from believed occupied to believed unoccupied(will be usefull for static occupancy calculation)
	seen.create(s)	    ;      //the number of times we have seen a cell
	miss.create(s)	    ;      //the number of times we have failed to see a cell from a known visible angle
	transp.create(s)	;      //the number of times a cell has been transparent(we saw something farther away)
	
	phivis=sector(s);       //the lowest and highest angles we have seen the cell from
	phiseen=sector(s);      //the recent angles where we have seen the cell
	phimiss=sector(s);      //the range of angles from which we have not recently seen this cell and could(should?) have seen it
	
	//Sets
	active.clear();    //the cells that are actively needing processing this time around, kept updated to corespond to cells where belief >0

	//Size
	size=s;					// the current grid size.
	
	//Scratch work
	label = Mat_<uint8_t>(s);   //the internal labels for all cells on the current scan.
	
	
}





/************************************************************/
//Function:bool isOverlapping(m,v,u)
    //determines whether two sectors are definitely overlapping, even taking uncertainty into account
    //Inputs:
    //m=sector 1 defined by a lo and hi angle pair and a valid to say if the sector is nonempty
    //v=sector 2
    //u=uncertainty in sector end angle positions, assumed to be the same for all end angles, assumed > 0
    //sectors are considered to contain their endpoints
    //uncertainty is applied only to half of the endpoints, because only relative uncertainty matters, 
    // so behaviour is correct where endpoint angles are +-u/2

bool isOverlapping(double& mlo,double& mhi,bool& mvalid,double& vlo,double& vhi,bool& vvalid,double u){
    //short circuit if either sector is empty
    if (!(mvalid&&vvalid)) return false;
    //We first check for a basic collision, meaning that m and v overlap even without taking uncertainty into account.
    //If there is no basic overlap then of course the sectors are not definitely overlapping, so we return false.
    //Iff no basic overlap then (mlo-vlo>vhi-vlo) && (vlo-mlo>mhi-mlo)  as prooved in proof 1
    //We use >= because the uncertainty is assumed to be >0 so exact hits will be ok
    if (C(mlo-vlo)>=C(vhi-vlo) && C(vlo-mlo)>=C(mhi-mlo)) return false;
    
    //Try to move the offending parts out upward. We move by u because this is the same as moving 
    //  the offending part by u/2 and the appropriate other angle by u/2. 
    //Trying to move downward just results in the sector increasing size. 
    //If we can move bottom upward to get out the top part will also be able to move that much, so no need to check.
    //There is a problem if both sectors contain each other and only one can escape, so we check both cases always.

    if ( C(vlo-mlo)>=C(mhi-mlo)||                                       //vlo fine, still need to check mlo
        (C(vlo-mlo) < C(mhi-mlo) && C(vlo-mlo)+u > C(mhi-mlo)) ){       //vlo inside m but can be moved out
         if( C(mlo-vlo) >= C(vhi-vlo)||                                     //mlo fine, vlo must have been problem
             (C(mlo-vlo) < C(vhi-vlo) && C(vlo-mlo)+u > C(mhi-mlo)) ){      //mlo inside v but can be moved out
              return false;//all overlaps solved
         }
    }
    
    //Couldn't solve the overlaps
    return true;
}
bool isOverlapping(sector & m,sector & v,int cell,double u){
	return isOverlapping( m.lo(0,cell),m.hi(0,cell),m.valid(0,cell), v.lo(0,cell), v.hi(0,cell), v.valid(0,cell),u);
}

/************************************************************/
    
//Function :trackadd()
//tries to expand the sector defined by lo,hi,valid to include ang in the direction dir(+1=ccw,-1=cw)
void trackadd(const double ang,double &lo,double &hi,bool &valid){
	int dir=(angdiff(ang , C(hi-lo)/2+lo)>0) ? 1:-1;//hack because angTo is not always precise enough
	
	
    if (!valid || dir>0 && C(ang-lo)>C(hi-lo)){//we expanded upward
        hi=ang;
    }
    if (!valid || dir<0 && C(hi-ang)>C(hi-lo)){//we expanded downward
        lo=ang;
    }
    valid=true;
}
void trackadd(const double ang,sector &m,int cell){
	trackadd(ang,m.lo(0,cell),m.hi(0,cell),m.valid(0,cell));
}

/************************************************************/
    
//Function :calculateLabels()
// Figures out what cells were transparent(seen through), seen, unscannable/occluded(saw something closer), and unsure(caused by quantization error)
// I use the naive algorithm here, where I simply make elliptic arcs instead of proper polylines. Thus, I don't get the unsure regions quite right, and it's probably slow, we'll see.
//Algorithm:
//1. Everything starts unscannable[0].
//2. Draw filled polygon(ellipses) representing area swept by scan, marking transparent[1].(NaNs count as 0 range here)
//3. Draw thick polyline(ellipse arcs) corresponding to points seen, mark unsure[2].
//4. Draw points seen, mark seen[3]. 
//
//unsure cells should never be messed with
void calculateLabels()
{
    label=0;//uint8 image
    int n=laser.range.size();
	double br=BEAM_WIDTH/2;//hack, seems to be off due to rounding in opencv, so inflate
	CvPoint pt=cvPoint(bot.x,bot.y);
	
	Point poly[n];

	for(int i=0; i<n;i++){
		double x=laser.x[i];
		double y=laser.y[i];
		if(std::isnan(laser.x[i])/*a NaN*/){//NaN count as 0 range
			x=bot.x;
			y=bot.y;
		}
		if(!isfinite(laser.x[i])){//should really create a long empty space, but I'm lazy, so 0
			x=bot.x;
			y=bot.y;
		}
		poly[i]=Point(x,y);

	}
	const Point* tmp[1]= {poly};
	
	fillPoly(label, tmp, &n, 1, Scalar(1));
	polylines(label, tmp, &n, 1/*1 contour*/, 0/*not closed*/, Scalar(2)/*color*/,2/*thick*/);
//    // start by converting the laser points to the equivalent cvPoints
//    for(int i=0; i<n;i++){
//		
//		double r=laser.range[i]/.05;
//		if(std::isnan(laser.x[i])/*a NaN*/){//NaN count as 0 range
//			r=0;
//			continue;
//		}
//		if(!isfinite(laser.x[i])){
//			r=32767;//really big r, but small enough to safely square
//		}
////		std::cout<<"x "<<pt.x<<"y"<<pt.y<<endl;
////		std::cout<<"ang "<<laser.ang[i]<<endl;
////		std::cout<<laser.x[i]<<"range "<<r<<endl;		
//		//hack, br seems to be off due to rounding in opencv, so inflate
//		ellipse( label, pt, Size(r,r), laser.ang[i]*180/M_PI, -br*180/M_PI*5, br*180/M_PI*5, Scalar(1), -1);//mark the transparent region
//	}
//	for(int i=0; i<n;i++){
//		
//		double r=laser.range[i]/.05;
//		if(std::isnan(laser.x[i])/*a NaN*/){//NaN count as 0 range
//			r=0;
//			continue;
//		}
//		if(!isfinite(laser.x[i])){
//			r=32767;//really big r, but small enough to safely square
//		}	
//		ellipse( label, pt, Size(r,r), laser.ang[i]*180/M_PI, -br*180/M_PI, br*180/M_PI, Scalar(2), 2);//mark the unsure region
//    }
//	for(int i=0; i<n;i++){
//		
//		double r=laser.range[i]/.05;
//		if(std::isnan(laser.x[i])/*a NaN*/){//NaN count as 0 range
//			r=0;
//			continue;
//		}
//		if(!isfinite(laser.x[i])){
//			r=32767;//really big r, but small enough to safely square
//		}	
//		ellipse( label, pt, Size(r,r), laser.ang[i]*180/M_PI, -br*180/M_PI, br*180/M_PI, Scalar(3), 1);//mark the seen region
//		label(laser.y[i],laser.x[i])=4;//debug
//	}
	
		ellipse( label, pt, Size(0,0), 0, 0, 0, Scalar(2), 2);//clean up the origin(will have the unusable points from NaNs,etc.

	
}

//Function: shouldSee(cell)
//checks that a cell is not occluded or unscannable
bool shouldSee(int cell){
	if (C(angTo(cell)-laser.lo)>C(laser.hi-laser.lo)) {
		return 0;
	}
	return isOverlapping(phivis,phimiss,cell,laser.uncert);
}



/************************************************************/

//PROOFS
// PROOF 1 of (mlo-vlo>vhi-vlo) && (vlo-mlo>mhi-mlo) iff no basic overlap
    //We take it as obvious that if two sectors overlap then at least one endpoint of one sector lies inside or on the boundary of the other sector
    //Note that the formula for "angle a is outside [vlo,vhi]" in a circle is:
    //  a-vlo > vhi-vlo                                                         (mod 2pi)
    //Thus the formula for "mlo outside [vlo,vhi] and vlo outside [mlo,mhi]" is:
    //  (mlo-vlo>vhi-vlo) && (vlo-mlo>mhi-mlo)                                  (mod 2pi)
    //It turns out that this also implies "mhi outside [vlo,vhi] and vhi outside [mlo,mhi]" because: 
    //  (a-b)%c = c-(b-a)%c, so:
    //      (vlo-mlo)%2pi = 2pi-(mlo-vlo)%2pi
    //      so:
    //          mhi-mlo<vlo-mlo (mod 2pi) -->  (mhi-mlo)%2pi<2pi-(mlo-vlo)%2pi --> (mlo-vlo)%2pi+(mhi-mlo)%2pi<2pi -->
    //              0<(mlo-vlo)%2pi+(mhi-mlo)%2pi<2pi [lemma 1]
    //  0>a+b<c --> a+b%c=a+b so:
    //      0<(mlo-vlo)%2pi+(mhi-mlo)%2pi<2pi [lemma 1] --> (mlo-vlo)%2pi+(mhi-mlo)%2pi = (mhi-mlo+mlo-vlo)%2pi [lemma 2]
    //  (mlo-vlo)%2pi+(mhi-mlo)%2pi>(mlo-vlo)%2pi and (mlo-vlo)%2pi>(vhi-vlo)%2pi so:
    //      (mlo-vlo)%2pi+(mhi-mlo)%2pi>(vhi-vlo)%2pi[lemma 3]
    //
    //  (mlo-vlo)%2pi+(mhi-mlo)%2pi = (mhi-mlo+mlo-vlo)%2pi [lemma 2] and (mlo-vlo)%2pi+(mhi-mlo)%2pi>(vhi-vlo)%2pi [lemma 3]so:
    //      (mhi-mlo+mlo-vlo)%2pi>(vhi-vlo)%2pi 
    //      i.e. mhi-vlo>vhi-vlo (mod 2pi) 
    //      so "mhi outside [vlo,vhi]" [lemma 4]
    //  A similar reasoning proves "vhi outside [mlo,mhi]"
