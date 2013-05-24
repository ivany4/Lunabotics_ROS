#ifndef _DOUBLE_LOOK_AHEAD_H_
#define _DOUBLE_LOOK_AHEAD_H_

#include "../types.h"

namespace lunabotics {

class DoubleLookAhead {
	private:
		Point _center;
		double _delta_f;	//Steering angle of the bicycle model
		double _C_alpha_f;	//Front cornering force
		double _C_alpha_r;	//Rear cornernig force
		double _l1;		//Distance from the CG to the front axis
		double _l2;		//Distance from the CG to the rear axis
		double _m1;		//Mas of the vehicle
		double _dxu;	//Longitudal velocity in the local frame
	
	public:
	
		
	
		//Feed forward methods
		double radiusFromPoints(PointArr points); //Used with 5 points (middle is fastforward look-ahead point)
		double steeringAngleFromCurvature(double k);
		void guessCurveCenter(PointArr points); //Used with 3 points (middle is fastforward look-ahead point)


};

}

#endif
