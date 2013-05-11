#ifndef _PID_GEOMETRY_H_
#define _PID_GEOMETRY_H_

#include "../types.h"

namespace lunabotics {
	
class PIDGeometry {
	private:
		Pose _currentPose;
		Point _referencePoint;
		bool _referencePointIsValid;
		PointArr _trajectory;
		bool _closestTrajectoryPointIsValid;
		Point _closestTrajectoryPoint;
		Point _closestTrajectoryPointInLocalFrame;
		Point _referencePointInLocalFrame;
		double _referenceDistance;
		bool _localFrameIsValid;
		double _linearVelocity;
		float _velocityOffset;
		float _velocityMultiplier;
		double getReferenceAngle(Point point1, Point point2, Point point3); //Angle at point1 
		double getReferenceAngle(double edge1, double edge2, double edge3); //Angle between edge1 and edge2
		void updateLocalFrame();
		void invalidateCache();
	public:
		PIDGeometry();
		PIDGeometry(float velocityOffset, float velocityMultiplier);
		~PIDGeometry();
		Point getReferencePoint();
		Point getReferencePointInLocalFrame();
		Point getClosestTrajectoryPoint();
		Point getClosestTrajectoryPointInLocalFrame();
		double getReferenceDistance();
		void setCurrentPose(Pose currentPose);
		void setLinearVelocity(double velocity);
		void setTrajectory(PointArr trajectory);
		void setVelocityOffset(float velocityOffset);
		void setVelocityMultiplier(float velocityMultiplier);
};

typedef PIDGeometry * PIDGeometryPtr;
}

#endif //_PIDGeometry_H_
