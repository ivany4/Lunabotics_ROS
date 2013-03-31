#ifndef _PID_H_
#define _PID_H_

#include "../types.h"

namespace motion {
	
	class PIDGeometry {
		private:
			pose_t _currentPose;
			point_t _referencePoint;
			bool _referencePointIsValid;
			point_arr _trajectory;
			bool _closestTrajectoryPointIsValid;
			point_t _closestTrajectoryPoint;
			point_t _closestTrajectoryPointInLocalFrame;
			point_t _referencePointInLocalFrame;
			double _referenceDistance;
			bool _localFrameIsValid;
			double _linearVelocity;
			float _velocityOffset;
			float _velocityMultiplier;
			double getReferenceAngle(point_t point1, point_t point2, point_t point3); //Angle at point1 
			double getReferenceAngle(double edge1, double edge2, double edge3); //Angle between edge1 and edge2
			void updateLocalFrame();
			void invalidateCache();
		public:
			PIDGeometry();
			PIDGeometry(float velocityOffset, float velocityMultiplier);
			~PIDGeometry();
			point_t getReferencePoint();
			point_t getReferencePointInLocalFrame();
			point_t getClosestTrajectoryPoint();
			point_t getClosestTrajectoryPointInLocalFrame();
			double getReferenceDistance();
			void setCurrentPose(pose_t currentPose);
			void setLinearVelocity(double velocity);
			void setTrajectory(point_arr trajectory);
			void setVelocityOffset(float velocityOffset);
			void setVelocityMultiplier(float velocityMultiplier);
	};
	
	double point_distance(point_t p1, point_t p2);
}

#endif //_PID_H_
