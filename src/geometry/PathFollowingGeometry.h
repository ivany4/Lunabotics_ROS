#ifndef _PATH_FOLLOWING_GEOMETRY_H_
#define _PATH_FOLLOWING_GEOMETRY_H_

#include "../types.h"

namespace lunabotics {
	
class PathFollowingGeometry {
	private:
		Pose current_pose;
		Point feedback_point;
		bool has_feedback_point;
		PointArr path;
		bool has_closest_path_point;
		Point closest_path_point;
		Point closest_path_point_in_local_frame;
		Point feedback_point_in_local_frame;
		double feedback_error;
		bool has_local_frame;
		double velocity;
		float feedback_point_offset_min;
		float feedback_point_offset_multiplier;
		double getReferenceAngle(Point point1, Point point2, Point point3); //Angle at point1 
		double getReferenceAngle(double edge1, double edge2, double edge3); //Angle between edge1 and edge2
		void updateLocalFrame();
		void invalidateCache();
	public:
		PathFollowingGeometry();
		PathFollowingGeometry(float velocityOffset, float velocityMultiplier);
		~PathFollowingGeometry();
		Point getFeedbackLookAheadPoint();
		Point getFeedbackLookAheadPointLocalFrame();
		Point getClosestPathPoint();
		Point getClosestPathPointInLocalFrame();
		double getFeedbackError();
		void setCurrentPose(Pose currentPose);
		void setVelocity(double velocity);
		void setPath(PointArr path);
		void setFeedbackPointOffsetMin(float velocityOffset);
		void setFeedbackPointOffsetMultiplier(float velocityMultiplier);
};

typedef PathFollowingGeometry * PathFollowingGeometryPtr;
}

#endif //_PIDGeometry_H_
