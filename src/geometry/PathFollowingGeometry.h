#ifndef _PATH_FOLLOWING_GEOMETRY_H_
#define _PATH_FOLLOWING_GEOMETRY_H_

#include "../types.h"
#include "allwheel.h"



namespace lunabotics {
	
#define MASS	80	//Mass of the vehicle
#define C_ALPHA_F	1	//Front cornering force
#define C_ALPHA_R	1	//Rear cornernig force

class PathFollowingGeometry {
	private:
	
		//Look-ahead points
		
		//Feedback
		Point feedback_point;
		Point feedback_point_in_local_frame;
		bool has_feedback_point;
		float feedback_point_offset_min;
		float feedback_point_offset_multiplier;
		float feedback_look_ahead_distance;
		bool has_feedback_look_ahead_distance;
		double feedback_error;
		bool has_feedback_error;
		Point feedback_path_point;
		Point feedback_path_point_in_local_frame;
		bool has_feedback_path_point;
	
	
		//Feedforward
		Point feedforward_point;
		Point feedforward_point_in_local_frame;
		bool has_feedforward_point;
		float feedforward_point_offset_fraction;
		float feedforward_curvature_detection_start;
		float feedforward_look_ahead_distance;
		bool has_feedforward_look_ahead_distance;
		double feedforward_prediction;
		bool has_feedforward_prediction;
		PointArr curvature_detection_points;
		PointArr curvature_detection_points_in_local_frame;
		bool has_curvature_detection_points;
		Point feedforward_center;
		bool has_feedforward_center;
		double curve_radius;
		bool has_curve_radius;
	
		Pose current_pose;
		PointArr path;
		
		bool has_local_frame;
		
		AllWheelGeometryPtr robotGeometry;
		
		double velocity;
		
		
		double getReferenceAngle(Point point1, Point point2, Point point3); //Angle at point1 
		double getReferenceAngle(double edge1, double edge2, double edge3); //Angle between edge1 and edge2
		void updateLocalFrame();
		void invalidateCache();
		bool getClosestPathPoint(Point referencePoint, Point &resultPoint);
		float getFeedbackLookAheadDistance();
		float getFeedforwardLookAheadDistance();
		Point getLookAheadPointAtDistance(double distance);
		Point localFramePoint(Point globalFramePoint);
	public:
		PathFollowingGeometry(AllWheelGeometryPtr geometry);
		PathFollowingGeometry(AllWheelGeometryPtr geometry, float feedback_offset, float feedback_multiplier, float feedforward_offset, float feedforward_fraction);
		~PathFollowingGeometry();
		Point getFeedbackPoint();
		Point getFeedbackPointInLocalFrame();
		Point getFeedforwardPoint();
		Point getFeedforwardPointInLocalFrame();
		Point getFeedbackPathPoint();
		Point getFeedbackPathPointInLocalFrame();
		Point getFeedforwardCurveCenterPoint();
		PointArr getCurvatureDetectionPoints();
		PointArr getCurvatureDetectionPointsInLocalFrame();
		double getCurveRadius();
		double getFeedbackError();
		double getFeedforwardPrediction();
		void setCurrentPose(Pose currentPose);
		void setVelocity(double velocity);
		void setPath(PointArr path);
		void setFeedbackPointOffsetMin(float velocityOffset);
		void setFeedbackPointOffsetMultiplier(float velocityMultiplier);
		void setFeedforwardPointOffsetFraction(float fraction);
		void setFeedforwardCurvatureDetectionStart(float velocityOffset);
};

typedef PathFollowingGeometry * PathFollowingGeometryPtr;
}

#endif //_PIDGeometry_H_
