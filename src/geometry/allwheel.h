#ifndef _ALL_WHEEL_H_
#define _ALL_WHEEL_H_

#include "../types.h"

#define GEOMETRY_OUTER_ANGLE_MAX	M_PI_2		
#define GEOMETRY_INNER_ANGLE_MAX	M_PI/180*64

namespace lunabotics {
	class AllWheelGeometry {
		private:
			Point lf;
			Point lr;
			Point rf;
			Point rr;
			float _wheel_offset;
			float _wheel_radius;
			float _wheel_width;
			
		
		public:
			bool geometryAcquired;
			AllWheelGeometry(Point left_front, Point left_rear, Point right_front, Point right_rear);
			AllWheelGeometry(AllWheelGeometry *copy);
			~AllWheelGeometry();
			bool calculateAngles(Point ICR, float &left_front, float &right_front, float &left_rear, float &right_rear);
			bool calculateVelocities(Point ICR, float center_velocity, float &left_front, float &right_front, float &left_rear, float &right_rear);
			void set_left_front(Point new_point);
			void set_left_rear(Point new_point);
			void set_right_front(Point new_point);
			void set_right_rear(Point new_point);
			void set_wheel_offset(float new_offset);
			void set_wheel_radius(float new_radius);
			void set_wheel_width(float new_width);
			Point left_front();
			Point left_rear();
			Point right_front();
			Point right_rear();
			Point point_outside_base_link(Point ICR);
			float wheel_offset();
			float wheel_radius();
			float wheel_width();
			float maxAvailableCurvature();
			
	};
	
	typedef AllWheelGeometry *AllWheelGeometryPtr;
	bool validateAngles(float &left_front, float &right_front, float &left_rear, float &right_rear);
}


#endif
