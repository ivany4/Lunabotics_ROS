#ifndef _ALL_WHEEL_H_
#define _ALL_WHEEL_H_

#include "../types.h"

#define GEOMETRY_OUTER_ANGLE_MAX	M_PI_2		
#define GEOMETRY_INNER_ANGLE_MAX	M_PI/180*65

namespace lunabotics {
namespace geometry {
	class AllWheelGeometry {
		private:
			point_t lf;
			point_t lr;
			point_t rf;
			point_t rr;
			float _wheel_offset;
			float _wheel_radius;
			float _wheel_width;
		
		public:
			AllWheelGeometry(point_t left_front, point_t left_rear, point_t right_front, point_t right_rear);
			AllWheelGeometry(AllWheelGeometry *copy);
			~AllWheelGeometry();
			bool calculateAngles(point_t ICR, float &left_front, float &right_front, float &left_rear, float &right_rear);
			bool calculateVelocities(point_t ICR, float center_velocity, float &left_front, float &right_front, float &left_rear, float &right_rear);
			void set_left_front(point_t new_point);
			void set_left_rear(point_t new_point);
			void set_right_front(point_t new_point);
			void set_right_rear(point_t new_point);
			void set_wheel_offset(float new_offset);
			void set_wheel_radius(float new_radius);
			void set_wheel_width(float new_width);
			point_t left_front();
			point_t left_rear();
			point_t right_front();
			point_t right_rear();
			float wheel_offset();
			float wheel_radius();
			float wheel_width();
			
	};
	
	typedef AllWheelGeometry *AllWheelGeometryPtr;
	bool validateAngles(float &left_front, float &right_front, float &left_rear, float &right_rear);
}
}


#endif
