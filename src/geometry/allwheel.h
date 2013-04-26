#ifndef _ALL_WHEEL_H_
#define _ALL_WHEEL_H_

#include "../types.h"

namespace geometry {
	class AllWheelGeometry {
		private:
			point_t lf;
			point_t lr;
			point_t rf;
			point_t rr;
		
		public:
			AllWheelGeometry(point_t left_front, point_t left_rear, point_t right_front, point_t right_rear);
			~AllWheelGeometry();
			bool calculateAngles(point_t ICR, float &left_front, float &right_front, float &left_rear, float &right_rear);
			void set_left_front(point_t new_point);
			void set_left_rear(point_t new_point);
			void set_right_front(point_t new_point);
			void set_right_rear(point_t new_point);
			point_t left_front();
			point_t left_rear();
			point_t right_front();
			point_t right_rear();
	};
	
	typedef AllWheelGeometry *AllWheelGeometryPtr;
}


#endif
