#ifndef _GAZEBO_UTILS_H_
#define _GAZEBO_UTILS_H_

#include <physics/physics.hh>

namespace gazebo
{   
bool findLinkByName(physics::ModelPtr model, physics::LinkPtr &link, const std::string name);
bool findJointByName(physics::ModelPtr model, physics::JointPtr &joint, const std::string name);
}
#endif
