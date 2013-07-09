#include "GazeboUtils.h"
	
namespace gazebo
{   
bool findLinkByName(physics::ModelPtr model, physics::LinkPtr &link, const std::string name) {
	link = model->GetLink(name);
	if (!link) {
		gzerr << "link by name [" << name << "] not found in model\n";
		return false;
	}
	return true;
}

bool findJointByName(physics::ModelPtr model, physics::JointPtr &joint, const std::string name) {
	joint = model->GetJoint(name);
	if (!joint) {
		gzerr << "joint by name [" << name << "] not found in model\n";
		return false;
	}
	return true;
}
}
