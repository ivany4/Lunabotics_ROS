/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "physics/physics.hh"
#include "transport/transport.hh"
#include "DiffDrivePlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(DiffDrivePlugin)

enum {RIGHT, LEFT};

/////////////////////////////////////////////////
DiffDrivePlugin::DiffDrivePlugin()
{
  this->wheelSpeed[LEFT] = this->wheelSpeed[RIGHT] = 0;
}

/////////////////////////////////////////////////
void DiffDrivePlugin::Load(physics::ModelPtr _model,
                           sdf::ElementPtr _sdf)
{
  this->model = _model;

  // Start up ROS
  int argc = 0;
  std::string name1 = "luna_gazebo_diffdrive"; // this is what appears in the rostopics
  ros::init(argc, NULL, name1);
  this->nodeHandle = ros::NodeHandle("~");

  this->velocitySubscriber = this->nodeHandle.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, &DiffDrivePlugin::ctrlCallback, this);
  

  if (!_sdf->HasElement("left_joint"))
    gzerr << "DiffDrive plugin missing <left_joint> element\n";

  if (!_sdf->HasElement("right_joint"))
    gzerr << "DiffDrive plugin missing <right_joint> element\n";

  this->leftJoint = _model->GetJoint(
      _sdf->GetElement("left_joint")->GetValueString());
  this->rightJoint = _model->GetJoint(
      _sdf->GetElement("right_joint")->GetValueString());

  if (_sdf->HasElement("torque"))
    this->torque = _sdf->GetElement("torque")->GetValueDouble();
  else
  {
    gzwarn << "No torque value set for the DiffDrive plugin.\n";
    this->torque = 5.0;
  }

  if (!this->leftJoint)
    gzerr << "Unable to find left joint["
          << _sdf->GetElement("left_joint")->GetValueString() << "]\n";
  if (!this->rightJoint)
    gzerr << "Unable to find right joint["
          << _sdf->GetElement("right_joint")->GetValueString() << "]\n";

  this->wheelSeparation = this->leftJoint->GetAnchor(0).Distance(
      this->rightJoint->GetAnchor(0));

  physics::EntityPtr parent = boost::shared_dynamic_cast<physics::Entity>(
      this->leftJoint->GetChild());

  math::Box bb = parent->GetBoundingBox();
  // This assumes that the largest dimension of the wheel is the diameter
  this->wheelRadius = bb.GetSize().GetMax() * 0.5;


  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&DiffDrivePlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void DiffDrivePlugin::ctrlCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  double vr, va;

  vr = msg->linear.x;
  va = msg->angular.z;

  this->wheelSpeed[LEFT] = vr + va * this->wheelSeparation / 2.0;
  this->wheelSpeed[RIGHT] = vr - va * this->wheelSeparation / 2.0;
}

/////////////////////////////////////////////////
void DiffDrivePlugin::OnUpdate()
{
  /* double d1, d2;
  double dr, da;

  this->prevUpdateTime = currTime; 

  // Distance travelled by front wheels
  d1 = stepTime.Double() * this->wheelRadius * this->leftJoint->GetVelocity(0);
  d2 = stepTime.Double() * this->wheelRadius * this->rightJoint->GetVelocity(0);

  dr = (d1 + d2) / 2;
  da = (d1 - d2) / this->wheelSeparation;
  common::Time currTime = this->model->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;
  */

  double leftVelDesired = (this->wheelSpeed[LEFT] / this->wheelRadius);
  double rightVelDesired = (this->wheelSpeed[RIGHT] / this->wheelRadius);

  this->leftJoint->SetVelocity(0, leftVelDesired);
  this->rightJoint->SetVelocity(0, rightVelDesired);

  this->leftJoint->SetMaxForce(0, this->torque);
  this->rightJoint->SetMaxForce(0, this->torque);
}
