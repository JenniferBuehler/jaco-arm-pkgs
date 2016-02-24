#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   Copyright (C) 2015 Jennifer Buehler

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
*/
#endif


#include <boost/thread/mutex.hpp>
#include <jaco_gazebo/JointController.h>
#include <gazebo/physics/Model.hh>

#include <map>
#include <string>

using gazebo::physics::ModelPtr;
using gazebo::physics::JointPtr;
// using gazebo::physics::JointController;
using gazebo::physics::JointControllerThreadsafe;
using gazebo::common::PID;
using gazebo::common::Time;

JointControllerThreadsafe::JointControllerThreadsafe(ModelPtr& _parent, JointControllerThreadsafe::JointControllerImplPtr& _jointController)
    : Base(boost::dynamic_pointer_cast<Base>(_parent)),
      jointController(_jointController)
{
    Base::SetName(UniqueName());
}

JointControllerThreadsafe::~JointControllerThreadsafe()
{
}

void JointControllerThreadsafe::AddJoint(JointPtr _joint)
{
    boost::unique_lock<boost::recursive_mutex> l(mtx);
    jointController->AddJoint(_joint);
}

void JointControllerThreadsafe::Update()
{
    boost::unique_lock<boost::recursive_mutex> l(mtx);
    jointController->Update();
}

void JointControllerThreadsafe::Reset()
{
//    std::cout<<"Reset JointController"<<std::endl;
    boost::unique_lock<boost::recursive_mutex> l(mtx);
    jointController->Reset();
}

void JointControllerThreadsafe::SetJointPosition(const std::string &_name, double _position, int _index)
{
    boost::unique_lock<boost::recursive_mutex> l(mtx);
    jointController->SetJointPosition(_name, _position, _index);
}

void JointControllerThreadsafe::SetJointPositions(const std::map<std::string, double> &_jointPositions)
{
    boost::unique_lock<boost::recursive_mutex> l(mtx);
    jointController->SetJointPositions(_jointPositions);
}


gazebo::common::Time JointControllerThreadsafe::GetLastUpdateTime() const
{
    boost::unique_lock<boost::recursive_mutex> l(mtx);
    return jointController->GetLastUpdateTime();
}

std::map<std::string, JointPtr> JointControllerThreadsafe::GetJoints() const
{
    boost::unique_lock<boost::recursive_mutex> l(mtx);
    return jointController->GetJoints();
}

void JointControllerThreadsafe::SetPositionPID(const std::string &_jointName, const common::PID &_pid)
{
    boost::unique_lock<boost::recursive_mutex> l(mtx);
    jointController->SetPositionPID(_jointName, _pid);
}

bool JointControllerThreadsafe::SetPositionTarget(const std::string &_jointName, double _target)
{
    boost::unique_lock<boost::recursive_mutex> l(mtx);
    return jointController->SetPositionTarget(_jointName, _target);
}

void JointControllerThreadsafe::SetVelocityPID(const std::string &_jointName, const common::PID &_pid)
{
    boost::unique_lock<boost::recursive_mutex> l(mtx);
    jointController->SetVelocityPID(_jointName, _pid);
}

bool JointControllerThreadsafe::SetVelocityTarget(const std::string &_jointName, double _target)
{
    boost::unique_lock<boost::recursive_mutex> l(mtx);
    return jointController->SetVelocityTarget(_jointName, _target);
}

std::map<std::string, gazebo::common::PID> JointControllerThreadsafe::GetPositionPIDs() const
{
    boost::unique_lock<boost::recursive_mutex> l(mtx);
    return jointController->GetPositionPIDs();
}

std::map<std::string, gazebo::common::PID> JointControllerThreadsafe::GetVelocityPIDs() const
{
    boost::unique_lock<boost::recursive_mutex> l(mtx);
    return jointController->GetVelocityPIDs();
}

std::map<std::string, double> JointControllerThreadsafe::GetForces() const
{
    boost::unique_lock<boost::recursive_mutex> l(mtx);
    return jointController->GetForces();
}

std::map<std::string, double> JointControllerThreadsafe::GetPositions() const
{
    boost::unique_lock<boost::recursive_mutex> l(mtx);
    return jointController->GetPositions();
}

std::map<std::string, double> JointControllerThreadsafe::GetVelocities() const
{
    boost::unique_lock<boost::recursive_mutex> l(mtx);
    return jointController->GetVelocities();
}

void JointControllerThreadsafe::SetJointPosition(JointPtr _joint, double _position, int _index)
{
    boost::unique_lock<boost::recursive_mutex> l(mtx);
    jointController->SetJointPosition(_joint, _position, _index);
}
