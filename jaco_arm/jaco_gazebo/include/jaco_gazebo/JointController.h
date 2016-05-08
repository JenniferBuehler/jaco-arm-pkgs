#ifndef JACO_GAZEBO_JOINTCONTROLLER_H
#define JACO_GAZEBO_JOINTCONTROLLER_H
/**
   Adapter fror gazebo::physics::JointController which adds thread safety.

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

#include <string>
#include <vector>
#include <map>

#include <gazebo/physics/Base.hh>
#include <boost/thread/recursive_mutex.hpp>

#define GAZEBO_ADVANCED_JOINTCONTROLLER_VERSION 5
#include "gazebo/transport/Node.hh"  // somehow brings in GAZEBO_MAJOR_VERSION
#if GAZEBO_MAJOR_VERSION >= GAZEBO_ADVANCED_JOINTCONTROLLER_VERSION
#include <gazebo/physics/JointController.hh>
#else
#include <JointControllerNewImpl.h>
#endif

namespace gazebo
{
namespace physics
{


/**
 * \brief Adapter fror gazebo::physics::JointController which adds thread safety.
 *
 * Unfortunately, the original gazebo::physics::JointController is not threadsafe.
 * This is a wrapper around this class which simply forwards all calls to all the
 * JointController functions to the original JointController object and locks a mutex
 * around the call.
 *
 * This class derives from Base such that it can be attached to a gazebo::physics::Model
 * as a child. This way, the threadsafe controller object may be used by independent
 * plugins which operate on the same model.
 *
 * \author Jennifer Buehler
 * \date January 2016
 */
#if GAZEBO_MAJOR_VERSION >= GAZEBO_ADVANCED_JOINTCONTROLLER_VERSION
class GAZEBO_VISIBLE JointControllerThreadsafe: public gazebo::physics::Base
#else
class JointControllerThreadsafe: public gazebo::physics::Base
#endif  // GAZEBO_MAJOR_VERSION
{
public:
#if GAZEBO_MAJOR_VERSION >= GAZEBO_ADVANCED_JOINTCONTROLLER_VERSION
    typedef physics::JointController JointControllerImpl;
    typedef physics::JointControllerPtr JointControllerImplPtr;
#else
    typedef JointControllerNewImpl JointControllerImpl;
    typedef JointControllerNewImplPtr JointControllerImplPtr;
#endif
    /**
     *  \brief Constructor.
     * \param[in] _model Model that uses this joint controller.
     *                                     Must be the same model which JointController
     *                                     _jointController has as model.
     */
    explicit JointControllerThreadsafe(ModelPtr& _parent, JointControllerImplPtr& _jointController);
    virtual ~JointControllerThreadsafe();
    void AddJoint(JointPtr _joint);
    void Update();
    void Reset();
    void SetJointPosition(const std::string &_name, double _position, int _index = 0);
    void SetJointPositions(const std::map<std::string, double> &_jointPositions);
    common::Time GetLastUpdateTime() const;
    std::map<std::string, JointPtr> GetJoints() const;
    void SetPositionPID(const std::string &_jointName,
                        const common::PID &_pid);
    bool SetPositionTarget(const std::string &_jointName,
                           double _target);
    void SetVelocityPID(const std::string &_jointName,
                        const common::PID &_pid);
    bool SetVelocityTarget(const std::string &_jointName,
                           double _target);
    std::map<std::string, common::PID> GetPositionPIDs() const;
    std::map<std::string, common::PID> GetVelocityPIDs() const;
    std::map<std::string, double> GetForces() const;
    std::map<std::string, double> GetPositions() const;
    std::map<std::string, double> GetVelocities() const;
    void SetJointPosition(JointPtr _joint, double _position, int _index = 0);

    /**
     * This name will be used for ALL objects of this class which are added
     * as children to any Model. Each Model should only have ONE JointControllerThreadsafe
     * object, just as they have only one original JointController.
     */
    static std::string UniqueName()
    {
        return "JointControllerThreadsafe";
    }

    boost::unique_lock<boost::recursive_mutex> GetLock()
    {
        return boost::unique_lock<boost::recursive_mutex>(mtx);
    }
    void lock()
    {
        mtx.lock();
    }
    void unlock()
    {
        mtx.unlock();
    }
private:
    // name should not be changed from outside, as we are always going
    // to use the return value of UniqueName().
    virtual void SetName(const std::string &_name)
    {
        Base::SetName(UniqueName());
    }

    JointControllerImplPtr jointController;
    mutable boost::recursive_mutex mtx;
};


typedef boost::shared_ptr<JointControllerThreadsafe> JointControllerThreadsafePtr;

}  // namespace physics
}  // namespace gazebo
#endif  //  JACO_GAZEBO_JOINTCONTROLLER_H
