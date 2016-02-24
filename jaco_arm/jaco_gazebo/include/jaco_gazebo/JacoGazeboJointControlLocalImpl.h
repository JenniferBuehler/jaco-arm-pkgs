#ifndef JACO_GAZEBO_JACOGAZEBOJOINTCONTROLLOCALIMPL_H
#define JACO_GAZEBO_JACOGAZEBOJOINTCONTROLLOCALIMPL_H
/**
   Implementation of JacoGazeboJoinControl which bypasses gazebo::physics::JointController::Update() call.

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

#include <map>
#include <jaco_gazebo/JacoGazeboJointControl.h>


namespace gazebo
{


/**
 * \brief Implementation of JacoGazeboJointControl which bypasses gazebo::physics::JointController::Update() call to
 * address limitations described in the base class.
 *
 * 1. This class can maintain position and velocity controllers at the same time.
 *    Which means, it accepts both velocity and position commands. It is even possible to send a mix of velocity and
 *    position commands for all the jaco joints.
 *
 * 2. The control of the joints may also happen via gazebo::physics::Joint::SetVelocity() instead of
 *    gazebo::physics::Joint::SetForce(). This may be useful for some Gazebo versions which don't work well
 *    with SetForce() on the Jaco arm. To use Joint::SetVelocity() instead of Joint::SetForce(), set the ROS parameter
 *    jaco/gazebo_use_set_velocity. Note that this requires loading different PID values via the ROS parameters as well.
 *
 * **To 1.)**
 *
 * Joints will be updated either by
 *
 * 1. force or
 * 2. velocity
 * 3. position
 *
 * in the given order of priority!
 * Specifying higher-priority values will overwrite lower-priority values.
 * For example, if both position and velocity commmands are specified for the same joint,
 * the velocity will overwrite the position command. There is only one exception to this:
 * If the velocity is 0, it will *not* overwrite the position command, because the
 * position command will then be required (see also documentation of base class!).
 *
 * **To 2.)**
 *
 * It is still on the TODO list to support the use of gazebo::physics::SetForce() instead of setting the
 * velocity. This should be relatively easy, but it will most likely also require other PID values.
 *
 * **Requirements**
 *
 * This class requires ROS parameters as in the file config/JacoControlLocalImpl.yaml to be loaded on the
 * parameter server. This is to read in the PID values which are specific to controlling the
 * joints via physics::Joint::SetForce(). If physics::Joint::SetVelocity() is to be used instead,
 * the configuration file config/JacoControlLocalImpl.yaml can be used instead.
 *
 * **Limitations**
 *
 *  At the moment, no force commands are implemented yet.
 *
 * \author Jennifer Buehler
 * \date January 2016
 */
class JacoGazeboJointControlLocalImpl : public JacoGazeboJointControl
{
public:
    /// \brief Constructor
    JacoGazeboJointControlLocalImpl();

    /// \brief Destructor
    virtual ~JacoGazeboJointControlLocalImpl();

protected:
    /**
     * Implementation of JacoGazeboJointControl::UpdateJoints() which supports
     * both position and velocity controllers and may also use SetVelocity() instead of
     * SetForce() on the joints.
     */
    virtual bool UpdateJoints();

    virtual bool DisableGravity() const;
    virtual bool UseForce() const;
    
    // returns true if a Joint::SetVelocity() is to be used instead of
    // Joint::SetForce(). Note that this will require loading different PID values.
    // The flag is read from the ROS parameter "jaco/gazebo_use_set_velocity", and
    // it defaults to true. This is because on Indigo, use of SetForce() doesn't work
    // yet at this stage. And using SetVelocity() is more stable also on Jade.
    bool SetVelocity() const;
    virtual void GetDefaultPosGains(float& kp, float& ki, float& kd) const;
    virtual void GetDefaultVelGains(float& kp, float& ki, float& kd) const;

    virtual void tested() const {}

private:

    /**
     *  Helper to adjust a PID command according to how close the joint
     *  is to the goal. *If* with the current joint velocity, 
     *  in \e lookaheadSecs seconds the target would be
     *  *overshot*, the joint needs to slow down, hence the current
     *  \e commandValue reduced. It is reduced according to how
     *  close the joint is to the target, assuming to scale the value
     *  such that a velocity may be achieved to exactly reach the target
     *  in \e lookaheadSecs.
     *
     *  This helps for example to reduce applied force when
     *  the arm is falling down due to gravity anyway, and
     *  actually an opposite force would need to be applied to
     *  slow it down in approach to its goal...
     *
     * \param distToTarget distance to target pose in radians (i.e. how
     * many radians the joint still has to move to its target)
     */
/*    double AdjustForCloseTarget(const physics::JointPtr& joint,
            const double distToTarget, const gazebo::common::Time& stepTime,
            const double lookaheadSecs, const double commandValue) const;*/



    double DistToPosition(const physics::JointPtr& joint, double targetPosition) const;

    /**
     * Updates the position PID controller passed as \e pid and returns the target velocity to set on the joint
     */
    double UpdateVelocityToPosition(const physics::JointPtr& joint, double targetPosition,
            common::PID& pid, const gazebo::common::Time& stepTime) const;
    /**
     * Updates the position PID controller passed as \e pid and returns the target force to set on the joint
     */
    double UpdateForceToPosition(const physics::JointPtr& joint, double targetPosition,
            common::PID& pid, const gazebo::common::Time& stepTime) const;


    common::Time prevUpdateTime;

    // see SetVelocity().
    bool useSetVelocity;
};

}  // namespace gazebo

#endif  // JACO_GAZEBO_JACOGAZEBOJOINTCONTROLLOCALIMPL_H
