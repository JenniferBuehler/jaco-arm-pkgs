#ifndef JACO_GAZEBO_JACOGAZEBOJOINTCONTROL_H
#define JACO_GAZEBO_JACOGAZEBOJOINTCONTROL_H

/**
   Provides a gazebo plugin which uses a PID controller to control the arm.

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


#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <jaco_joints/JacoJointManager.h>
#include <jaco_gazebo/JointController.h>

#include <ros/ros.h>

#include <string>

/**
 * this is the angle (in radians) that the joints will always be away from their limits.
 * this is to prevent gazebo instabilities (fingers go flying at joint limits at times)
 */
#define JOINTLIMIT_PADDING 0.01

#ifndef GAZEBO_JADE_VERSION
/**
 * Minimum version of Gazebo with ROS Jade
 */
#define GAZEBO_JADE_VERSION 5
#endif

namespace gazebo
{


/**
 * \brief Provides a gazebo plugin which uses a PID controller to control the arm.
 *
 * This class has the following main features:
 * 1.  A gazebo::physics::JointController object is used to manage the PID controllers of the joints.
 *     This JointController is attached to the actual gazebo::Model, such that several plugins
 *     can access the same controller seamlessly.
 * 2.  Maintains the robot's position according to the last commands send to the PID controller.
 * 3.  Addresses a limitation of the current gazebo_ros_control DefaultRobotHWSim implementation,
 *     which does not allow the simultaneous use of both position and velocity PID controllers.
 *     However to effectively use velocity controllers, also position controllers are required
 *     (read more about this below).
 *
 * **To 1)**
 *
 * It is convenient if several separate Gazebo plugins can access the same joint controller and
 * send velocity commands. Evidently this can also lead to conflicts if several plugins send
 * contradicting commands. It is assumed that plugins trigger the change of position or velocity
 * commands based on a service request of the user (e.g. execute joint trajectory or set a certain
 * arm pose), and that the user does not send commands in such way that they interfere
 * (i.e. don't send two service requests for the arm at the same time).
 * Such inconsistent commanding of the model would always lead to some sort of conflict anyway.
 *
 * The JointController in use could in theory also be the one which is directly attached
 * to the gazebo model and can be retrieved with physics::Model::GetJointController().
 * However to avoid possible conflicts with other internal gazebo usage of the same controller,
 * a new one is created. There is also another reason to use a separate JointController:
 * gazebo::physics::JointController is *not threadsafe*. So to allow simulaneous access to
 * the same JointController instance, it should be made threadsafe. A threadsafe wrapper of
 * physics::JointController has been added in the local implementation of JointController.
 * All plugins should all use this *threadsafe* access to the JointController object,
 * which can be retrieved with
 * model->GetChild(physics::JointControllerThreadsafe::UniqueName()).
 * The returned child will be an instance of JointControllerThreadsafePtr.
 *
 * **To 3)**
 *
 * So why do we need both position and velocity controllers?
 * This is best explained with an example.
 *
 * Say we want to control the arm with velocities *only*. And we want it to maintain the current
 * position by setting all velocities to 0.
 * This does not work with PID controllers. Take the example:
 * - we want the hand to maintain a position in which it is *stretched out* and pulled down by
 *   gravity, so that it would fall down unless an "upwards" force is applied.
 * - gravity will pull it down, and the joint will have a small negative velocity
 *   (or, rather, a "downwards" velocity).
 *   The PID controller now wants to set it to positive ("upwards") velocity in order to reach 0 again.
 * - now the velocity is positive, just to get back to 0 velocity. So the PID demands a
 *   a negative ("downwards") velocity again, to reach zero. And here we have an instable situation.
 *   The arm will never rest, wiggle around, and eventually not maintain its position very well at all.
 *
 * This can be solved by adding position controllers as well:
 * - Use position controllers to maintain the *current* position if there is no velocity commands, or
 *   the velocity command is 0.
 * - if there are non-zero velocity commands, ignore the position command and only set the
 *   effort/velocity based on the velocity command.
 *
 * The ideal solution would be to re-implement (or extend) gazebo_ros_control::gazebo::DefaultRobotHWSim
 * to support the above functionanlity. Then, we could also use ros_control ROS controllers for the Jaco arm.
 *
 * **Limitation of this base class**
 *
 * At the current time it is not possible to create an instance of this class. This is
 * because the basic implementation in this class acts as adapter to physics::JointController
 * and calls physics::JointController::Update() in its update loop.
 * However the current physics::JointController implementation has the following limitation:
 * - it does not fully support having both position and velocity controllers (reasons outlined
 *   above that positions are required when velocity is 0).
 * - it uses physics::Joint::SetForce() to control the robot, which can lead to problems with
 *   some Gazebo versions.
 *
 * Therefore, currently it is required to instantiate only subclasses which work around these
 * limitations. The pure virtual protected method tested() has been added to make sure no instances
 * can be created of this class. It will be removed as soon as above mentioned limitations of
 * JointController have been fixed.
 *
 * **Requirements**
 *
 * This class requires ROS parameters as in the file config/JacoControl.yaml to be loaded on the
 * parameter server. This is to read in the PID values. The format is kept such that it will
 * later by compatible with ros_control, so at this time, only the PID values matter.
 *
 * *Important*: Please refer to subclasses documentation if they require a particular
 * version of this .yaml file with different values to be loaded.
 * (the same yaml config file can also be used with ros_control, because the
 * naming in the config file refers to transmissions tags in the URDF file).
 *
 *
 * **Fine-tuning the jaco arm**
 *
 * Controlling the Jaco joints may require a bit of paramerer tuning on different
 * systems and Gazebo versions. If the robot explodes while moving, it mostly means
 * that too much force is applied on the joints. However if not enough force is applied,
 * the arm can't lift itself.
 * The weight and PID values also play a significant role.
 * - Robot weight can be adjusted in the URDF file using the mass_divider parameter to the
 *   jaco_arm xacro macro.
 * - PID values can be adjusted in the config .yaml files.
 * - Maximum forces can be adjusted in JacoGazeboControlConstants.h for use with this
 *   class. Later, when there will be a possible merge with gazebo_ros_control, you
 *   will have to specify the maximum forces **in the URDF file** as well!
 * - The damping and friction parameters for the joints in the URDF file are also
 *   very significant. The right values can prevent the robot from exploding or
 *   moving slower than the specified velocity.
 *
 * The seemingly most important factor in making robots explode is
 * too much force allowed or a damping/friction which is too high.
 * Higher PID gains don't have too much of an effect. It may be better to start testing with
 * higher PID values to increase likelyhood to achieve the max force.
 * The robot mass also plays a role. The lighter the robot, the easier it explodes when
 * movements are made; when it is heavy, it is harder to lift the arm, and when it moves down,
 * there's a risk of it collapsing down and then exploding after.
 *
 * \author Jennifer Buehler
 * \date January 2016
 */
class JacoGazeboJointControl : public ModelPlugin
{
public:
    /// \brief Constructor
    JacoGazeboJointControl();

    /// \brief Destructor
    virtual ~JacoGazeboJointControl();

    /** \brief Load the controller
     *
     * Initializes the models joint controller. It creates an instance
     * of gazebo::physics::JointControllerThreadsafe and initializes the joint controller of
     * this instance with gazebo::physics::Model::GetJointController() of the model.
     * Then, adds the JointControllerThreadsafe instance as a child of the
     * model, using the name JointControllerThreadsafe::UniqueName.
     *
     * Then, initializes the joint controller by assigning
     * position PID controllers to all the Jaco joints. If the
     * ROS parameter "gazebo/load_velocity_controllers" is set to true, also velocity
     * PID controllers are loaded (true is the default).
     *
     * Further, sets the initial joint positions to the "home" position.
     * Also sets the first *position* PID target values according to the initial pose.
     *
     * Method UpdateJoints() is then supposed to
     * use the joint controller which can be retrieved from
     * model->GetChild(physics::JointControllerThreadsafe::UniqueName()) to
     * read the current target values. Then, it should either:
     *
     * 1. call the gazebo::physics::JointControllerThreadsafe::Update()
     *    in order to control the joints accoring to the PID values or
     * 2. implement another workaround to control the joints according to
     *    the target values set in the JointController and the PID values.
     */
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
    virtual void UpdateChild();

    /**
     * \return true if gravity is to be explicitly disabled for
     * all links attached to the arm joints.
     */
    virtual bool DisableGravity() const;

    /**
     * \return true if force is to be used to achieve velocities,
     * and also the PID controllers are initialized with force.
     */
    virtual bool UseForce() const;

    /**
     * Return default PID values for all position controllers. This is used
     * in case reading from ROS parameters fails.
     */
    virtual void GetDefaultPosGains(float& kp, float& ki, float& kd) const;
    /**
     * Return default PID values for all velocity controllers. This is used
     * in case reading from ROS parameters fails.
     */
    virtual void GetDefaultVelGains(float& kp, float& ki, float& kd) const;

    /**
     * This method updates the joints according to the values
     * set in the joint controller (which can be retrieved by
     * physics::Model::GetJointController()). This joint controller
     * is initialized in this plugins Load() method by assigning
     * PID controllers to all the Jaco joints.
     *
     * This method takes charge of either
     *
     * 1. calling the gazebo::physics::JointControllerThreadsafe::Update()
     *    in order to control the joints accoring to the PID values or
     * 2. implementing another workaround to control the joints according to
     *    the target values set in the JointController and the PID values.
     *
     * Subclasses may chosse either of the options for implementation.
     * \return false if some error occured with updating the joints, true otherwise
     */
    virtual bool UpdateJoints();

    /**
     * Adjusts a target velocity to set limits.
     *
     * 1. caps it to the maximum values allowed for this joint
     * 2. if the joint is near to its limit (within JOINTLIMIT_PADDING) the velocity
     *     is set to 0 and
     * 3. [optional] if the joint is close to the actual target end pose (see parameter
     *     useEndTargetPos and endTargetPos), the velocity is set to 0.
     *
     * \param useEndTargetPos when set to true, consider \e endTargetPos and \e tolerance as
     *   follows: aside from the target velocity, there may also be an end target
     *   pose \e endTargetPos which the joint is supposed to reach eventually.
     *   Whenever the joint is close to the end target (within \e tolerance),
     *   the velocity of the joint is set to 0.
     */
    double capTargetVel(const physics::JointPtr joint, const float targetVel, const bool useEndTargetPos = false,
                        const float endTargetPos = 0, const float tolerance = 1e-03) const;

    /**
     * Adjusts a target force to set limits.
     *
     * 1. caps it to the maximum values allowed for this joint
     * 2. [optional, only if considerJointLimits=true): if the joint is near to its
     *     limit (within JOINTLIMIT_PADDING) the force 
     *     is not allowed to go in the direction beyond the limit
     */
    double capTargetForce(const physics::JointPtr joint, const float targetForce, const bool considerJointLimits) const;



    bool isFinger(const physics::JointPtr& joint) const;
    
    bool velocityControllersLoaded() const
    {
        return loadedVelocityControllers;
    }

    virtual void tested() const = 0;

    /**
     * The gazebo model
     */
    physics::ModelPtr model;

    /**
     * The jaco joint names manager
     */
    JacoJointManager joints;

    /**
     * The joint controller which is created in Load().
     */
    physics::JointControllerThreadsafePtr jointController;

private:
    /**
     * Calls JacoJointManager::GetPosGains() and uses the returned PID values
     */
    void GetPosGains(const std::string& jointName, float& kp, float& ki, float& kd) const;
    /**
     * Calls JacoJointManager::GetVelGains() and uses the returned PID values
     */
    void GetVelGains(const std::string& jointName, float& kp, float& ki, float& kd) const;

    /**
     * Reads maximum force and velocity for this joints from the constants
     */
    void GetMaxVals(const std::string& jointName, float& force, float& velocity) const;

    void WorldUpdate();
 //   void WorldUpdate(const ros::TimerEvent& t);

    event::ConnectionPtr update_connection;
//    ros::Timer update_connection;
    ros::NodeHandle nh;


    bool loadedVelocityControllers;
};

}  // namespace gazebo

#endif  // JACO_GAZEBO_JACOGAZEBOJOINTCONTROL_H
