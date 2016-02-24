#ifndef JACO_JOINTS_JOINTVELOCITYTRACKER_H
#define JACO_JOINTS_JOINTVELOCITYTRACKER_H

#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   Tracking actual joint velocities when the velocity sensors aren't as accurate as position sensors.

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

#include <string>
#include <list>
#include <map>

#include <ros/ros.h>

// average the velocity over the last <AVG_VELOCITY_SAMPLES> updates
#define AVG_VELOCITY_SAMPLES 5

/**
 * \brief Helper to keep a joint position together with a timestamp and last i velocity samples.
 * Also can be used to store any type of joint reference along with it, no matter
 * whether it's used or not (just pass random int if you don't need the joint reference).
 */
template<class Joint>
class JointStateStamped {
public:
    JointStateStamped<Joint>(const double& pos, const ros::Time& t, Joint& jnt):
        time(t),
        position(pos),
        joint(jnt),
        valid(true){}
    JointStateStamped<Joint>(const double& pos, const ros::Time& t):
        time(t),
        position(pos),
        valid(true){}
    JointStateStamped<Joint>(Joint& jnt):
        joint(jnt),
        valid(false) {}
    JointStateStamped<Joint>():
        valid(false) {}

    JointStateStamped<Joint>(const JointStateStamped<Joint>& o):
        joint(o.joint),
        time(o.time),
        position(o.position),
        velocities(o.velocities),
        valid(o.valid) {}
   
    void reset()
    {
        valid=false;
        velocities.clear();
    }
    void set(const double pos, const ros::Time& t)
    {
        position=pos;
        time=t;
        valid=true;
    }
    bool isValid() const { return valid; }
    void invalidate() { valid=false; } 
    const ros::Time& getTime() const { return time; }
    double getPosition() const { return position; }
    const std::list<double>& getVelocities() const  { return velocities; }

    void addVelocity(double velocity)
    {
        if (velocities.size() >= AVG_VELOCITY_SAMPLES) 
            velocities.pop_front();
        velocities.push_back(velocity);
    }
 
    Joint joint;
private:
    ros::Time time;
    double position;
    bool valid;
    std::list<double> velocities;
};

/**
 *  \brief Helper class to track actual joint velocities when the velocity sensors aren't as accurate as position sensors.
 *
 * Method update() has to be called at frequent intervals for *all* joints,
 * which should be tracked with this helper class.
 *
 * This class is not threadsafe.
 *
 * \author Jennifer Buehler
 * \date January 2016
 */
template<class Joint>
class JointVelocityTracker
{
private:
    typedef std::map<std::string,JointStateStamped<Joint> > JointStateMap;
    typedef JointStateStamped<Joint> JointStateStampedT;
public:
    JointVelocityTracker() {}
    JointVelocityTracker(const JointVelocityTracker& o) {}

    /**
     * Adds this joint to the tracker.
     * \param joint optionally, the joint instance to store along can be passed.
     *      This is only a pointer so that the parameter can be set to NULL and is not
     *      mandatory.
     * \return false if the joint name already exists
     */
    bool add(const std::string& jointName, Joint * joint=NULL)
    {
        typename JointStateMap::iterator it=jointStates.find(jointName);
        if (it==jointStates.end())
        {
            if (joint) jointStates.insert(std::make_pair(jointName,JointStateStampedT(*joint)));
            else jointStates.insert(std::make_pair(jointName,JointStateStampedT()));
            return true; 
        }
        return false;
    }

    /**
     * \param jointName name of the joint
     * \param jointPos current joint position
     * \param atTime time of joint position
     * \return false on error, e.g. if the joint was not added with add().
     */
    bool update(const std::string& jointName, double jointPos, const ros::Time& atTime)
    {
        typename JointStateMap::iterator it=jointStates.find(jointName);
        if (it==jointStates.end())
        {
            return false; 
        }
        if (it->second.getTime() > atTime) {
            ROS_ERROR_STREAM("Inconsistency: update time "<<atTime<<" is greater than last update time "
                <<it->second.getTime()<<" for joint "<<jointName<<". Won't update.");
            return false;
        }

        if (!it->second.isValid())
        {   // this is the first joint update, we can't compute a velocity yet.
            // this call will also validate the instance for later calls.
            it->second.set(jointPos,atTime);
            return true;
        }
        ros::Duration stepTime = atTime - it->second.getTime();

        // not too small a timestep to update:
        if (stepTime.toSec() > 1e-05)
        {
            double velocity = (jointPos - it->second.getPosition()) / stepTime.toSec(); 
            it->second.addVelocity(velocity); 
        }
        
        it->second.set(jointPos,atTime);
        return true;
    }

    /**
     * If update() was called with a valid joint object, it was stored with the
     * joint information and can be retrieved with this function.
     * \return false if the joint is not maintained in this instance, true otherwise,
     *      then \e jnt will contain the joint reference.
     */
    bool getJoint(const std::string& jointName, Joint& jnt) {
        typename JointStateMap::iterator it=jointStates.find(jointName);
        if (it==jointStates.end()) return false;
        jnt=it->second.joint;
        return true;
    }   
 
    /**
     * Computes the average velocity of the last AVG_VELOCITY_SAMPLES measurements for this joint.
     * \retval 1 velocity returned in \e vel
     * \retval 0 no velocities computed yet for this joint (needs at least 1 update), \vel will be 0.
     * \retval -1 joint is not in this tracker
     */
    int getJointVelocity(const std::string& jointName, double& vel) const
    {
        typename JointStateMap::const_iterator it=jointStates.find(jointName);
        if (it==jointStates.end())
            return -1; 
    
        const std::list<double>& velocities = it->second.getVelocities();
        
        if (velocities.empty()) {
            vel=0;
            return 0;
        }

        std::list<double>::const_iterator vels;
        double sum=0;
        for (vels=velocities.begin(); vels!=velocities.end(); ++vels)
        {
            sum+=*vels;
        }
        vel=sum/velocities.size();
        return 1;
    }
private:
    std::map<std::string,JointStateStampedT> jointStates;
};

#endif  // JACO_JOINTS_JOINTVELOCITYTRACKER_H
