#ifndef JACO_JOINTS_JACOJOINTMANAGER_H
#define JACO_JOINTS_JACOJOINTMANAGER_H

#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   Manages the jaco joint names as specified in the URDF file.

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
#include <vector>

#include <arm_components_name_manager/ArmComponentsNameManager.h>

#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

/**
 * \author Jennifer Buehler
 * \date January 2016
 */
class JacoJointManager: public arm_components_name_manager::ArmComponentsNameManager
{
public:
    /**
     * Initializes joint names by reading from parameter server.
     * \namespace of the robot which is used in the YAML file
     */
    JacoJointManager(const std::string& robot_namespace="jaco", bool readParams=true);
    JacoJointManager(const JacoJointManager& o);
    
    virtual bool hasDefaults()
    {
        return true;
    }


protected:

    virtual std::string getDefaultPalmLink() const;
    virtual std::vector<std::string> getDefaultArmJoints() const;
    virtual std::vector<std::string> getDefaultArmLinks() const;
    virtual std::vector<float> getDefaultArmJointsInitPose() const;
    virtual std::vector<float> getDefaultGripperJointsInitPose() const;
    virtual std::vector<std::string> getDefaultGripperJoints() const;
    virtual std::vector<std::string> getDefaultGripperLinks() const;
    virtual std::vector<float> getDefaultArmJointsMaxVel() const; 
    virtual std::vector<float> getDefaultArmJointsMaxForce() const; 
    virtual std::vector<float> getDefaultGripperJointsMaxVel() const; 
    virtual std::vector<float> getDefaultGripperJointsMaxForce() const; 
    
};

#endif  // JACO_JOINTS_JACOJOINTMANAGER_H
