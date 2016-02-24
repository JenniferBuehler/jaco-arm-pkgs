#ifndef JACO_GAZEBO_JACOGAZEBOCONTROLCONSTANTS_H
#define JACO_GAZEBO_JACOGAZEBOCONTROLCONSTANTS_H
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

// have to define the max efforts / velocities
// here so far because gazebo doesn't always read them
// from from the xacro files (at least velocities for continuous joints
// on Indigi/Gazebo 2.2 ? Have to verify this again...)

#define MAX_FINGER_VELOCITY 1.0
#define MAX_ARM_0_VELOCITY 2.0
#define MAX_ARM_1_VELOCITY 2.0
#define MAX_ARM_2_VELOCITY 2.0
#define MAX_ARM_3_VELOCITY 2.0
#define MAX_ARM_4_VELOCITY 4.0
#define MAX_ARM_5_VELOCITY 4.0


#define MAX_FINGER_EFFORT 2 
#define MAX_ARM_0_EFFORT 8
#define MAX_ARM_1_EFFORT 8 
#define MAX_ARM_2_EFFORT 4
#define MAX_ARM_3_EFFORT 2
#define MAX_ARM_4_EFFORT 2
#define MAX_ARM_5_EFFORT 2

#endif  // JACO_GAZEBO_JACOGAZEBOCONTROLCONSTANTS_H
