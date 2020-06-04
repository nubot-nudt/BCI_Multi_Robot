/*
 * Copyright (C) 2015 NuBot team of National University of Defense Technology
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

/* Desc: As an extension to the operation of gazebo::ignition::math::Vector3d
 *       Calculate angles between two vectors both used for planar and spatial vectors
 * Author: Weijia Yao
 * Date: Jun 2015
 */

#ifndef VECTOR_ANGLE_HH
#define VECTOR_ANGLE_HH

#include <gazebo/gazebo.hh>     // the core gazebo header files, including gazebo/math/gzmath.hh
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>

#define PI 3.14159265
using namespace gazebo;

double get_cos_angle(ignition::math::Vector3d vector1, ignition::math::Vector3d vector2); // vectors' angle range [-PI, PI]
double get_sin_angle(ignition::math::Vector3d reference_vector, ignition::math::Vector3d target_vector); // vectors' angle range [-PI, PI]
double get_angle_PI(ignition::math::Vector3d reference_vector, ignition::math::Vector3d target_vector);   // return angle range [-PI,PI]
double get_angle_2PI(ignition::math::Vector3d reference_vector, ignition::math::Vector3d target_vector);  // return angle range [0, 2*PI]

double get_cos_angle(ignition::math::Vector3d vector1, ignition::math::Vector3d vector2)
{
    ignition::math::Vector3d vector1_norm = vector1.Normalize();
    ignition::math::Vector3d vector2_norm = vector2.Normalize();
    return vector1_norm.Dot(vector2_norm);
}

double get_sin_angle(ignition::math::Vector3d reference_vector, ignition::math::Vector3d target_vector)
{
    ignition::math::Vector3d reference_vector_norm = reference_vector.Normalize();
    ignition::math::Vector3d target_vector_norm = target_vector.Normalize();
    ignition::math::Vector3d cross_vector = reference_vector_norm.Cross(target_vector_norm);
    return (cross_vector.Z()>0 ? cross_vector.Length() : -cross_vector.Length());
}

double get_angle_PI(ignition::math::Vector3d reference_vector, ignition::math::Vector3d target_vector) // return angle range [-PI,PI]
{
    double cos_angle = get_cos_angle(reference_vector, target_vector);
    double sin_angle = get_sin_angle(reference_vector, target_vector);
    double angle = acos(cos_angle);
    return (sin_angle>0 ? angle : -angle);
}

double get_angle_2PI(ignition::math::Vector3d reference_vector, ignition::math::Vector3d target_vector)  // return angle range [0, 2*PI]
{
    double cos_angle = get_cos_angle(reference_vector, target_vector);
    double sin_angle = get_sin_angle(reference_vector, target_vector);
    double angle = acos(cos_angle);
    return (sin_angle>0 ? angle : 2*PI-angle);
}

#endif //! VECTOR_ANGLE_HH
