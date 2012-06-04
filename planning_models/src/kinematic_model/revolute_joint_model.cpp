/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#include <planning_models/kinematic_model.h>
#include <boost/math/constants/constants.hpp>
#include <limits>
#include <cmath>

planning_models::KinematicModel::RevoluteJointModel::RevoluteJointModel(const std::string& name) : JointModel(name),
                                                                                                   axis_(0.0, 0.0, 0.0), continuous_(false)
{
  type_ = REVOLUTE;
  variable_bounds_.push_back(std::make_pair(-boost::math::constants::pi<double>(), boost::math::constants::pi<double>()));
  variable_names_.push_back(name_);
}

unsigned int planning_models::KinematicModel::RevoluteJointModel::getStateSpaceDimension(void) const
{
  return 1;
}

double planning_models::KinematicModel::RevoluteJointModel::getMaximumExtent(void) const
{  
  return variable_bounds_[0].second - variable_bounds_[0].first;
}

void planning_models::KinematicModel::RevoluteJointModel::getDefaultValues(std::vector<double> &values, const Bounds &bounds) const
{
  // if zero is a valid value
  if (bounds[0].first <= 0.0 && bounds[0].second >= 0.0)
    values.push_back(0.0);
  else
    values.push_back((bounds[0].first + bounds[0].second)/2.0);
}

void planning_models::KinematicModel::RevoluteJointModel::getRandomValues(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values, const Bounds &bounds) const
{
  values.push_back(rng.uniformReal(bounds[0].first, bounds[0].second));
}

void planning_models::KinematicModel::RevoluteJointModel::getRandomValuesNearBy(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values, const Bounds &bounds,
                                                                                const std::vector<double> &near, const double distance) const
{  
  if (continuous_)
  {
    values.push_back(rng.uniformReal(near[values.size()] - distance, near[values.size()] + distance));
    enforceBounds(values, bounds);
  }
  else
    values.push_back(rng.uniformReal(std::max(bounds[0].first, near[values.size()] - distance),
                                     std::min(bounds[0].second, near[values.size()] + distance)));
}

void planning_models::KinematicModel::RevoluteJointModel::interpolate(const std::vector<double> &from, const std::vector<double> &to, const double t, std::vector<double> &state) const
{
  if (continuous_)
  {
    double diff = to[0] - from[0];
    if (fabs(diff) <= boost::math::constants::pi<double>())
      state[0] = from[0] + diff * t;
    else
    {
      if (diff > 0.0)
        diff = 2.0 * boost::math::constants::pi<double>() - diff;
      else
        diff = -2.0 * boost::math::constants::pi<double>() - diff;
      state[0] = from[0] - diff * t;
      // input states are within bounds, so the following check is sufficient
      if (state[0] > boost::math::constants::pi<double>())
        state[0] -= 2.0 * boost::math::constants::pi<double>();
      else
        if (state[0] < -boost::math::constants::pi<double>())
          state[0] += 2.0 * boost::math::constants::pi<double>();
    }
  }
  else
    state[0] = from[0] + (to[0] - from[0]) * t;
}

double planning_models::KinematicModel::RevoluteJointModel::distance(const std::vector<double> &values1, const std::vector<double> &values2) const
{
  assert(values1.size() == 1);
  assert(values2.size() == 1);
  if (continuous_)
  {
    double d = fabs(values1[0] - values2[0]);
    return (d > boost::math::constants::pi<double>()) ? 2.0 * boost::math::constants::pi<double>() - d : d;
  }
  else
    return fabs(values1[0] - values2[0]);
}

bool planning_models::KinematicModel::RevoluteJointModel::satisfiesBounds(const std::vector<double> &values, const Bounds &bounds) const
{
  if (continuous_)
    return true;
  assert(bounds.size() > 0);
  if (values[0] < bounds[0].first || values[0] > bounds[0].second)
    return false;
  return true;
}

void planning_models::KinematicModel::RevoluteJointModel::enforceBounds(std::vector<double> &values, const Bounds &bounds) const
{
  if (continuous_)
  {
    double &v = values[0];
    v = fmod(v, 2.0 * boost::math::constants::pi<double>());
    if (v < -boost::math::constants::pi<double>())
      v += 2.0 * boost::math::constants::pi<double>();
    else
      if (v > boost::math::constants::pi<double>())
        v -= 2.0 * boost::math::constants::pi<double>();
  }
  else 
  {
    const std::pair<double, double> &b = bounds[0];
    if (values[0] < b.first)
      values[0] = b.first;
    else
      if (values[0] > b.second)
        values[0] = b.second;
  }
}

std::vector<moveit_msgs::JointLimits> planning_models::KinematicModel::RevoluteJointModel::getVariableLimits(void) const
{
  std::vector<moveit_msgs::JointLimits> ret_vec = JointModel::getVariableLimits();
  if (continuous_)
    ret_vec[0].has_position_limits = false;
  return ret_vec;
}

void planning_models::KinematicModel::RevoluteJointModel::computeTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const
{
  transf.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
  updateTransform(joint_values, transf);
}

void planning_models::KinematicModel::RevoluteJointModel::updateTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const
{
  transf = Eigen::Affine3d(Eigen::AngleAxisd(joint_values[0], axis_)); 
}

void planning_models::KinematicModel::RevoluteJointModel::computeJointStateValues(const Eigen::Affine3d& transf, std::vector<double> &joint_values) const
{
  joint_values.resize(1);
  Eigen::Quaterniond q(transf.rotation());
  q.normalize();
  joint_values[0] = acos(q.w())*2.0f;
}
