/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Willow Garage, Inc.
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

#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/revolute_joint_model.h>
#include <moveit/exceptions/exceptions.h>
#include <console_bridge/console.h>
#include <boost/lexical_cast.hpp>
#include <algorithm>

namespace moveit
{
namespace core
{
namespace
{

struct OrderLinksByIndex
{
  bool operator()(const LinkModel *a, const LinkModel *b) const
  {
    return a->getTreeIndex() < b->getTreeIndex();
  }
};

struct OrderJointsByIndex
{
  bool operator()(const JointModel *a, const JointModel *b) const
  {
    return a->getTreeIndex() < b->getTreeIndex();
  }
};
  
// check if a parent or ancestor of joint is included in this group
bool includesParent(const JointModel *joint, const JointModelGroup *group)
{
  bool found = false;
  // if we find that an ancestor is also in the group, then the joint is not a root
  while (joint->getParentLinkModel() != NULL)
  {
    joint = joint->getParentLinkModel()->getParentJointModel();
    if (group->hasJointModel(joint->getName()) && joint->getVariableCount() > 0 && joint->getMimic() == NULL)
    {
      found = true;
      break;
    }
    else
      if (joint->getMimic() != NULL)
      {
        const JointModel *mjoint = joint->getMimic();
        if (group->hasJointModel(mjoint->getName()) && mjoint->getVariableCount() > 0 && mjoint->getMimic() == NULL)
          found = true;
        else
          if (includesParent(mjoint, group))
            found = true;
        if (found)
          break;
      }
  }
  return found;
}

// check if joint a is right below b, in the kinematic chain, with no active DOF missing
bool jointPrecedes(const JointModel *a, const JointModel *b)
{
  if (!a->getParentLinkModel())
    return false;
  const JointModel *p = a->getParentLinkModel()->getParentJointModel();
  while (p)
  {
    if (p == b)
      return true;
    if (p->getType() == JointModel::FIXED)
      p = p->getParentLinkModel() ? p->getParentLinkModel()->getParentJointModel() : NULL;
  }
  
  return false;
}

}
}
}

moveit::core::JointModelGroup::JointModelGroup(const std::string& group_name,
                                               const srdf::Model::Group &config,
                                               const std::vector<const JointModel*> &unsorted_group_joints,
                                               const RobotModel* parent_model)
  : parent_model_(parent_model)
  , name_(group_name)
  , variable_count_(0)
  , is_chain_(false)
  , default_ik_timeout_(0.5)
  , default_ik_attempts_(2)
  , config_(config)
{
  // sort joints in Depth-First order
  std::vector<const JointModel*> group_joints = unsorted_group_joints;
  std::sort(group_joints.begin(), group_joints.end(), OrderJointsByIndex());
  
  for (std::size_t i = 0 ; i < group_joints.size() ; ++i)
  {
    joint_model_map_[group_joints[i]->getName()] = group_joints[i];
    unsigned int vc = group_joints[i]->getVariableCount();
    if (vc > 0)
    {
      if (group_joints[i]->getMimic() == NULL)
      {
        joint_model_vector_.push_back(group_joints[i]);
        joint_model_start_index_.push_back(variable_count_);
        variable_count_ += vc;
      }
      else
        mimic_joints_.push_back(group_joints[i]);
      
      if (group_joints[i]->getType() == JointModel::REVOLUTE && 
          static_cast<const RevoluteJointModel*>(group_joints[i])->isContinuous())
        continuous_joint_model_vector_.push_back(group_joints[i]);
    }
    else
      fixed_joints_.push_back(group_joints[i]);
  }
  
  //now we need to find all the set of joints within this group
  //that root distinct subtrees
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
  {
    joint_model_name_vector_.push_back(joint_model_vector_[i]->getName());
    bool found = false;
    const JointModel *joint = joint_model_vector_[i];
    // if we find that an ancestor is also in the group, then the joint is not a root
    if (!includesParent(joint, this))
      joint_roots_.push_back(joint_model_vector_[i]);
  }
  
  // compute joint_variables_index_map_
  unsigned int vector_index_counter = 0;
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
  {
    const std::vector<std::string>& name_order = joint_model_vector_[i]->getVariableNames();
    for (std::size_t j = 0; j < name_order.size(); ++j)
    {
      joint_variables_index_map_[name_order[j]] = vector_index_counter + j;
      active_variable_names_.push_back(name_order[j]);
      active_variable_names_set_.insert(name_order[j]);
    }
    joint_variables_index_map_[joint_model_vector_[i]->getName()] = vector_index_counter;
    vector_index_counter += name_order.size();
  }

  // now we need to make another pass for group links (we include the fixed joints here)
  std::set<const LinkModel*> group_links_set;
  for (std::size_t i = 0 ; i < group_joints.size() ; ++i)
    group_links_set.insert(group_joints[i]->getChildLinkModel());
  for (std::set<const LinkModel*>::iterator it = group_links_set.begin(); it != group_links_set.end(); ++it)
    link_model_vector_.push_back(*it);
  std::sort(link_model_vector_.begin(), link_model_vector_.end(), OrderLinksByIndex());

  for (std::size_t i = 0 ; i < link_model_vector_.size() ; ++i)
  {
    link_model_map_[link_model_vector_[i]->getName()] = link_model_vector_[i];
    link_model_name_vector_.push_back(link_model_vector_[i]->getName());
    if (!link_model_vector_[i]->getShapes().empty())
    {
      link_model_with_geometry_vector_.push_back(link_model_vector_[i]);
      link_model_with_geometry_name_vector_.push_back(link_model_vector_[i]->getName());
    }
  }
  
  // compute updated links
  for (std::size_t i = 0 ; i < joint_roots_.size() ; ++i)
  {
    const std::vector<const LinkModel*> &links = joint_roots_[i]->getDescendantLinkModels();
    updated_link_model_set_.insert(links.begin(), links.end());
  }
  for (std::set<const LinkModel*>::iterator it = updated_link_model_set_.begin(); it != updated_link_model_set_.end(); ++it)
  {
    updated_link_model_name_set_.insert((*it)->getName());
    updated_link_model_vector_.push_back(*it);
    if (!(*it)->getShapes().empty())
    {
      updated_link_model_with_geometry_vector_.push_back(*it);
      updated_link_model_with_geometry_set_.insert(*it);
      updated_link_model_with_geometry_name_set_.insert((*it)->getName());
    }
  }
  std::sort(updated_link_model_vector_.begin(), updated_link_model_vector_.end(), OrderLinksByIndex());
  std::sort(updated_link_model_with_geometry_vector_.begin(), updated_link_model_with_geometry_vector_.end(), OrderLinksByIndex());
  for (std::size_t i = 0; i < updated_link_model_vector_.size(); ++i)
    updated_link_model_name_vector_.push_back(updated_link_model_vector_[i]->getName());
  for (std::size_t i = 0; i < updated_link_model_with_geometry_vector_.size(); ++i)
    updated_link_model_with_geometry_name_vector_.push_back(updated_link_model_with_geometry_vector_[i]->getName());
  
  // check if this group should actually be a chain
  if (joint_roots_.size() == 1 && joint_model_vector_.size() > 1 && !is_chain_)
  {
    bool chain = true;
    // due to our sorting, the joints are sorted in a DF fashion, so looking at them in reverse, 
    // we should always get to the parent.    
    for (std::size_t k = group_joints.size() - 1 ; k > 0 ; --k)
      if (!jointPrecedes(group_joints[k], group_joints[k - 1]))
      {
        chain = false;
        break;
      }
    if (chain)
      markAsChain();
  }
  computeVariableBoundsMsg();
}

moveit::core::JointModelGroup::~JointModelGroup()
{
}

void moveit::core::JointModelGroup::setSubgroupNames(const std::vector<std::string> &subgroups)
{
  subgroup_names_ = subgroups;
  subgroup_names_set_.clear();
  for (std::size_t i = 0 ; i < subgroup_names_.size() ; ++i)
    subgroup_names_set_.insert(subgroup_names_[i]);
}

bool moveit::core::JointModelGroup::hasJointModel(const std::string &joint) const
{
  return joint_model_map_.find(joint) != joint_model_map_.end();
}

bool moveit::core::JointModelGroup::hasLinkModel(const std::string &link) const
{
  return link_model_map_.find(link) != link_model_map_.end();
}

const moveit::core::LinkModel* moveit::core::JointModelGroup::getLinkModel(const std::string &name) const
{
  boost::container::flat_map<std::string, const LinkModel*>::const_iterator it = link_model_map_.find(name);
  if (it == link_model_map_.end())
  {
    logError("Link '%s' not found in group '%s'", name.c_str(), name_.c_str());
    return NULL;
  }
  return it->second;
}

const moveit::core::JointModel* moveit::core::JointModelGroup::getJointModel(const std::string &name) const
{
  boost::container::flat_map<std::string, const JointModel*>::const_iterator it = joint_model_map_.find(name);
  if (it == joint_model_map_.end())
  {
    logError("Joint '%s' not found in group '%s'", name.c_str(), name_.c_str());
    return NULL;
  }
  return it->second;
}

void moveit::core::JointModelGroup::getVariableRandomValues(random_numbers::RandomNumberGenerator &rng, double *values) const
{
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
    joint_model_vector_[i]->getVariableRandomValues(rng, values + joint_model_start_index_[i]);
}

void moveit::core::JointModelGroup::getVariableRandomValuesNearBy(random_numbers::RandomNumberGenerator &rng, double *values, const double *near, double distance) const
{
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
    joint_model_vector_[i]->getVariableRandomValuesNearBy(rng, values + joint_model_start_index_[i], near + joint_model_start_index_[i], distance);
}

void moveit::core::JointModelGroup::getVariableRandomValuesNearBy(random_numbers::RandomNumberGenerator &rng, double *values, const double *near, const std::map<JointModel::JointType, double> &distance_map) const
{
  for (std::size_t i = 0  ; i < joint_model_vector_.size() ; ++i)
  {
    double distance = 0.0;
    std::map<moveit::core::JointModel::JointType, double>::const_iterator iter = distance_map.find(joint_model_vector_[i]->getType());
    if (iter != distance_map.end())
      distance = iter->second;
    else
      logWarn("Did not pass in distance for '%s'", joint_model_vector_[i]->getName().c_str());
    joint_model_vector_[i]->getVariableRandomValuesNearBy(rng, values + joint_model_start_index_[i], near + joint_model_start_index_[i], distance);
  }
}

void moveit::core::JointModelGroup::getVariableRandomValuesNearBy(random_numbers::RandomNumberGenerator &rng, double *values, const double *near, const std::vector<double> &distances) const
{
  if (distances.size() != joint_model_vector_.size())
    throw Exception("When sampling random values nearby for group '" + name_ + "', distances vector should be of size " + 
                    boost::lexical_cast<std::string>(joint_model_vector_.size()) + ", but it is of size " + 
                    boost::lexical_cast<std::string>(distances.size()));  
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
    joint_model_vector_[i]->getVariableRandomValuesNearBy(rng, values + joint_model_start_index_[i], near + joint_model_start_index_[i], distances[i]);
}

double moveit::core::JointModelGroup::getMaximumExtent(void) const
{
  double max_distance = 0.0;
  for (std::size_t j = 0 ; j < joint_model_vector_.size() ; ++j)
    max_distance += joint_model_vector_[j]->getMaximumExtent() * joint_model_vector_[j]->getDistanceFactor();
  return max_distance;
}

void moveit::core::JointModelGroup::addDefaultState(const std::string &name, const std::map<std::string, double> &default_state)
{
  default_states_[name] = default_state;
  default_states_names_.push_back(name);
}

bool moveit::core::JointModelGroup::getVariableDefaultValues(const std::string &name, std::map<std::string, double> &values) const
{
  std::map<std::string, std::map<std::string, double> >::const_iterator it = default_states_.find(name);
  if (it == default_states_.end())
    return false;
  values = it->second;
  return true;
}

void moveit::core::JointModelGroup::getVariableDefaultValues(double *values) const
{
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
    joint_model_vector_[i]->getVariableDefaultValues(values + joint_model_start_index_[i]);
}

void moveit::core::JointModelGroup::getVariableDefaultValues(std::map<std::string, double> &values) const
{
  std::vector<double> tmp(variable_count_);
  getVariableDefaultValues(&tmp[0]);
  for (std::size_t i = 0 ; i < active_variable_names_.size() ; ++i)
    values[active_variable_names_[i]] = tmp[i];
}

void moveit::core::JointModelGroup::computeVariableBoundsMsg()
{
  variable_bounds_msg_.clear();
  for (std::size_t i = 0; i < joint_model_vector_.size(); i++)
  {
    const std::vector<moveit_msgs::JointLimits> &jvec = joint_model_vector_[i]->getVariableBoundsMsg();
    variable_bounds_msg_.insert(variable_bounds_msg_.end(), jvec.begin(), jvec.end());
  }
}

void moveit::core::JointModelGroup::markAsChain()
{
  is_chain_ = true;
}

void moveit::core::JointModelGroup::setEndEffectorName(const std::string &name)
{
  end_effector_name_ = name;
}

void moveit::core::JointModelGroup::setEndEffectorParent(const std::string &group, const std::string &link)
{
  end_effector_parent_.first = group;
  end_effector_parent_.second = link;
}

void moveit::core::JointModelGroup::attachEndEffector(const std::string &eef_name)
{
  attached_end_effector_names_.push_back(eef_name);
}

void moveit::core::JointModelGroup::setVariableBounds(const std::vector<moveit_msgs::JointLimits>& jlim)
{
  for (std::size_t i = 0; i < joint_model_vector_.size(); ++i)
    const_cast<JointModel*>(joint_model_vector_[i])->setVariableBounds(jlim);
  computeVariableBoundsMsg();
}

void moveit::core::JointModelGroup::setDefaultIKTimeout(double ik_timeout)
{
  default_ik_timeout_ = ik_timeout;
  if (solver_instance_)
    solver_instance_->setDefaultTimeout(ik_timeout);
}

void moveit::core::JointModelGroup::setSolverAllocators(const std::pair<SolverAllocatorFn, SolverAllocatorMapFn> &solvers)
{
  solver_allocators_ = solvers;
  if (solver_allocators_.first)
  {
    solver_instance_ = solver_allocators_.first(this);
    if (solver_instance_)
    {
      ik_joint_bijection_.clear();
      const std::vector<std::string> &ik_jnames = solver_instance_->getJointNames();
      for (std::size_t i = 0 ; i < ik_jnames.size() ; ++i)
      {
        VariableIndexMap::const_iterator it = joint_variables_index_map_.find(ik_jnames[i]);
        if (it == joint_variables_index_map_.end())
        {
          logError("IK solver computes joint values for joint '%s' but group '%s' does not contain such a joint.", ik_jnames[i].c_str(), getName().c_str());
          solver_instance_.reset();
          return;
        }
        const JointModel *jm = getJointModel(ik_jnames[i]);
        for (unsigned int k = 0 ; k < jm->getVariableCount() ; ++k)
          ik_joint_bijection_.push_back(it->second + k);
      }
      solver_instance_const_ = solver_instance_;
    }
  }
}

bool moveit::core::JointModelGroup::canSetStateFromIK(const std::string &tip) const
{
  const kinematics::KinematicsBaseConstPtr& solver = getSolverInstance();
  if (!solver || tip.empty())
    return false;
  const std::string &tip_frame = solver->getTipFrame();
  if (tip_frame.empty())
    return false;
  
  // remove frame reference, if specified
  const std::string &tip_local = tip[0] == '/' ? tip.substr(1) : tip;
  const std::string &tip_frame_local = tip_frame[0] == '/' ? tip_frame.substr(1) : tip_frame;

  if (tip_local != tip_frame_local)
  {
    if (hasLinkModel(tip_frame_local))
    {
      const LinkModel *lm = getLinkModel(tip_frame_local);
      const LinkModel::AssociatedFixedTransformMap &fixed_links = lm->getAssociatedFixedTransforms();
      for (LinkModel::AssociatedFixedTransformMap::const_iterator it = fixed_links.begin() ; it != fixed_links.end() ; ++it)
        if (it->first->getName() == tip_local)
          return true;
    }
    return false;
  }
  else
    return true;
}

void moveit::core::JointModelGroup::printGroupInfo(std::ostream &out) const
{
  out << "Group '" << name_ << "':" << std::endl;
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
  {
    const std::vector<std::string> &vn = joint_model_vector_[i]->getVariableNames();
    for (std::vector<std::string>::const_iterator it = vn.begin() ; it != vn.end() ; ++it)
    {
      out << "   " << *it << " [";
      const VariableBounds &b = joint_model_vector_[i]->getVariableBounds(*it);
      if (b.min_position_ <= -std::numeric_limits<double>::max())
        out << "DBL_MIN";
      else
        out << b.min_position_;
      out << ", ";
      if (b.max_position_ >= std::numeric_limits<double>::max())
        out << "DBL_MAX";
      else
        out << b.max_position_;
      out << "]";
      if (joint_model_vector_[i]->getMimic())
        out << " *";
      if (joint_model_vector_[i]->isPassive())
        out << " +";
      out << std::endl;
    }
  }
}
