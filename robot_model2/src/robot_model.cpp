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

#include <moveit/robot_model/robot_model.h>
#include <geometric_shapes/shape_operations.h>
#include <boost/math/constants/constants.hpp>
#include <moveit/profiler/profiler.h>
#include <algorithm>
#include <limits>
#include <queue>
#include <cmath>

/* ------------------------ RobotModel ------------------------ */


namespace moveit
{
namespace core
{
namespace
{

struct OrderGroupsByName
{
  bool operator()(const JointModelGroup *a, const JointModelGroup *b) const
  {
    return a->getName() < b->getName();
  }
};

}
}
}

moveit::core::RobotModel::RobotModel(const boost::shared_ptr<const urdf::ModelInterface> &urdf_model,
                                     const boost::shared_ptr<const srdf::Model> &srdf_model)
{
  root_joint_ = NULL;
  urdf_ = urdf_model;
  srdf_ = srdf_model;
  buildModel(*urdf_model, *srdf_model);
}

moveit::core::RobotModel::~RobotModel()
{
  for (JointModelGroupMap::iterator it = joint_model_group_map_.begin() ; it != joint_model_group_map_.end() ; ++it)
    delete it->second;
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
    delete joint_model_vector_[i];
  for (std::size_t i = 0 ; i < link_model_vector_.size() ; ++i)
    delete link_model_vector_[i];
}

const moveit::core::JointModel* moveit::core::RobotModel::getRootJoint() const
{
  return root_joint_;
}

const moveit::core::LinkModel* moveit::core::RobotModel::getRootLink() const
{
  return root_link_;
}

void moveit::core::RobotModel::buildModel(const urdf::ModelInterface &urdf_model, const srdf::Model &srdf_model)
{
  moveit::tools::Profiler::ScopedStart prof_start;
  moveit::tools::Profiler::ScopedBlock prof_block("RobotModel::buildModel");

  root_joint_ = NULL;
  root_link_ = NULL;
  model_name_ = urdf_model.getName();
  if (urdf_model.getRoot())
  {
    const urdf::Link *root_link_ptr = urdf_model.getRoot().get();
    model_frame_ = '/' + root_link_ptr->name;

    root_joint_ = buildRecursive(NULL, root_link_ptr, srdf_model);
    if (root_joint_)
      root_link_ = root_joint_->getChildLinkModel();
    buildMimic(urdf_model);
    buildJointInfo();

    if (link_models_with_collision_geometry_vector_.empty())
      logWarn("No geometry is associated to any robot links");

    // build groups
    buildGroups(srdf_model);
    buildGroupStates(srdf_model);

    std::stringstream ss;
    printModelInfo(ss);
    logDebug("%s", ss.str().c_str());
  }
  else
    logWarn("No root link found");
}

namespace moveit
{
namespace core
{
namespace 
{
void computeDescendantsHelper(const JointModel *joint, std::vector<const JointModel*> &parents)
{
  if (!joint)
    return;
  
  for (std::size_t i = 0 ; i < parents.size() ; ++i)
    const_cast<JointModel*>(parents[i])->addDescendantJoint(joint);
  
  const LinkModel *lm = joint->getChildLinkModel();
  if (!lm)
    return;
  
  for (std::size_t i = 0 ; i < parents.size() ; ++i)
    const_cast<JointModel*>(parents[i])->addDescendantLink(lm);
  
  parents.push_back(joint);
  const std::vector<const JointModel*> &ch = lm->getChildJointModels();
  for (std::size_t i = 0 ; i < ch.size() ; ++i)
    computeDescendantsHelper(ch[i], parents);
  const std::vector<const JointModel*> &mim = joint->getMimicRequests();
  for (std::size_t i = 0 ; i < mim.size() ; ++i)
    computeDescendantsHelper(mim[i], parents);
  parents.pop_back();
}

void computeCommonRoots(const JointModel *joint, std::vector<int> &common_roots, int size)
{
  if (!joint)
    return;
  const LinkModel *lm = joint->getChildLinkModel();
  if (!lm)
    return;
  
  const std::vector<const JointModel*> &ch = lm->getChildJointModels();
  for (std::size_t i = 0 ; i < ch.size() ; ++i)
  {
    const std::vector<const JointModel*> &a = ch[i]->getDescendantJointModels();
    for (std::size_t j = i + 1 ; j < ch.size() ; ++j)
    {
      const std::vector<const JointModel*> &b = ch[j]->getDescendantJointModels();
      for (std::size_t k = 0 ; k < a.size() ; ++k)
        for (std::size_t m = 0 ; m < b.size() ; ++m)
          common_roots[a[k]->getJointIndex() * size + b[m]->getJointIndex()] = 
            common_roots[a[k]->getJointIndex() + b[m]->getJointIndex() * size] = joint->getJointIndex();
    }
    computeCommonRoots(ch[i], common_roots, size);
  }
}

}
}
}


void moveit::core::RobotModel::buildJointInfo()
{
  // construct additional maps for easy access by name
  variable_count_ = 0;
  variable_bounds_.clear();
  joint_model_start_index_.resize(joint_model_vector_.size(), -1);
  
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
  {
    const std::vector<std::string> &name_order = joint_model_vector_[i]->getVariableNames();
    for (std::size_t j = 0 ; j < name_order.size() ; ++j)
      variable_bounds_[name_order[j]] = joint_model_vector_[i]->getVariableBounds(name_order[j]);
    
    if (joint_model_vector_[i]->getMimic() == NULL)
    {
      // compute index map
      if (name_order.size() > 0)
      {
        for (std::size_t j = 0; j < name_order.size(); ++j)
        {
          joint_variables_index_map_[name_order[j]] = variable_count_ + j;
          active_variable_names_.push_back(name_order[j]);
          joints_of_variable_.push_back(joint_model_vector_[i]);
        }
        joint_model_start_index_[i] = variable_count_;
        joint_model_vector_[i]->setFirstVariableIndex(variable_count_);
        joint_model_vector_[i]->setJointIndex(i);
        joint_variables_index_map_[joint_model_vector_[i]->getName()] = variable_count_;

        // compute variable count
        variable_count_ += joint_model_vector_[i]->getVariableCount();
      }
    }
  }
  
  for (std::size_t i = 0 ; i < link_model_vector_.size() ; ++i)
  {
    LinkModel::AssociatedFixedTransformMap associated_transforms;
    computeFixedTransforms(link_model_vector_[i], Eigen::Affine3d::Identity(), associated_transforms);
    if (associated_transforms.size() > 1)
    {
      for (LinkModel::AssociatedFixedTransformMap::iterator it = associated_transforms.begin() ; it != associated_transforms.end() ; ++it)
        if (it->first != link_model_vector_[i])
        {
          getLinkModel(it->first->getName())->addAssociatedFixedTransform(link_model_vector_[i], it->second.inverse());
          link_model_vector_[i]->addAssociatedFixedTransform(it->first, it->second);
        }
    }
  }
  
  // compute the list of descendants for all joints
  std::vector<const JointModel*> dummy;
  computeDescendantsHelper(root_joint_, dummy);
  
  // compute common roots for all pairs of joints
  common_joint_roots_.resize(joint_model_vector_.size() * joint_model_vector_.size(), 0);
  computeCommonRoots(root_joint_, common_joint_roots_, joint_model_vector_.size());
}

void moveit::core::RobotModel::buildGroupStates(const srdf::Model &srdf_model)
{
  // copy the default states to the groups
  const std::vector<srdf::Model::GroupState> &ds = srdf_model.getGroupStates();
  for (std::size_t i = 0 ; i < ds.size() ; ++i)
  {
    if (hasJointModelGroup(ds[i].group_))
    {
      JointModelGroup *jmg = getJointModelGroup(ds[i].group_);
      std::map<std::string, double> state;
      for (std::map<std::string, std::vector<double> >::const_iterator jt = ds[i].joint_values_.begin() ; jt != ds[i].joint_values_.end() ; ++jt)
      {
        if (jmg->hasJointModel(jt->first))
        {
          const JointModel* jm = jmg->getJointModel(jt->first);
          const std::vector<std::string> &vn = jm->getVariableNames();
          if (vn.size() == jt->second.size())
            for (std::size_t j = 0 ; j < vn.size() ; ++j)
              state[vn[j]] = jt->second[j];
          else
            logError("The model for joint '%s' requires %d variable values, but only %d variable values were supplied in default state '%s' for group '%s'",
                     jt->first.c_str(), (int)vn.size(), (int)jt->second.size(), ds[i].name_.c_str(), jmg->getName().c_str());
        }
        else
          logError("Group state '%s' specifies value for joint '%s', but that joint is not part of group '%s'", ds[i].name_.c_str(),
                   jt->first.c_str(), jmg->getName().c_str());
      }
      if (!state.empty())
        jmg->addDefaultState(ds[i].name_, state);
    }
    else
      logError("Group state '%s' specified for group '%s', but that group does not exist", ds[i].name_.c_str(), ds[i].group_.c_str());
  }
}

void moveit::core::RobotModel::buildMimic(const urdf::ModelInterface &urdf_model)
{
  // compute mimic joints
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
  {
    const urdf::Joint *jm = urdf_model.getJoint(joint_model_vector_[i]->getName()).get();
    if (jm)
      if (jm->mimic)
      {
        JointModelMap::const_iterator jit = joint_model_map_.find(jm->mimic->joint_name);
        if (jit != joint_model_map_.end())
        {
          if (joint_model_vector_[i]->getVariableCount() == jit->second->getVariableCount())
            joint_model_vector_[i]->setMimic(jit->second, jm->mimic->multiplier, jm->mimic->offset);
          else
            logError("Join '%s' cannot mimic joint '%s' because they have different number of DOF",
                     joint_model_vector_[i]->getName().c_str(), jm->mimic->joint_name.c_str());
        }
        else
          logError("Joint '%s' cannot mimic unknown joint '%s'", joint_model_vector_[i]->getName().c_str(), jm->mimic->joint_name.c_str());
      }
  }
  
  // in case we have a joint that mimics a joint that already mimics another joint, we can simplify things:
  bool change = true;
  while (change)
  {
    change = false;
    for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
      if (joint_model_vector_[i]->getMimic())
      {
        if (joint_model_vector_[i]->getMimic()->getMimic())
        {
          joint_model_vector_[i]->setMimic(joint_model_vector_[i]->getMimic()->getMimic(),
                                           joint_model_vector_[i]->getMimicFactor() * joint_model_vector_[i]->getMimic()->getMimicFactor(),
                                           joint_model_vector_[i]->getMimicOffset() + 
                                           joint_model_vector_[i]->getMimicFactor() * joint_model_vector_[i]->getMimic()->getMimicOffset());
          change = true;
        }
        if (joint_model_vector_[i] == joint_model_vector_[i]->getMimic())
        {
          logError("Cycle found in joint that mimic each other. Ignoring all mimic joints.");
          for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
            joint_model_vector_[i]->setMimic(NULL, 0.0, 0.0);
          change = false;
          break;
        }
      }
  }
  // build mimic requests
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
    if (joint_model_vector_[i]->getMimic())
      const_cast<JointModel*>(joint_model_vector_[i]->getMimic())->addMimicRequest(joint_model_vector_[i]);
}

bool moveit::core::RobotModel::hasEndEffector(const std::string& eef) const
{
  return end_effectors_map_.find(eef) != end_effectors_map_.end();
}

const moveit::core::JointModelGroup* moveit::core::RobotModel::getEndEffector(const std::string& name) const
{
  JointModelGroupMap::const_iterator it = end_effectors_map_.find(name);
  if (it == end_effectors_map_.end())
  {
    it = joint_model_group_map_.find(name);
    if (it != joint_model_group_map_.end() && it->second->isEndEffector())
      return it->second;
    logError("End-effector '%s' not found in model '%s'", name.c_str(), model_name_.c_str());
    return NULL;
  }
  return it->second;
}

moveit::core::JointModelGroup* moveit::core::RobotModel::getEndEffector(const std::string& name)
{
  JointModelGroupMap::const_iterator it = end_effectors_map_.find(name);
  if (it == end_effectors_map_.end())
  {
    it = joint_model_group_map_.find(name);
    if (it != joint_model_group_map_.end() && it->second->isEndEffector())
      return it->second;
    logError("End-effector '%s' not found in model '%s'", name.c_str(), model_name_.c_str());
    return NULL;
  }
  return it->second;
}

bool moveit::core::RobotModel::hasJointModelGroup(const std::string &name) const
{
  return joint_model_group_map_.find(name) != joint_model_group_map_.end();
}

const moveit::core::JointModelGroup* moveit::core::RobotModel::getJointModelGroup(const std::string& name) const
{
  JointModelGroupMap::const_iterator it = joint_model_group_map_.find(name);
  if (it == joint_model_group_map_.end())
  {
    logError("Group '%s' not found in model '%s'", name.c_str(), model_name_.c_str());
    return NULL;
  }
  return it->second;
}

moveit::core::JointModelGroup* moveit::core::RobotModel::getJointModelGroup(const std::string& name)
{
  JointModelGroupMap::const_iterator it = joint_model_group_map_.find(name);
  if (it == joint_model_group_map_.end())
  {    
    logError("Group '%s' not found in model '%s'", name.c_str(), model_name_.c_str());
    return NULL;
  }
  return it->second;
}

void moveit::core::RobotModel::buildGroups(const srdf::Model &srdf_model)
{
  const std::vector<srdf::Model::Group>& group_configs = srdf_model.getGroups();

  //the only thing tricky is dealing with subgroups
  std::vector<bool> processed(group_configs.size(), false);

  bool added = true;
  while (added)
  {
    added = false;

    //going to make passes until we can't do anything else
    for (std::size_t i = 0 ; i < group_configs.size() ; ++i)
      if (!processed[i])
      {
        //if we haven't processed, check and see if the dependencies are met yet
        bool all_subgroups_added = true;
        for (std::size_t j = 0; j < group_configs[i].subgroups_.size(); ++j)
          if (joint_model_group_map_.find(group_configs[i].subgroups_[j]) == joint_model_group_map_.end())
          {
            all_subgroups_added = false;
            break;
          }
        if (all_subgroups_added)
        {
          added = true;
          processed[i] = true;
          if (!addJointModelGroup(group_configs[i]))
            logWarn("Failed to add group '%s'", group_configs[i].name_.c_str());
        }
      }
  }

  for (std::size_t i = 0 ; i < processed.size() ; ++i)
    if (!processed[i])
      logWarn("Could not process group '%s' due to unmet subgroup dependencies", group_configs[i].name_.c_str());
  
  for (JointModelGroupMap::const_iterator it = joint_model_group_map_.begin() ; it != joint_model_group_map_.end(); ++it)
    joint_model_groups_.push_back(it->second);
  std::sort(joint_model_groups_.begin(), joint_model_groups_.end(), OrderGroupsByName());
  for (std::size_t i = 0 ; i < joint_model_groups_.size() ; ++i)
  {
    joint_model_groups_const_.push_back(joint_model_groups_[i]);
    joint_model_group_names_.push_back(joint_model_groups_[i]->getName());
  }
  
  buildGroupsInfo_Subgroups(srdf_model);
  buildGroupsInfo_EndEffectors(srdf_model);
}

void moveit::core::RobotModel::buildGroupsInfo_Subgroups(const srdf::Model &srdf_model)
{
  // compute subgroups
  for (JointModelGroupMap::const_iterator it = joint_model_group_map_.begin() ; it != joint_model_group_map_.end(); ++it)
  {
    JointModelGroup *jmg = it->second;
    std::vector<std::string> subgroup_names;
    std::set<const JointModel*> joints(jmg->getJointModels().begin(), jmg->getJointModels().end());
    for (JointModelGroupMap::const_iterator jt = joint_model_group_map_.begin() ; jt != joint_model_group_map_.end(); ++jt)
      if (jt->first != it->first)
      {
        bool ok = true;
        JointModelGroup *sub_jmg = jt->second;
        const std::vector<const JointModel*> &sub_joints = sub_jmg->getJointModels();
        for (std::size_t k = 0 ; k < sub_joints.size() ; ++k)
          if (joints.find(sub_joints[k]) == joints.end())
          {
            ok = false;
            break;
          }
        if (ok)
          subgroup_names.push_back(sub_jmg->getName());
      }
    if (!subgroup_names.empty())
      jmg->setSubgroupNames(subgroup_names);
  }
}

void moveit::core::RobotModel::buildGroupsInfo_EndEffectors(const srdf::Model &srdf_model)
{
  // set the end-effector flags
  const std::vector<srdf::Model::EndEffector> &eefs = srdf_model.getEndEffectors();
  for (JointModelGroupMap::const_iterator it = joint_model_group_map_.begin() ; it != joint_model_group_map_.end(); ++it)
  {
    // check if this group is a known end effector
    for (std::size_t k = 0 ; k < eefs.size() ; ++k)
      if (eefs[k].component_group_ == it->first)
      {
        // if it is, mark it as such        
        it->second->setEndEffectorName(eefs[k].name_);
        end_effectors_map_[eefs[k].name_] = it->second;
        end_effectors_.push_back(it->second);
        JointModelGroup *eef_parent_group = NULL;

        // if a parent group is specified in SRDF, try to use it
        if (!eefs[k].parent_group_.empty())
        {
          JointModelGroupMap::const_iterator jt = joint_model_group_map_.find(eefs[k].parent_group_);
          if (jt != joint_model_group_map_.end())
          {
            if (jt->second->hasLinkModel(eefs[k].parent_link_))
            {
              if (jt->second != it->second)
                eef_parent_group = jt->second;
              else
                logError("Group '%s' for end-effector '%s' cannot be its own parent", eefs[k].parent_group_.c_str(), eefs[k].name_.c_str());
            }
            else
              logError("Group '%s' was specified as parent group for end-effector '%s' but it does not include the parent link '%s'",
                       eefs[k].parent_group_.c_str(), eefs[k].name_.c_str(), eefs[k].parent_link_.c_str());
          }
          else
            logError("Group name '%s' not found (specified as parent group for end-effector '%s')",
                     eefs[k].parent_group_.c_str(), eefs[k].name_.c_str());
        }
        
        if (eef_parent_group == NULL)
        {
          // check to see if there are groups that contain the parent link of this end effector.
          // record this information if found
          std::vector<JointModelGroup*> possible_parent_groups;
          for (JointModelGroupMap::const_iterator jt = joint_model_group_map_.begin() ; jt != joint_model_group_map_.end(); ++jt)
            if (jt->first != it->first)
            {
              if (jt->second->hasLinkModel(eefs[k].parent_link_))
                possible_parent_groups.push_back(jt->second);
            }
          if (!possible_parent_groups.empty())
          {
            // if there are multiple options for the group that contains this end-effector,
            // we pick the group with fewest joints.
            std::size_t best = 0;
            for (std::size_t g = 1 ; g < possible_parent_groups.size() ; ++g)
              if (possible_parent_groups[g]->getJointModels().size() < possible_parent_groups[best]->getJointModels().size())
                best = g;
            eef_parent_group = possible_parent_groups[best];
          }
        }
        if (eef_parent_group)
        {
          //attached_end_effector_names_.push_back(
          eef_parent_group->attachEndEffector(eefs[k].name_);
          it->second->setEndEffectorParent(eef_parent_group->getName(), eefs[k].parent_link_);
        }
        else
        {
          logWarn("Could not identify parent group for end-effector '%s'", eefs[k].name_.c_str());
          it->second->setEndEffectorParent("", eefs[k].parent_link_);
        }        
        break;
      }
  }
  std::sort(end_effectors_.begin(), end_effectors_.end(), OrderGroupsByName());
}

bool moveit::core::RobotModel::addJointModelGroup(const srdf::Model::Group& gc)
{
  if (joint_model_group_map_.find(gc.name_) != joint_model_group_map_.end())
  {
    logWarn("A group named '%s' already exists. Not adding.",  gc.name_.c_str());
    return false;
  }

  std::set<const JointModel*> jset;

  // add joints from chains
  for (std::size_t i = 0 ; i < gc.chains_.size() ; ++i)
  {
    const LinkModel* base_link = getLinkModel(gc.chains_[i].first);
    const LinkModel* tip_link = getLinkModel(gc.chains_[i].second);
    if (base_link && tip_link)
    {
      // go from tip, up the chain, until we hit the root or we find the base_link
      const LinkModel* lm = tip_link;
      std::vector<const JointModel*> cj;
      while (lm)
      {
        if (lm == base_link)
          break;
        cj.push_back(lm->getParentJointModel());
        lm = lm->getParentJointModel()->getParentLinkModel();
      }
      // if we did not find the base_link, we could have a chain like e.g.,
      // from one end-effector to another end-effector, so the root is in between
      if (lm != base_link)
      {
        // we go up the chain from the base this time, and see where we intersect the other chain
        lm = base_link;
        std::size_t index = 0;
        std::vector<const JointModel*> cj2;
        while (lm)
        {
          for (std::size_t j = 0 ; j < cj.size() ; ++j)
            if (cj[j] == lm->getParentJointModel())
            {
              index = j + 1;
              break;
            }
          if (index > 0)
            break;
          cj2.push_back(lm->getParentJointModel());
          lm = lm->getParentJointModel()->getParentLinkModel();
        }
        if (index > 0)
        {
          jset.insert(cj.begin(), cj.begin() + index);
          jset.insert(cj2.begin(), cj2.end());
        }
      }
      else
        // if we have a simple chain, just add the joints
        jset.insert(cj.begin(), cj.end());
    }
  }

  // add joints
  for (std::size_t i = 0 ; i < gc.joints_.size() ; ++i)
  {
    const JointModel *j = getJointModel(gc.joints_[i]);
    if (j)
      jset.insert(j);
  }

  // add joints that are parents of included links
  for (std::size_t i = 0 ; i < gc.links_.size() ; ++i)
  {
    const LinkModel *l = getLinkModel(gc.links_[i]);
    if (l)
      jset.insert(l->getParentJointModel());
  }

  // add joints from subgroups
  for (std::size_t i = 0 ; i < gc.subgroups_.size() ; ++i)
  {
    const JointModelGroup *sg = getJointModelGroup(gc.subgroups_[i]);
    if (sg)
    {
      //active joints
      const std::vector<const JointModel*> &js = sg->getJointModels();
      for (std::size_t j = 0 ; j < js.size() ; ++j)
        jset.insert(js[j]);

      //fixed joints
      const std::vector<const JointModel*> &fs = sg->getFixedJointModels();
      for (std::size_t j = 0 ; j < fs.size() ; ++j)
        jset.insert(fs[j]);

      //mimic joints
      const std::vector<const JointModel*> &ms = sg->getMimicJointModels();
      for (std::size_t j = 0 ; j < ms.size() ; ++j)
        jset.insert(ms[j]);
    }
  }

  if (jset.empty())
  {
    logWarn("Group '%s' must have at least one valid joint", gc.name_.c_str());
    return false;
  }

  std::vector<const JointModel*> joints;
  for (std::set<const JointModel*>::iterator it = jset.begin() ; it != jset.end() ; ++it)
    joints.push_back(*it);

  JointModelGroup *jmg = new JointModelGroup(gc.name_, gc, joints, this);
  joint_model_group_map_[gc.name_] = jmg;

  // if the group is defined as a single chain, then we mark is as a chain aleady
  // (this is for the case where the chain does not consist of consecutive joints and would not be detected as a chain later)
  if (gc.chains_.size() == 1 && gc.joints_.empty() && gc.links_.empty() && gc.subgroups_.empty())
    jmg->markAsChain();
  
  return true;
}

moveit::core::JointModel* moveit::core::RobotModel::buildRecursive(LinkModel *parent, const urdf::Link *urdf_link,
                                                                   const srdf::Model &srdf_model)
{
  // construct the joint
  JointModel *joint = urdf_link->parent_joint ?
    constructJointModel(urdf_link->parent_joint.get(), urdf_link, srdf_model) :
    constructJointModel(NULL, urdf_link, srdf_model);
  if (joint == NULL)
    return NULL;
  
  // bookkeeping for the joint
  joint_model_map_[joint->getName()] = joint;
  joint->setTreeIndex(joint_model_vector_.size());
  joint_model_vector_.push_back(joint);
  joint_model_vector_const_.push_back(joint);
  joint_model_names_vector_.push_back(joint->getName());
  if (joint->getType() == JointModel::REVOLUTE && static_cast<const RevoluteJointModel*>(joint)->isContinuous())
    continuous_joint_model_vector_.push_back(joint);
  joint->setParentLinkModel(parent);

  // construct the link
  LinkModel *link = constructLinkModel(urdf_link);
  joint->setChildLinkModel(link);

  // bookkeeping for the link
  link_model_map_[joint->getChildLinkModel()->getName()] = link;
  link->setTreeIndex(link_model_vector_.size());
  link_model_vector_.push_back(link);
  link_model_vector_const_.push_back(link);
  link_model_names_vector_.push_back(link->getName());
  if (!link->getShapes().empty())
  {
    link_models_with_collision_geometry_vector_.push_back(link);
    link_model_names_with_collision_geometry_vector_.push_back(link->getName());
  }
  link->setParentJointModel(joint);

  // recursively build child links (and joints)
  for (std::size_t i = 0 ; i < urdf_link->child_links.size() ; ++i)
  {
    JointModel* jm = buildRecursive(link, urdf_link->child_links[i].get(), srdf_model);
    if (jm)
      link->addChildJointModel(jm);
  }
  return joint;
}

namespace
{
// construct bounds for 1DOF joint
static inline moveit::core::VariableBounds jointBoundsFromURDF(const urdf::Joint *urdf_joint)
{
  moveit::core::VariableBounds b;
  if (urdf_joint->safety)
  {
    b.position_bounded_ = true;
    b.min_position_ = urdf_joint->safety->soft_lower_limit;
    b.max_position_ = urdf_joint->safety->soft_upper_limit;
    if (urdf_joint->limits)
    {
      if (urdf_joint->limits->lower > b.min_position_)
        b.min_position_ = urdf_joint->limits->lower;
      if (urdf_joint->limits->upper < b.max_position_)
        b.max_position_ = urdf_joint->limits->upper;
    }
  }
  else
  {
    if (urdf_joint->limits)
    {
      b.position_bounded_ = true;
      b.min_position_ = urdf_joint->limits->lower;
      b.max_position_ = urdf_joint->limits->upper;
    }
  }
  if (urdf_joint->limits)
  {
    b.max_velocity_ = fabs(urdf_joint->limits->velocity);
    b.min_velocity_ = -b.max_velocity_;
    b.velocity_bounded_ = b.max_velocity_ > std::numeric_limits<double>::epsilon();
  }
}
}

moveit::core::JointModel* moveit::core::RobotModel::constructJointModel(const urdf::Joint *urdf_joint, const urdf::Link *child_link,
                                                                        const srdf::Model &srdf_model)
{
  JointModel* result = NULL;

  // must be the root link transform
  if (urdf_joint)
  {
    switch (urdf_joint->type)
    {
    case urdf::Joint::REVOLUTE:
      {
        RevoluteJointModel *j = new RevoluteJointModel(urdf_joint->name);
        j->setVariableBounds(j->getName(), jointBoundsFromURDF(urdf_joint));
        j->setContinuous(false);
        j->setAxis(Eigen::Vector3d(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z));
        result = j;
      }
      break;
    case urdf::Joint::CONTINUOUS:
      {
        RevoluteJointModel *j = new RevoluteJointModel(urdf_joint->name);
        j->setVariableBounds(j->getName(), jointBoundsFromURDF(urdf_joint));
        j->setContinuous(true);
        j->setAxis(Eigen::Vector3d(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z));
        result = j;
      }
      break;
    case urdf::Joint::PRISMATIC:
      {
        PrismaticJointModel *j = new PrismaticJointModel(urdf_joint->name);
        j->setVariableBounds(j->getName(), jointBoundsFromURDF(urdf_joint));
        j->setAxis(Eigen::Vector3d(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z));
        result = j;
      }
      break;
    case urdf::Joint::FLOATING:
      result = new FloatingJointModel(urdf_joint->name);
      break;
    case urdf::Joint::PLANAR:
      result = new PlanarJointModel(urdf_joint->name);
      break;
    case urdf::Joint::FIXED:
      result = new FixedJointModel(urdf_joint->name);
      break;
    default:
      logError("Unknown joint type: %d", (int)urdf_joint->type);
      break;
    }
  }
  else
  {
    const std::vector<srdf::Model::VirtualJoint> &vjoints = srdf_model.getVirtualJoints();
    for (std::size_t i = 0 ; i < vjoints.size() ; ++i)
      if (vjoints[i].child_link_ == child_link->name && !vjoints[i].parent_frame_.empty())
      {
        if (vjoints[i].type_ == "fixed")
          result = new FixedJointModel(vjoints[i].name_);
        else if (vjoints[i].type_ == "planar")
          result = new PlanarJointModel(vjoints[i].name_);
        else if (vjoints[i].type_ == "floating")
          result = new FloatingJointModel(vjoints[i].name_);
        if (result)
        {
          // for fixed frames we still use the robot root link
          if (vjoints[i].type_ != "fixed")
          {
            model_frame_ = vjoints[i].parent_frame_;
            if (model_frame_[0] != '/')
              model_frame_ = '/' + model_frame_;
          }
          break;
        }
      }
    if (!result)
    {
      logInform("No root joint specified. Assuming fixed joint");
      result = new FixedJointModel("ASSUMED_FIXED_ROOT_JOINT");
    }
  }

  if (result)
  {
    result->setDistanceFactor(result->getStateSpaceDimension());
    const std::vector<srdf::Model::PassiveJoint> &pjoints = srdf_model.getPassiveJoints();
    for (std::size_t i = 0 ; i < pjoints.size() ; ++i)
    {
      if (result->getName() == pjoints[i].name_)
      {
        result->setPassive(true);
        break;
      }
    }
  }
  
  return result;
}

namespace 
{
static inline Eigen::Affine3d urdfPose2Affine3d(const urdf::Pose &pose)
{
  Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
  Eigen::Affine3d af(Eigen::Translation3d(pose.position.x, pose.position.y, pose.position.z)*q.toRotationMatrix());
  return af;
}

}

moveit::core::LinkModel* moveit::core::RobotModel::constructLinkModel(const urdf::Link *urdf_link)
{
  LinkModel *result = new LinkModel(urdf_link->name);

  const std::vector<boost::shared_ptr<urdf::Collision> > &col_array = urdf_link->collision_array.empty() ? 
    std::vector<boost::shared_ptr<urdf::Collision> >(1, urdf_link->collision) : urdf_link->collision_array;
  
  std::vector<shapes::ShapeConstPtr> shapes;
  EigenSTL::vector_Affine3d poses;
  
  for (std::size_t i = 0 ; i < col_array.size() ; ++i)
    if (col_array[i] && col_array[i]->geometry)
    {
      shapes::ShapeConstPtr s = constructShape(col_array[i]->geometry.get());
      if (s)
      {
        shapes.push_back(s);
        poses.push_back(urdfPose2Affine3d(col_array[i]->origin));
      }
    }
  if (shapes.empty())
  {
    const std::vector<boost::shared_ptr<urdf::Visual> > &vis_array = urdf_link->visual_array.empty() ? 
      std::vector<boost::shared_ptr<urdf::Visual> >(1, urdf_link->visual) : urdf_link->visual_array;
    for (std::size_t i = 0 ; i < vis_array.size() ; ++i)
      if (vis_array[i] && vis_array[i]->geometry)
      {
        shapes::ShapeConstPtr s = constructShape(vis_array[i]->geometry.get());
        if (s)
        {
          shapes.push_back(s);
          poses.push_back(urdfPose2Affine3d(vis_array[i]->origin));
        }
      }
  }
  
  result->setGeometry(shapes, poses);
  
  // figure out visual mesh (try visual urdf tag first, collision tag otherwise
  if (urdf_link->visual && urdf_link->visual->geometry)
  {
    if (urdf_link->visual->geometry->type == urdf::Geometry::MESH)
    {
      const urdf::Mesh *mesh = static_cast<const urdf::Mesh*>(urdf_link->visual->geometry.get());
      if (!mesh->filename.empty())
        result->setVisualMesh(mesh->filename, Eigen::Vector3d(mesh->scale.x, mesh->scale.y, mesh->scale.z));
    }
  }
  else
    if (urdf_link->collision && urdf_link->collision->geometry)
    {
      if (urdf_link->collision->geometry->type == urdf::Geometry::MESH)
      {
        const urdf::Mesh *mesh = static_cast<const urdf::Mesh*>(urdf_link->collision->geometry.get());
        if (!mesh->filename.empty())
          result->setVisualMesh(mesh->filename, Eigen::Vector3d(mesh->scale.x, mesh->scale.y, mesh->scale.z));
      }
    }
  
  if (urdf_link->parent_joint)
    result->setJointOriginTransform(urdfPose2Affine3d(urdf_link->parent_joint->parent_to_joint_origin_transform));
  
  return result;
}

shapes::ShapePtr moveit::core::RobotModel::constructShape(const urdf::Geometry *geom)
{
  moveit::tools::Profiler::ScopedBlock prof_block("RobotModel::constructShape");

  shapes::Shape *result = NULL;
  switch (geom->type)
  {
  case urdf::Geometry::SPHERE:
    result = new shapes::Sphere(static_cast<const urdf::Sphere*>(geom)->radius);
    break;
  case urdf::Geometry::BOX:
    {
      urdf::Vector3 dim = static_cast<const urdf::Box*>(geom)->dim;
      result = new shapes::Box(dim.x, dim.y, dim.z);
    }
    break;
  case urdf::Geometry::CYLINDER:
    result = new shapes::Cylinder(static_cast<const urdf::Cylinder*>(geom)->radius,
                                  static_cast<const urdf::Cylinder*>(geom)->length);
    break;
  case urdf::Geometry::MESH:
    {
      const urdf::Mesh *mesh = static_cast<const urdf::Mesh*>(geom);
      if (!mesh->filename.empty())
      {
        Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
        shapes::Mesh *m = shapes::createMeshFromResource(mesh->filename, scale);
        result = m;
      }
    }
    break;
  default:
    logError("Unknown geometry type: %d", (int)geom->type);
    break;
  }

  return shapes::ShapePtr(result);
}

bool moveit::core::RobotModel::hasJointModel(const std::string &name) const
{
  return joint_model_map_.find(name) != joint_model_map_.end();
}

bool moveit::core::RobotModel::hasLinkModel(const std::string &name) const
{
  return link_model_map_.find(name) != link_model_map_.end();
}

const moveit::core::JointModel* moveit::core::RobotModel::getJointModel(const std::string &name) const
{
  JointModelMap::const_iterator it = joint_model_map_.find(name);
  if (it != joint_model_map_.end())
    return it->second;
  logError("Joint '%s' not found in model '%s'", name.c_str(), model_name_.c_str());
  return NULL;
}

moveit::core::JointModel* moveit::core::RobotModel::getJointModel(const std::string &name)
{
  JointModelMap::const_iterator it = joint_model_map_.find(name);
  if (it != joint_model_map_.end())
    return it->second;
  logError("Joint '%s' not found in model '%s'", name.c_str(), model_name_.c_str());
  return NULL;
}

const moveit::core::LinkModel* moveit::core::RobotModel::getLinkModel(const std::string &name) const
{
  LinkModelMap::const_iterator it = link_model_map_.find(name);
  if (it != link_model_map_.end())
    return it->second;
  logError("Link '%s' not found in model '%s'", name.c_str(), model_name_.c_str());
  return NULL;  
}

moveit::core::LinkModel* moveit::core::RobotModel::getLinkModel(const std::string &name)
{
  LinkModelMap::const_iterator it = link_model_map_.find(name);
  if (it != link_model_map_.end())
    return it->second;
  logError("Link '%s' not found in model '%s'", name.c_str(), model_name_.c_str());
  return NULL;
}

/*
void moveit::core::RobotModel::getChildLinkModels(const LinkModel *parent, std::vector<const LinkModel*> &links) const
{
  links.clear();
  links.push_back(parent);
  std::queue<const LinkModel*> q;
  std::set<const LinkModel*> seen;
  q.push(parent);
  while (!q.empty())
  {
    const LinkModel* t = q.front();
    q.pop();
    if (seen.insert(t).second)
      for (std::size_t i = 0 ; i < t->child_joint_models_.size() ; ++i)
      {
        links.push_back(t->child_joint_models_[i]->child_link_model_);
        q.push(t->child_joint_models_[i]->child_link_model_);
        for (std::size_t j = 0 ; j < t->child_joint_models_[i]->mimic_requests_.size() ; ++j)
        {
          links.push_back(t->child_joint_models_[i]->mimic_requests_[j]->child_link_model_);
          q.push(t->child_joint_models_[i]->mimic_requests_[j]->child_link_model_);
        }
      }
  }
}

void moveit::core::RobotModel::getChildLinkModels(const JointModel *parent, std::vector<const LinkModel*> &links) const
{
  getChildLinkModels(parent->child_link_model_, links);
}

void moveit::core::RobotModel::getChildJointModels(const LinkModel *parent, std::vector<const JointModel*> &joints) const
{
  joints.clear();
  std::queue<const LinkModel*> q;
  std::set<const LinkModel*> seen;
  q.push(parent);

  while (!q.empty())
  {
    const LinkModel* t = q.front();
    q.pop();
    if (seen.insert(t).second)
      for (unsigned int i = 0 ; i < t->child_joint_models_.size() ; ++i)
      {
        joints.push_back(t->child_joint_models_[i]);
        q.push(t->child_joint_models_[i]->child_link_model_);
        for (std::size_t j = 0 ; j < t->child_joint_models_[i]->mimic_requests_.size() ; ++j)
        {
          joints.push_back(t->child_joint_models_[i]->mimic_requests_[j]);
          q.push(t->child_joint_models_[i]->mimic_requests_[j]->child_link_model_);
        }
      }
  }
}

void moveit::core::RobotModel::getChildJointModels(const JointModel *parent, std::vector<const JointModel*> &joints) const
{
  getChildJointModels(parent->child_link_model_, joints);
  joints.insert(joints.begin(), parent);
}

std::vector<std::string> moveit::core::RobotModel::getChildLinkModelNames(const LinkModel *parent) const
{
  std::vector<const LinkModel*> links;
  getChildLinkModels(parent, links);
  std::vector<std::string> ret_vec(links.size());
  for (std::size_t i = 0; i < links.size(); ++i)
    ret_vec[i] = links[i]->getName();
  return ret_vec;
}

std::vector<std::string> moveit::core::RobotModel::getChildLinkModelNames(const JointModel *parent) const
{
  std::vector<const LinkModel*> links;
  getChildLinkModels(parent, links);
  std::vector<std::string> ret_vec(links.size());
  for(unsigned int i = 0; i < links.size(); ++i)
    ret_vec[i] = links[i]->getName();
  return ret_vec;
}

std::vector<std::string> moveit::core::RobotModel::getChildJointModelNames(const LinkModel *parent) const
{
  std::vector<const JointModel*> joints;
  getChildJointModels(parent, joints);
  std::vector<std::string> ret_vec(joints.size());
  for(unsigned int i = 0 ; i < joints.size() ; ++i)
    ret_vec[i] = joints[i]->getName();
  return ret_vec;
}

std::vector<std::string> moveit::core::RobotModel::getChildJointModelNames(const JointModel *parent) const
{
  std::vector<const JointModel*> joints;
  getChildJointModels(parent, joints);
  std::vector<std::string> ret_vec(joints.size());
  for(unsigned int i = 0 ; i < joints.size(); ++i)
    ret_vec[i] = joints[i]->getName();
  return ret_vec;
}

*/

void moveit::core::RobotModel::getVariableRandomValues(random_numbers::RandomNumberGenerator &rng, double *values) const
{
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
    if (joint_model_start_index_[i] >= 0)
      joint_model_vector_[i]->getVariableRandomValues(rng, values + joint_model_start_index_[i]);
}

void moveit::core::RobotModel::getVariableRandomValues(random_numbers::RandomNumberGenerator &rng, std::map<std::string, double> &values) const
{
  std::vector<double> tmp(variable_count_);
  getVariableRandomValues(rng, &tmp[0]);
  values.clear();
  for (std::size_t i = 0 ; i < active_variable_names_.size() ; ++i)
    values[active_variable_names_[i]] = tmp[i];
}

void moveit::core::RobotModel::getVariableDefaultValues(double *values) const
{
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
    if (joint_model_start_index_[i] >= 0)
      joint_model_vector_[i]->getVariableDefaultValues(values + joint_model_start_index_[i]);
}

void moveit::core::RobotModel::getVariableDefaultValues(std::map<std::string, double> &values) const
{
  std::vector<double> tmp(variable_count_);
  getVariableDefaultValues(&tmp[0]);
  values.clear();
  for (std::size_t i = 0 ; i < active_variable_names_.size() ; ++i)
    values[active_variable_names_[i]] = tmp[i];
}

int moveit::core::RobotModel::getVariableIndex(const std::string &variable) const
{
  VariableIndexMap::const_iterator it = joint_variables_index_map_.find(variable);
  if (it == joint_variables_index_map_.end())
    throw Exception("Variable '" + variable + "' is not known to model '" + model_name_ + "'");
  return it->second;
}

void moveit::core::RobotModel::setKinematicsAllocators(const std::map<std::string, SolverAllocatorFn> &allocators)
{
  for (JointModelGroupMap::const_iterator it = joint_model_group_map_.begin() ; it != joint_model_group_map_.end() ; ++it)
  {
    JointModelGroup *jmg = it->second;
    std::pair<SolverAllocatorFn, SolverAllocatorMapFn> result;
    std::map<std::string, SolverAllocatorFn>::const_iterator jt = allocators.find(jmg->getName());
    if (jt == allocators.end())
    {
      // if an kinematics allocator is NOT available for this group, we try to see if we can use subgroups for IK
      std::set<const JointModel*> joints;
      joints.insert(jmg->getJointModels().begin(), jmg->getJointModels().end());

      std::vector<const JointModelGroup*> subs;

      // go through the groups that we know have IK allocators and see if they are included in the group that does not; if so, put that group in sub
      for (std::map<std::string, SolverAllocatorFn>::const_iterator kt = allocators.begin() ; kt != allocators.end() ; ++kt)
      {
        const JointModelGroup *sub = getJointModelGroup(kt->first);
        if (!sub)
        {
          subs.clear();
          break;
        }
        std::set<const JointModel*> sub_joints;
        sub_joints.insert(sub->getJointModels().begin(), sub->getJointModels().end());

        if (std::includes(joints.begin(), joints.end(), sub_joints.begin(), sub_joints.end()))
        {
          std::set<const JointModel*> resultj;
          std::set_difference(joints.begin(), joints.end(), sub_joints.begin(), sub_joints.end(),
                              std::inserter(resultj, resultj.end()));
          subs.push_back(sub);
          joints.swap(resultj);
        }
      }

      // if we found subgroups, pass that information to the planning group
      if (!subs.empty())
      {
        std::stringstream ss;
        for (std::size_t i = 0 ; i < subs.size() ; ++i)
        {
          ss << subs[i]->getName() << " ";
          result.second[subs[i]] = allocators.find(subs[i]->getName())->second;
        }
        logDebug("Added sub-group IK allocators for group '%s': [ %s]", jmg->getName().c_str(), ss.str().c_str());
      }
    }
    else
      // if the IK allocator is for this group, we use it
      result.first = jt->second;
    jmg->setSolverAllocators(result);
  }
}

void moveit::core::RobotModel::printModelInfo(std::ostream &out) const
{
  out << "Model " << model_name_ << " in frame " << model_frame_ << ", of dimension " << getVariableCount() << std::endl;

  std::ios_base::fmtflags old_flags = out.flags();
  out.setf(std::ios::fixed, std::ios::floatfield);
  std::streamsize old_prec = out.precision();
  out.precision(5);
  out << "Joint values bounds: " << std::endl;
  for (unsigned int i = 0 ; i < joint_model_vector_.size() ; ++i)
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
  out << std::endl;
  out.precision(old_prec);
  out.flags(old_flags);

  out << "Available groups: " << std::endl;
  for (JointModelGroupMap::const_iterator it = joint_model_group_map_.begin() ; it != joint_model_group_map_.end() ; ++it)
  {
    out << "   " << it->first << " (of dimension " << it->second->getVariableCount() << "):" << std::endl;
    out << "     joints:" << std::endl;
    const std::vector<std::string> &jnt = it->second->getJointModelNames();
    for (std::size_t k = 0 ; k < jnt.size() ; ++k)
      out << "      " << jnt[k] << std::endl;
    out << "     links:" << std::endl;
    const std::vector<std::string> &lnk = it->second->getLinkModelNames();
    for (std::size_t k = 0 ; k < lnk.size() ; ++k)
      out << "      " << lnk[k] << std::endl;
    out << "     roots:" << std::endl;
    const std::vector<const JointModel*> &jr = it->second->getJointRoots();
    for (std::size_t k = 0 ; k < jr.size() ; ++k)
      out << "      " << jr[k]->getName() << std::endl;

  }
}

void moveit::core::RobotModel::computeFixedTransforms(const LinkModel *link, const Eigen::Affine3d &transform,
                                                      LinkModel::AssociatedFixedTransformMap &associated_transforms)
{
  associated_transforms[link] = transform;
  for (std::size_t i = 0 ; i < link->getChildJointModels().size() ; ++i)
    if (link->getChildJointModels()[i]->getType() == JointModel::FIXED)
      computeFixedTransforms(link->getChildJointModels()[i]->getChildLinkModel(), transform * link->getJointOriginTransform(), associated_transforms);
}
