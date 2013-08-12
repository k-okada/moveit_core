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

#include <moveit/robot_state/robot_state.h>
#include <geometric_shapes/shape_operations.h>
#include <eigen_conversions/eigen_msg.h>

moveit::core::RobotState::RobotState(const RobotModelConstPtr &robot_model, AllocComponents alloc_components)
  : robot_model_(robot_model)
  , called_new_for_(ALLOC_POSITION)
  , position_(new double[robot_model->getVariableCount() * 
			 (1 + (alloc_components & (ALLOC_VELOCITY | ALLOC_ACCELERATION) ? 1 : 0) + 
			  (alloc_components & ALLOC_ACCELERATION ? 1 : 0))])
  , velocity_(alloc_components & (ALLOC_VELOCITY | ALLOC_ACCELERATION) ? position_ + robot_model->getVariableCount() : NULL)
  , acceleration_(alloc_components & ALLOC_ACCELERATION ? velocity_ + robot_model->getVariableCount() : NULL)
  , variable_joint_transforms_(NULL)
  , global_link_transforms_(NULL)
  , global_collision_body_transforms_(NULL)
  , dirty_fk_(NULL)
  , dirty_link_transforms_(NULL)
  , dirty_collision_body_transforms_(NULL)
  , rng_(NULL)
{
  if (alloc_components & ALLOC_TRANSFORMS)
    allocTransforms();
}

moveit::core::RobotState::RobotState(const RobotState &other)
  : position_(NULL)
  , velocity_(NULL)
  , acceleration_(NULL)
  , variable_joint_transforms_(NULL)
  , global_link_transforms_(NULL)
  , global_collision_body_transforms_(NULL)
  , rng_(NULL)
{
  copyFrom(other);
}

moveit::core::RobotState::~RobotState()
{
  delete[] position_;
  if (called_new_for_ & ALLOC_VELOCITY)
    delete[] velocity_;
  if (called_new_for_ & ALLOC_ACCELERATION)
    delete[] acceleration_;
  if (rng_)
    delete rng_;
}

moveit::core::RobotState& moveit::core::RobotState::operator=(const RobotState &other)
{
  if (this != &other)
    copyFrom(other);
  return *this;
}

void moveit::core::RobotState::allocVelocity()
{
  if (!velocity_)
  {
    called_new_for_ |= ALLOC_VELOCITY;
    velocity_ = new double[robot_model_->getVariableCount()];
  }
}

void moveit::core::RobotState::allocAcceleration()
{
  if (!acceleration_)
  {
    if (velocity_)
    {
      called_new_for_ |= ALLOC_ACCELERATION;
      acceleration_ = new double[robot_model_->getVariableCount()];
    }
    else
    {
      called_new_for_ |= ALLOC_VELOCITY;
      velocity_ = new double[robot_model_->getVariableCount() * 2];
      acceleration_ = velocity_ + robot_model_->getVariableCount();
    }
  }
}

void moveit::core::RobotState::allocTransforms()
{
  if (!variable_joint_transforms_)
  {
    transforms_.resize(robot_model_->getJointModelCount() + robot_model_->getLinkModelCount() + robot_model_->getLinkGeometryCount());
    if (transforms_.size() > 0)
    {
      variable_joint_transforms_ = &transforms_[0];      
      global_link_transforms_ = variable_joint_transforms_ + robot_model_->getJointModelCount();
      global_collision_body_transforms_ = global_link_transforms_ + robot_model_->getLinkModelCount();
    }
  }
}

void moveit::core::RobotState::copyFrom(const RobotState &other)
{
  robot_model_ = other.robot_model_;
  
  // if the other state has allocated memory for transforms, we may have to copy data
  if (other.variable_joint_transforms_)
  {
    // if this state does not have memory allocated for transforms yet, allocate it
    if (!variable_joint_transforms_)
      allocTransforms();
    
    // if the other state is fully computed, it is worth copying the data
    if (other.dirty_fk_ == NULL)
    {
      transforms_ = other.transforms_;
      dirty_fk_ = NULL;
      dirty_collision_body_transforms_ = other.dirty_collision_body_transforms_;
      dirty_link_transforms_ = other.dirty_link_transforms_;
    }
    else
    {
      // otherwise, we will just assume everything is dirty and re-compute when needed
      dirty_fk_ = robot_model_->getRootJoint();
      dirty_collision_body_transforms_ = NULL;
      dirty_link_transforms_ = NULL;
    }
  }
  else
  {
    // no transforms to copy, so everything will become dirty if/when they get allocated.
    dirty_fk_ = NULL;
    dirty_collision_body_transforms_ = NULL;
    dirty_link_transforms_ = NULL;
  }
  
  // if we have previously allocated some memory, 
  if (position_)
  {
    // see if more memory needs to be allocated, minimizing calls to new
    if (other.acceleration_)
      allocAcceleration();
    else
      if (other.velocity_)
        allocVelocity();
    // copy the data. we use 3 calls to memcpy to avoid problems of non-contiguous blocks at source & destination
    std::size_t c = robot_model_->getVariableCount() * sizeof(double);
    memcpy(position_, other.position_, c);
    if (other.velocity_)
      memcpy(velocity_, other.velocity_, c);
    if (other.acceleration_)
      memcpy(acceleration_, other.acceleration_, c);      
  }
  else
  {
    // we allocate all the memory we need in one block
    called_new_for_ = ALLOC_POSITION;
    std::size_t vc = robot_model_->getVariableCount();
    position_ = new double[(1 + (other.velocity_ ? 1 : 0) + (other.acceleration_ ? 1 : 0)) * vc];
    std::size_t c = vc * sizeof(double);
    
    // copy the data. we use 3 calls to memcpy to avoid problems of non-contiguous blocks at source & destination
    memcpy(position_, other.position_, c);
    if (other.velocity_)
    {
      velocity_ = position_ + vc;
      memcpy(velocity_, other.velocity_, c);
    }
    if (other.acceleration_)
    {
      acceleration_ = velocity_ + vc;
      memcpy(acceleration_, other.acceleration_, c);
    }
  }
  
  // copy attached bodies
  clearAttachedBodies();
  for (std::map<std::string, AttachedBody*>::const_iterator it = other.attached_body_map_.begin() ; it != other.attached_body_map_.end() ; ++it)
    attachBody(it->second->getName(), it->second->getShapes(), it->second->getFixedTransforms(),
               it->second->getTouchLinks(), it->second->getAttachedLinkName(), it->second->getDetachPosture());
}

void moveit::core::RobotState::setVariablePositions(const std::map<std::string, double> &variable_map)
{
  for (std::map<std::string, double>::const_iterator it = variable_map.begin(), end = variable_map.end() ; it != end ; ++it)
  {
    int index = robot_model_->getVariableIndex(it->first);
    position_[index] = it->second;
    updateMimicPosition(index);
    dirtyFK(index);
  }
}

void moveit::core::RobotState::getMissingKeys(const std::map<std::string, double> &variable_map, std::vector<std::string> &missing_variables) const
{
  missing_variables.clear();
  const std::vector<std::string> &nm = robot_model_->getVariableNames();
  for (std::size_t i = 0 ; i < nm.size() ; ++i)
    if (variable_map.find(nm[i]) == variable_map.end())
      if (robot_model_->getJointOfVariable(nm[i])->getMimic() == NULL)
        missing_variables.push_back(nm[i]);
}

void moveit::core::RobotState::setVariablePositions(const std::map<std::string, double> &variable_map, std::vector<std::string> &missing_variables)
{
  setVariablePositions(variable_map);
  getMissingKeys(variable_map, missing_variables);
}

void moveit::core::RobotState::setVariablePositions(const std::vector<std::string>& variable_names, const std::vector<double>& variable_position)
{
  for (std::size_t i = 0 ; i < variable_names.size() ; ++i)
  { 
    int index = robot_model_->getVariableIndex(variable_names[i]);
    position_[index] = variable_position[i];  
    updateMimicPosition(index);
    dirtyFK(index);
  }
}

void moveit::core::RobotState::setVariableVelocities(const std::map<std::string, double> &variable_map)
{
  allocVelocity();
  for (std::map<std::string, double>::const_iterator it = variable_map.begin(), end = variable_map.end() ; it != end ; ++it)
    velocity_[robot_model_->getVariableIndex(it->first)] = it->second;
}

void moveit::core::RobotState::setVariableVelocities(const std::map<std::string, double> &variable_map, std::vector<std::string>& missing_variables)
{
  setVariableVelocities(variable_map);
  getMissingKeys(variable_map, missing_variables);
}

void moveit::core::RobotState::setVariableVelocities(const std::vector<std::string>& variable_names, const std::vector<double>& variable_velocity)
{
  assert(variable_names.size() == variable_velocity.size());
  allocVelocity();
  for (std::size_t i = 0 ; i < variable_names.size() ; ++i)
    velocity_[robot_model_->getVariableIndex(variable_names[i])] = variable_velocity[i];  
}

void moveit::core::RobotState::setVariableAccelerations(const std::map<std::string, double> &variable_map)
{
  allocAcceleration();
  for (std::map<std::string, double>::const_iterator it = variable_map.begin(), end = variable_map.end() ; it != end ; ++it)
    acceleration_[robot_model_->getVariableIndex(it->first)] = it->second;
}

void moveit::core::RobotState::setVariableAccelerations(const std::map<std::string, double> &variable_map, std::vector<std::string>& missing_variables)
{
  setVariableAccelerations(variable_map);
  getMissingKeys(variable_map, missing_variables);
}

void moveit::core::RobotState::setVariableAccelerations(const std::vector<std::string>& variable_names, const std::vector<double>& variable_acceleration)
{
  assert(variable_names.size() == variable_acceleration.size());
  allocAcceleration();
  for (std::size_t i = 0 ; i < variable_names.size() ; ++i)
    acceleration_[robot_model_->getVariableIndex(variable_names[i])] = variable_acceleration[i];  
}

void moveit::core::RobotState::setJointGroupPositions(const JointModelGroup *group, const double *gstate)
{
  const std::vector<int> &il = group->getVariableIndexList();
  if (group->isContiguousWithinState())
    memcpy(position_ + il[0], gstate, group->getVariableCount() * sizeof(double));
  else
  {
    for (std::size_t i = 0 ; i < il.size() ; ++i)
      position_[il[i]] = gstate[i];
  }
  const std::vector<const JointModel*> &mimic = group->getMimicJointModels();
  for (std::size_t i = 0 ; i < mimic.size() ; ++i)
    updateMimicJoint(mimic[i]);
  dirtyFK(group->getCommonRoot());
}

void moveit::core::RobotState::copyJointGroupPositions(const JointModelGroup *group, double *gstate) const
{
  const std::vector<int> &il = group->getVariableIndexList();
  if (group->isContiguousWithinState())
    memcpy(gstate, position_ + il[0], group->getVariableCount() * sizeof(double));
  else
    for (std::size_t i = 0 ; i < il.size() ; ++i)
      gstate[i] = position_[il[i]];
}

void moveit::core::RobotState::updateCollisionBodyTransforms()
{
  if (dirty_fk_ != NULL)
  {
    updateJointTransforms();
    updateLinkTransforms();
  }
  else
    if (dirty_link_transforms_ != NULL)
      updateLinkTransforms();
  
  if (dirty_collision_body_transforms_ != NULL)
  {
    const std::vector<const LinkModel*> &links = dirty_collision_body_transforms_->getDescendantLinkModels();
    dirty_collision_body_transforms_ = NULL;
    for (std::size_t i = 0 ; i < links.size() ; ++i)
    {
      const EigenSTL::vector_Affine3d &ot = links[i]->getCollisionOriginTransforms();
      int index_co = links[i]->getFirstCollisionBodyTransformIndex();
      int index_l = links[i]->getLinkIndex();
      for (std::size_t j = 0 ; j < ot.size() ; ++j)
        global_collision_body_transforms_[index_co + j] = global_link_transforms_[index_l] * ot[j];
    }
  }
}

void moveit::core::RobotState::updateLinkTransforms()
{
  if (dirty_fk_ != NULL)
    updateJointTransforms(); // resets dirty_fk_, makes sure memory is allocated for transforms
  if (dirty_link_transforms_ != NULL)
  {
    updateLinkTransformsInternal(dirty_link_transforms_);
    if (dirty_collision_body_transforms_)
      dirty_collision_body_transforms_ = robot_model_->getCommonRoot(dirty_collision_body_transforms_, dirty_link_transforms_);
    else
      dirty_collision_body_transforms_ = dirty_link_transforms_;
    dirty_link_transforms_ = NULL;
  }
}

void moveit::core::RobotState::updateLinkTransformsInternal(const JointModel *start)
{
  const std::vector<const LinkModel*> &links = start->getDescendantLinkModels();
  for (std::size_t i = 0 ; i < links.size() ; ++i)
  {
    const LinkModel *parent = links[i]->getParentJointModel()->getParentLinkModel();
    int index_j = links[i]->getParentJointModel()->getJointIndex();
    int index_l = links[i]->getLinkIndex();
    if (parent)
      global_link_transforms_[index_l] = global_link_transforms_[parent->getLinkIndex()] * links[i]->getJointOriginTransform() * variable_joint_transforms_[index_j];
    else
      global_link_transforms_[index_l] = links[i]->getJointOriginTransform() * variable_joint_transforms_[index_j];
  }
  
  // update attached bodies tf; these are usually very few, so we update them all
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin() ; it != attached_body_map_.end() ; ++it)
    it->second->computeTransform(global_link_transforms_[it->second->getAttachedLink()->getLinkIndex()]);
}

void moveit::core::RobotState::updateJointTransforms()
{
  if (dirty_fk_ != NULL)
  {
    const std::vector<const JointModel*> &joint_models = dirty_fk_->getDescendantJointModels();
    if (dirty_link_transforms_)
      dirty_link_transforms_ = robot_model_->getCommonRoot(dirty_fk_, dirty_link_transforms_);
    else
      dirty_link_transforms_ = dirty_fk_;
    if (!variable_joint_transforms_)
      allocTransforms();
    int index = dirty_fk_->getJointIndex();
    int vindex = dirty_fk_->getFirstVariableIndex();
    dirty_fk_->computeTransform(position_ + vindex, variable_joint_transforms_[index]);
    dirty_fk_ = NULL;
    
    for (std::size_t i = 0 ; i < joint_models.size() ; ++i)
    {
      index = joint_models[i]->getJointIndex();
      vindex = joint_models[i]->getFirstVariableIndex();
      joint_models[i]->computeTransform(position_ + vindex, variable_joint_transforms_[index]);
    }
  }
}

void moveit::core::RobotState::updateStateWithLinkAt(const LinkModel *link, const Eigen::Affine3d& transform, bool backward)
{
  updateLinkTransforms(); // no link transforms must be dirty, otherwise the transform we set will be overwritten
  
  // update the fact that collision body transforms are out of date
  if (dirty_collision_body_transforms_)
    dirty_collision_body_transforms_ = robot_model_->getCommonRoot(dirty_collision_body_transforms_, link->getParentJointModel());
  else
    dirty_collision_body_transforms_ = link->getParentJointModel();
  
  global_link_transforms_[link->getLinkIndex()] = transform;

  // update link transforms for descendant links only (leaving the transform for the current link untouched)
  const std::vector<const JointModel*> &cj = link->getChildJointModels();
  for (std::size_t i = 0 ; i < cj.size() ; ++i)
    updateLinkTransformsInternal(cj[i]);
  
  // if we also need to go backward
  if (backward)
  {
    const LinkModel *parent_link = link;
    const LinkModel *child_link;
    while (parent_link->getParentJointModel()->getParentLinkModel())
    {
      child_link = parent_link;
      parent_link = parent_link->getParentJointModel()->getParentLinkModel();

      // update the transform of the parent
      global_link_transforms_[parent_link->getLinkIndex()] = global_link_transforms_[child_link->getLinkIndex()] *
        (child_link->getJointOriginTransform() * variable_joint_transforms_[child_link->getParentJointModel()->getJointIndex()]).inverse();

      // update link transforms for descendant links only (leaving the transform for the current link untouched)
      // with the exception of the child link we are coming backwards from
      const std::vector<const JointModel*> &cj = parent_link->getChildJointModels();
      for (std::size_t i = 0 ; i < cj.size() ; ++i)
        if (cj[i] != child_link->getParentJointModel())
          updateLinkTransformsInternal(cj[i]);
    }
    // update the root joint of the model to match (as best as possible given #DOF) the transfor we wish to obtain for the root link.
    // but I am disabling this code, since I do not think this function should modify variable values.
    //    parent_link->getParentJointModel()->computeVariableValues(global_link_transforms_[parent_link->getLinkIndex()],
    //                                                              position_ + parent_link->getParentJointModel()->getFirstVariableIndex());
  }
  
  // update attached bodies tf; these are usually very few, so we update them all
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin() ; it != attached_body_map_.end() ; ++it)
    it->second->computeTransform(global_link_transforms_[it->second->getAttachedLink()->getLinkIndex()]);
}

bool moveit::core::RobotState::satisfiesBounds(double margin) const
{
  const std::vector<const JointModel*> &jm = robot_model_->getJointModels();
  for (std::size_t i = 0 ; i < jm.size() ; ++i)
    if (!satisfiesBounds(jm[i], margin))
      return false;
  return true;
}

bool moveit::core::RobotState::satisfiesBounds(const JointModelGroup *group, double margin) const
{
  const std::vector<const JointModel*> &jm = group->getJointModels();
  for (std::size_t i = 0 ; i < jm.size() ; ++i)
    if (!satisfiesBounds(jm[i], margin))
      return false;
  return true;  
}

void moveit::core::RobotState::enforceBounds()
{
  const std::vector<const JointModel*> &jm = robot_model_->getJointModels();
  for (std::size_t i = 0 ; i < jm.size() ; ++i)
    enforceBounds(jm[i]);
}

void moveit::core::RobotState::enforceBounds(const JointModelGroup *joint_group)
{
  const std::vector<const JointModel*> &jm = joint_group->getJointModels();
  for (std::size_t i = 0 ; i < jm.size() ; ++i)
    enforceBounds(jm[i]);
}

double moveit::core::RobotState::distance(const double *state_position) const
{
  double d = 0.0;
  const std::vector<const JointModel*> &jm = robot_model_->getJointModels();
  for (std::size_t i = 0 ; i < jm.size() ; ++i)
  {
    const int idx = jm[i]->getFirstVariableIndex();
    d += jm[i]->getDistanceFactor() * jm[i]->distance(position_ + idx, state_position + idx);
  }
  return d;
}

double moveit::core::RobotState::distance(const RobotState &other, const JointModelGroup *joint_group) const
{
  double d = 0.0;
  const std::vector<const JointModel*> &jm = joint_group->getJointModels();
  for (std::size_t i = 0 ; i < jm.size() ; ++i)
  {
    const int idx = jm[i]->getFirstVariableIndex();
    d += jm[i]->getDistanceFactor() * jm[i]->distance(position_ + idx, other.position_ + idx);
  }
  return d;  
}

double moveit::core::RobotState::distance(const double *joint_group_position, const JointModelGroup *joint_group) const
{
  double d = 0.0;
  const std::vector<const JointModel*> &jm = joint_group->getJointModels();
  unsigned int i2 = 0;
  for (std::size_t i = 0 ; i < jm.size() ; ++i)
  {
    const int idx = jm[i]->getFirstVariableIndex();
    d += jm[i]->getDistanceFactor() * jm[i]->distance(position_ + idx, joint_group_position + i2);
    i2 += jm[i]->getVariableCount();
  }
  return d;
}

void moveit::core::RobotState::interpolate(const double *to, double t, double *state) const
{
  // we interpolate values only for active joint models (non-mimic)
  const std::vector<const JointModel*> &jm = robot_model_->getActiveJointModels();
  for (std::size_t i = 0 ; i < jm.size() ; ++i)
  {
    const int idx = jm[i]->getFirstVariableIndex();
    jm[i]->interpolate(position_ + idx, to + idx, t, state + idx);
  }
  // now we update mimic as needed
  robot_model_->updateMimicJoints(state);
}

void moveit::core::RobotState::interpolate(const double *to, double t, double *state, const JointModelGroup *joint_group) const
{
  // we use the mimic joints here as well (not just the active joints)
  // because there may be updates to mimic joints we cannot do (the parent joint values are not part of the group)
  const std::vector<const JointModel*> &jm = joint_group->getJointModels();
  unsigned int i2 = 0;
  for (std::size_t i = 0 ; i < jm.size() ; ++i)
  {
    const int idx = jm[i]->getFirstVariableIndex();
    jm[i]->interpolate(position_ + idx, to + i2, t, state + i2);
    i2 += jm[i]->getVariableCount();
  }

  // update mimic (only local joints as we are dealing with a local group state)
  joint_group->updateMimicJoints(state);
}

void moveit::core::RobotState::interpolate(const RobotState &to, double t, RobotState &state, const JointModelGroup *joint_group) const
{
  const std::vector<const JointModel*> &jm = joint_group->getActiveJointModels();
  for (std::size_t i = 0 ; i < jm.size() ; ++i)
  {
    const int idx = jm[i]->getFirstVariableIndex();
    jm[i]->interpolate(position_ + idx, to.position_ + idx, t, state.position_ + idx);
  }
  const std::vector<const JointModel*> &mimic = joint_group->getMimicJointModels();
  for (std::size_t i = 0 ; i < mimic.size() ; ++i)
    state.updateMimicJoint(mimic[i]);
  state.dirtyFK(joint_group->getCommonRoot());
}

void moveit::core::RobotState::setAttachedBodyUpdateCallback(const AttachedBodyCallback &callback)
{
  attached_body_update_callback_ = callback;
}

bool moveit::core::RobotState::hasAttachedBody(const std::string &id) const
{
  return attached_body_map_.find(id) != attached_body_map_.end();
}

const moveit::core::AttachedBody* moveit::core::RobotState::getAttachedBody(const std::string &id) const
{
  std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.find(id);
  if (it == attached_body_map_.end())
  {
    logError("Attached body '%s' not found", id.c_str());
    return NULL;
  }
  else
    return it->second;
}

void moveit::core::RobotState::attachBody(AttachedBody *attached_body)
{
  attached_body_map_[attached_body->getName()] = attached_body;
  attached_body->computeTransform(getGlobalLinkTransform(attached_body->getAttachedLink()));
  if (attached_body_update_callback_)
    attached_body_update_callback_(attached_body, true);
}

void moveit::core::RobotState::attachBody(const std::string &id,
                                          const std::vector<shapes::ShapeConstPtr> &shapes,
                                          const EigenSTL::vector_Affine3d &attach_trans,
                                          const std::set<std::string> &touch_links,
                                          const std::string &link,
                                          const sensor_msgs::JointState &detach_posture)
{
  const LinkModel *l = robot_model_->getLinkModel(link);
  AttachedBody *ab = new AttachedBody(l, id, shapes, attach_trans, touch_links, detach_posture);
  attached_body_map_[id] = ab;
  ab->computeTransform(getGlobalLinkTransform(l));
  if (attached_body_update_callback_)
    attached_body_update_callback_(ab, true);
}

void moveit::core::RobotState::getAttachedBodies(std::vector<const AttachedBody*> &attached_bodies) const
{
  attached_bodies.clear();
  attached_bodies.reserve(attached_body_map_.size());
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin() ; it != attached_body_map_.end() ;  ++it)
    attached_bodies.push_back(it->second);
}

void moveit::core::RobotState::clearAttachedBodies()
{
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin() ; it != attached_body_map_.end() ;  ++it)
  {
    if (attached_body_update_callback_)
      attached_body_update_callback_(it->second, false);
    delete it->second;
  }
  attached_body_map_.clear();
}

void moveit::core::RobotState::clearAttachedBodies(const LinkModel *link)
{
  std::map<std::string, AttachedBody*>::iterator it = attached_body_map_.begin();
  while (it != attached_body_map_.end())
  {
    if (it->second->getAttachedLink() != link)
    {
      ++it;
      continue;
    }
    if (attached_body_update_callback_)
      attached_body_update_callback_(it->second, false);
    delete it->second;
    std::map<std::string, AttachedBody*>::iterator del = it++;
    attached_body_map_.erase(del);
  }
}

void moveit::core::RobotState::clearAttachedBodies(const JointModelGroup *group)
{
  std::map<std::string, AttachedBody*>::iterator it = attached_body_map_.begin();
  while (it != attached_body_map_.end())
  {
    if (!group->hasLinkModel(it->second->getAttachedLinkName()))
    {
      ++it;
      continue;
    }
    if (attached_body_update_callback_)
      attached_body_update_callback_(it->second, false);
    delete it->second;
    std::map<std::string, AttachedBody*>::iterator del = it++;
    attached_body_map_.erase(del);
  }
}

bool moveit::core::RobotState::clearAttachedBody(const std::string &id)
{
  std::map<std::string, AttachedBody*>::iterator it = attached_body_map_.find(id);
  if (it != attached_body_map_.end())
  {
    if (attached_body_update_callback_)
      attached_body_update_callback_(it->second, false);
    delete it->second;
    attached_body_map_.erase(it);
    return true;
  }
  else
    return false;
}

const Eigen::Affine3d& moveit::core::RobotState::getFrameTransform(const std::string &id)
{
  updateLinkTransforms();
  return const_cast<RobotState*>(this)->getFrameTransform(id);
}

const Eigen::Affine3d& moveit::core::RobotState::getFrameTransform(const std::string &id) const
{
  if (!id.empty() && id[0] == '/')
    return getFrameTransform(id.substr(1));
  static const Eigen::Affine3d identity_transform = Eigen::Affine3d::Identity();
  if (id.size() + 1 == robot_model_->getModelFrame().size() && '/' + id == robot_model_->getModelFrame())
    return identity_transform;
  if (robot_model_->hasLinkModel(id))
  {
    const LinkModel *lm = robot_model_->getLinkModel(id);
    return global_link_transforms_[lm->getLinkIndex()];
  }
  std::map<std::string, AttachedBody*>::const_iterator jt = attached_body_map_.find(id);
  if (jt == attached_body_map_.end())
  {
    logError("Transform from frame '%s' to frame '%s' is not known ('%s' should be a link name or an attached body id).",
             id.c_str(), robot_model_->getModelFrame().c_str(), id.c_str());
    return identity_transform;
  }
  const EigenSTL::vector_Affine3d &tf = jt->second->getGlobalCollisionBodyTransforms();
  if (tf.empty())
  {
    logError("Attached body '%s' has no geometry associated to it. No transform to return.", id.c_str());
    return identity_transform;
  }
  if (tf.size() > 1)
    logWarn("There are multiple geometries associated to attached body '%s'. Returning the transform for the first one.", id.c_str());
  return tf[0];
}

bool moveit::core::RobotState::knowsFrameTransform(const std::string &id) const
{
  if (!id.empty() && id[0] == '/')
    return knowsFrameTransform(id.substr(1));
  if (robot_model_->hasLinkModel(id))
    return true;
  std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.find(id);
  return it != attached_body_map_.end() && it->second->getGlobalCollisionBodyTransforms().size() == 1;
}

void moveit::core::RobotState::getRobotMarkers(visualization_msgs::MarkerArray& arr,
                                               const std::vector<std::string> &link_names,
                                               const std_msgs::ColorRGBA& color,
                                               const std::string& ns,
                                               const ros::Duration& dur,
                                               bool include_attached) const
{
  std::size_t cur_num = arr.markers.size();
  getRobotMarkers(arr, link_names, include_attached);
  unsigned int id = cur_num;
  for (std::size_t i = cur_num ; i < arr.markers.size() ; ++i, ++id)
  {
    arr.markers[i].ns = ns;
    arr.markers[i].id = id;
    arr.markers[i].lifetime = dur;
    arr.markers[i].color = color;
  }
}

void moveit::core::RobotState::getRobotMarkers(visualization_msgs::MarkerArray& arr, const std::vector<std::string> &link_names, bool include_attached) const
{
  ros::Time tm = ros::Time::now();
  for (std::size_t i = 0; i < link_names.size(); ++i)
  {
    logDebug("Trying to get marker for link '%s'", link_names[i].c_str());
    const LinkModel* lm = robot_model_->getLinkModel(link_names[i]);
    if (!lm)
      continue;
    if (include_attached)
      for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin() ; it != attached_body_map_.end() ; ++it)
        if (it->second->getAttachedLink() == lm)
        {
          for (std::size_t j = 0 ; j < it->second->getShapes().size() ; ++j)
          {
            visualization_msgs::Marker att_mark;
            att_mark.header.frame_id = robot_model_->getModelFrame();
            att_mark.header.stamp = tm;
            if (shapes::constructMarkerFromShape(it->second->getShapes()[j].get(), att_mark))
            {
              // if the object is invisible (0 volume) we skip it
              if (fabs(att_mark.scale.x * att_mark.scale.y * att_mark.scale.z) < std::numeric_limits<float>::epsilon())
                continue;
              tf::poseEigenToMsg(it->second->getGlobalCollisionBodyTransforms()[j], att_mark.pose);
              arr.markers.push_back(att_mark);
            }
          }
        }
    
    if (lm->getShapes().empty())
      continue;

    for (std::size_t j = 0 ; j < lm->getShapes().size() ; ++j)
    {
      visualization_msgs::Marker mark;
      mark.header.frame_id = robot_model_->getModelFrame();
      mark.header.stamp = tm;
      
      // we prefer using the visual mesh, if a mesh is available and we have one body to render
      const std::string& mesh_resource = lm->getVisualMeshFilename();
      if (mesh_resource.empty() || lm->getShapes().size() > 1)
      {
        if (!shapes::constructMarkerFromShape(lm->getShapes()[j].get(), mark))
          continue;
        // if the object is invisible (0 volume) we skip it
        if (fabs(mark.scale.x * mark.scale.y * mark.scale.z) < std::numeric_limits<float>::epsilon())
          continue;
      }
      else
      {
        mark.type = mark.MESH_RESOURCE;
        mark.mesh_use_embedded_materials = false;
        mark.mesh_resource = mesh_resource;
        const Eigen::Vector3d &mesh_scale = lm->getVisualMeshScale();
        
        mark.scale.x = mesh_scale[0];
        mark.scale.y = mesh_scale[1];
        mark.scale.z = mesh_scale[2];
      }
      tf::poseEigenToMsg(global_collision_body_transforms_[lm->getFirstCollisionBodyTransformIndex() + j], mark.pose);
      arr.markers.push_back(mark);
    }
  }
}


void moveit::core::RobotState::printStateInfo(std::ostream &out) const
{
  out << "Robot State @" << this << std::endl;
  out << "  * Memory is available for" << (position_ ? " position" : "")
      << (velocity_ ? " velocity" : "") <<  (acceleration_ ? " acceleration" : "")
      << (variable_joint_transforms_ ? " transforms" : "") << std::endl;
  out << "  * Called new[] for" << (position_ ? " position" : "")
      << (called_new_for_ & ALLOC_VELOCITY ? " velocity" : "")
      << (called_new_for_ & ALLOC_ACCELERATION ? " acceleration" : "")
      << (transforms_.size() > 0 ? " transforms" : "") << std::endl;
  
  std::size_t n = robot_model_->getVariableCount();
  if (position_)
  {
    out << "  * Position: ";
    for (std::size_t i = 0 ; i < n ; ++i)
      out << position_[i] << " ";
    out << std::endl;
  }
  else
    out << "  * Position: NULL" << std::endl;
  
  if (velocity_)
  {
    out << "  * Velocity: ";
    for (std::size_t i = 0 ; i < n ; ++i)
      out << velocity_[i] << " ";
    out << std::endl;
  }
  else
    out << "  * Velocity: NULL" << std::endl;

  if (acceleration_)
  {
    out << "  * Acceleration: ";
    for (std::size_t i = 0 ; i < n ; ++i)
      out << acceleration_[i] << " ";
    out << std::endl;
  }
  else
    out << "  * Acceleration: NULL" << std::endl;
  
  out << "  * Dirty FK: " << (dirty_fk_ ? dirty_fk_->getName() : "NULL") << std::endl;
  out << "  * Dirty Link Transforms: " << (dirty_link_transforms_ ? dirty_link_transforms_->getName() : "NULL") << std::endl;
  out << "  * Dirty Collision Body Transforms: " << (dirty_collision_body_transforms_ ? dirty_collision_body_transforms_->getName() : "NULL") << std::endl;
}

void moveit::core::RobotState::printTransform(const Eigen::Affine3d &transform, std::ostream &out) const
{
  Eigen::Quaterniond q(transform.rotation());  
  out << "T.xyz = [" << transform.translation().x() << ", " << transform.translation().y() << ", " << transform.translation().z() << "], Q.xyzw = ["
      << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << "]" << std::endl;
}

void moveit::core::RobotState::printTransforms(std::ostream &out) const
{
  if (!variable_joint_transforms_)
  {
    out << "No transforms computed" << std::endl;
    return;
  }
  
  out << "Joint transforms:" << std::endl;
  const std::vector<const JointModel*> &jm = robot_model_->getJointModels();
  for (std::size_t i = 0 ; i < jm.size() ; ++i)
  {
    out << "  " << jm[i]->getName() << ": ";
    printTransform(variable_joint_transforms_[jm[i]->getJointIndex()], out);
  }
  
  out << "Link poses:" << std::endl;
  const std::vector<const LinkModel*> &lm = robot_model_->getLinkModels();
  for (std::size_t i = 0 ; i < lm.size() ; ++i)
  {
    out << "  " << lm[i]->getName() << ": ";
    printTransform(global_link_transforms_[lm[i]->getLinkIndex()], out);
  }
}

std::string moveit::core::RobotState::getStateTreeString(const std::string& prefix) const
{
  std::stringstream ss;
  ss << "ROBOT: " << robot_model_->getName() << std::endl;
  getStateTreeJointString(ss, robot_model_->getRootJoint(), "   ", true);
  return ss.str();
}

namespace
{
void getPoseString(std::ostream& ss, const Eigen::Affine3d& pose, const std::string& pfx)
{
  ss.precision(3);
  for (int y = 0 ; y < 4 ; ++y)
  {
    ss << pfx;
    for (int x = 0 ; x < 4 ; ++x)
    {
      ss << std::setw(8) << pose(y, x) << " ";
    }
    ss << std::endl;
  }
}
}

void moveit::core::RobotState::getStateTreeJointString(std::ostream& ss, const JointModel* jm, const std::string& pfx0, bool last) const
{
  std::string pfx = pfx0 + "+--";
  
  ss << pfx << "Joint: " << jm->getName() << std::endl;

  pfx = pfx0 + (last ? "   " : "|  ");

  for (std::size_t i = 0 ; i < jm->getVariableCount(); ++i)
  {
    ss.precision(3);
    ss << pfx << jm->getVariableNames()[i] << std::setw(12) << position_[jm->getFirstVariableIndex() + i] << std::endl;
  }

  const LinkModel* lm = jm->getChildLinkModel();

  ss << pfx << "Link: " << lm->getName() << std::endl;
  getPoseString(ss, lm->getJointOriginTransform(), pfx + "joint_origin:");
  if (variable_joint_transforms_)
  {
    getPoseString(ss, variable_joint_transforms_[jm->getJointIndex()], pfx + "joint_variable:");
    getPoseString(ss, global_link_transforms_[lm->getLinkIndex()], pfx + "link_global:");
  }
  
  for (std::vector<const JointModel*>::const_iterator it = lm->getChildJointModels().begin() ; it != lm->getChildJointModels().end() ; ++it)
    getStateTreeJointString(ss, *it, pfx, it + 1 == lm->getChildJointModels().end());
}


