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
    const std::vector<const LinkModel*> &links = dirty_link_transforms_->getDescendantLinkModels();
    if (dirty_collision_body_transforms_)
      dirty_collision_body_transforms_ = robot_model_->getCommonRoot(dirty_collision_body_transforms_, dirty_link_transforms_);
    else
      dirty_collision_body_transforms_ = dirty_link_transforms_;
    dirty_link_transforms_ = NULL;
    
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
    
    // update attached bodies tf
  }
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
