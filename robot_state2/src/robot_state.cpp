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

moveit::core::RobotState::RobotState(const RobotModelConstPtr &robot_model, AllocComponents alloc_components) : 
    robot_model_(robot_model),
    called_new_for_(ALLOC_POSITION),
    position_(new double[robot_model->getVariableCount() * 
			 (1 + (alloc_components & (ALLOC_VELOCITY | ALLOC_ACCELERATION) ? 1 : 0) + 
			  (alloc_components & ALLOC_ACCELERATION ? 1 : 0))]),
    velocity_(alloc_components & (ALLOC_VELOCITY | ALLOC_ACCELERATION) ? position_ + robot_model->getVariableCount() : NULL),
    acceleration_(alloc_components & ALLOC_ACCELERATION ? velocity_ + robot_model->getVariableCount() : NULL),
    variable_joint_transforms_(NULL),
    global_link_transforms_(NULL),
    global_collision_body_transforms_(NULL),
    dirty_fk_(0)
{
  if (alloc_components & ALLOC_TRANSFORMS)
    allocTransforms();
}

moveit::core::RobotState(const RobotState &other)
{
  if (this != &other)
    copyFrom(other);
}

moveit::core::RobotState::~RobotState()
{
  delete[] position_;
  if (called_new_for_ & ALLOC_VELOCITY)
    delete[] velocity_;
  if (called_new_for_ & ALLOC_ACCELERATION)
    delete[] acceleration_;
  if (variable_joint_transforms_)
    delete[] variable_joint_transforms_;
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
    if (other.dirty_fk_ < 0)
    {
      transforms_ = other.transforms_;
      dirty_fk_ = -1;
    }
    else
      // otherwise, we will just assume everything is dirty and re-compute when needed
      dirty_fk_ = 0;
  }
  else
    // no transforms to copy, so everything is dirty
    dirty_fk_ = 0;
  
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
    std::size_t c = robot_model_->getVariableCount();
    position_ = new double[(1 + (other.velocity_ ? 1 : 0) + (other.acceleration_ ? 1 : 0)) * c];
    c *= sizeof(double);
    
    // copy the data. we use 3 calls to memcpy to avoid problems of non-contiguous blocks at source & destination
    memcpy(position_, other.position_, c);
    if (other.velocity_)
      memcpy(velocity_, other.velocity_, c);
    if (other.acceleration_)
      memcpy(acceleration_, other.acceleration_, c);
  }
  
  // copy attached bodies
  
}
