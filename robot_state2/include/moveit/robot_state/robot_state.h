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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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

#ifndef MOVEIT_CORE_ROBOT_STATE_
#define MOVEIT_CORE_ROBOT_STATE_

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/attached_body.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>

namespace moveit
{
namespace core
{

MOVEIT_CLASS_FORWARD(RobotState); 

class RobotState
{
public:
  
  enum AllocComponents
    {
      ALLOC_POSITION = 1,
      ALLOC_VELOCITY = 2,
      ALLOC_ACCELERATION = 4,
      ALLOC_TRANSFORMS = 8,
      ALLOC_POSITION_AND_VELOCITY = ALLOC_POSITION | ALLOC_VELOCITY,
      ALLOC_ALL = ALLOC_POSITION | ALLOC_VELOCITY | ALLOC_ACCELERATION | ALLOC_TRANSFORMS
    };
  
  RobotState(const RobotModelConstPtr &robot_model, AllocComponents alloc_components = ALLOC_POSITION);  
  ~RobotState();
  
  RobotState(const RobotState &other);

  RobotState& operator=(const RobotState &other);

  const RobotModelConstPtr& getRobotModel() const
  {
    return robot_model_;
  }
   
  std::size_t getVariableCount() const
  {
    return robot_model_->getVariableCount();
  }
  
  const std::vector<std::string>& getVariableNames() const
  {
    return robot_model_->getVariableNames();
  }
  
  /** \defgroup setVariablePosition_Fn Getting and setting variable position
   *  @{
   */

  double* getVariablePositions()
  {
    return position_;
  }
  
  const double* getVariablePositions() const
  {
    return position_;
  }
  
  void setVariablePositions(const double *position)
  {
    // assume everything is in order in terms of array lengths (for efficiency reasons)
    memcpy(position_, position, robot_model_->getVariableCount() * sizeof(double));
    
    // the full state includes mimic joint values, so no need to update mimic here
    
    // the index of the root joint of the system is 0. Since all joint values have potentially
    // changed, we will need to do FK from the root (if FK is ever required)
    dirty_fk_ = NULL;
  }
  
  void setVariablePositions(const std::vector<double> &position)
  {
    assert(robot_model_->getVariableCount() <= position.size()); // checked only in debug mode
    setVariablePositions(&position[0]);
  }
  
  void setVariablePositions(const std::map<std::string, double> &variable_map);
  void setVariablePositions(const std::map<std::string, double> &variable_map, std::vector<std::string> &missing_variables);
  void setVariablePositions(const std::vector<std::string>& variable_names, const std::vector<double>& variable_position);
  
  void setVariablePosition(const std::string &variable, double value)
  {
    setVariablePosition(robot_model_->getVariableIndex(variable), value);
  }
  
  void setVariablePosition(int index, double value)
  {
    position_[index] = value;
    updateMimicPosition(index);
    dirtyFK(index);
  }
  
  const double getVariablePosition(const std::string &variable) const
  {
    return position_[robot_model_->getVariableIndex(variable)];
  }
  
  const double getVariablePosition(int index) const
  {
    return position_[index];
  }
  
  /** @} */

  /** \defgroup setVariableVelocity_Fn Getting and setting variable velocity
   *  @{
   */

  double* getVariableVelocities()
  {
    allocVelocity();
    return velocity_;
  }
  
  const double* getVariableVelocities() const
  {
    return velocity_;
  }
  
  void setVariableVelocities(const double *velocity)
  {
    allocVelocity();
    // assume everything is in order in terms of array lengths (for efficiency reasons)
    memcpy(velocity_, velocity, robot_model_->getVariableCount() * sizeof(double));
  }
  
  void setVariableVelocities(const std::vector<double> &velocity)
  {
    assert(robot_model_->getVariableCount() <= velocity.size()); // checked only in debug mode
    setVariableVelocities(&velocity[0]);
  }
  
  void setVariableVelocities(const std::map<std::string, double> &variable_map);
  void setVariableVelocities(const std::map<std::string, double> &variable_map, std::vector<std::string>& missing_variables);
  void setVariableVelocities(const std::vector<std::string>& variable_names, const std::vector<double>& variable_velocity);
  
  void setVariableVelocity(const std::string &variable, double value)
  {
    setVariableVelocity(robot_model_->getVariableIndex(variable), value);
  }
  
  void setVariableVelocity(int index, double value)
  {
    allocVelocity();
    velocity_[index] = value;
  }
  
  const double getVariableVelocity(const std::string &variable) const
  {
    return velocity_[robot_model_->getVariableIndex(variable)];
  }
  
  const double getVariableVelocity(int index) const
  {
    return velocity_[index];
  }
  
  /** @} */


  /** \defgroup setVariableAcceleration_Fn Getting and setting variable acceleration
   *  @{
   */

  double* getVariableAccelerations()
  {
    allocAcceleration();
    return acceleration_;
  }
  
  const double* getVariableAccelerations() const
  {
    return acceleration_;
  }
  
  void setVariableAccelerations(const double *acceleration)
  {
    allocAcceleration();
    // assume everything is in order in terms of array lengths (for efficiency reasons)
    memcpy(acceleration_, acceleration, robot_model_->getVariableCount() * sizeof(double));
  }
  
  void setVariableAccelerations(const std::vector<double> &acceleration)
  {
    assert(robot_model_->getVariableCount() <= acceleration.size()); // checked only in debug mode
    setVariableAccelerations(&acceleration[0]);
  }
  
  void setVariableAccelerations(const std::map<std::string, double> &variable_map);
  void setVariableAccelerations(const std::map<std::string, double> &variable_map, std::vector<std::string>& missing_variables);
  void setVariableAccelerations(const std::vector<std::string>& variable_names, const std::vector<double>& variable_acceleration);
  
  void setVariableAcceleration(const std::string &variable, double value)
  {
    setVariableAcceleration(robot_model_->getVariableIndex(variable), value);
  }
  
  void setVariableAcceleration(int index, double value)
  {
    allocAcceleration();
    acceleration_[index] = value;
  }
  
  const double getVariableAcceleration(const std::string &variable) const
  {
    return acceleration_[robot_model_->getVariableIndex(variable)];
  }
  
  const double getVariableAcceleration(int index) const
  {
    return acceleration_[index];
  }
  
  /** @} */

  
  /** \defgroup setJointPosition_Fn Getting and setting joint positions
   *  @{
   */
  void setJointPositions(const std::string &joint_name, const double *position)
  {  
    setJointPositions(robot_model_->getJointModel(joint_name), position);
  }
  
  void setJointPositions(const std::string &joint_name, const std::vector<double> &position)
  {
    setJointPositions(robot_model_->getJointModel(joint_name), &position[0]);
  }
  
  void setJointPositions(const JointModel *joint, const std::vector<double> &position)
  {
    setJointPositions(joint, &position[0]);
  }
  
  void setJointPositions(const JointModel *joint, const double *position)
  {  
    memcpy(position_ + joint->getFirstVariableIndex(), position, joint->getVariableCount() * sizeof(double));
    updateMimicJoint(joint);
    dirtyFK(joint);
  }
  
  void setJointPositions(const std::string &joint_name, const Eigen::Affine3d& transform)
  {
    setJointPositions(robot_model_->getJointModel(joint_name), transform);
  }
  
  void setJointPositions(const JointModel *joint, const Eigen::Affine3d& transform)
  {
    joint->computeVariableValues(transform, position_ + joint->getFirstVariableIndex());
    updateMimicJoint(joint);
    dirtyFK(joint);
  }
  
  const double* getJointPositions(const std::string &joint_name) const
  {
    return getJointPositions(robot_model_->getJointModel(joint_name));
  }
  
  const double* getJointPositions(const JointModel *joint) const
  {
    return position_ + joint->getFirstVariableIndex();
  }
  /** @} */
  
  
  /** \defgroup setGroupPosition_Fn Getting and setting group positions
   *  @{
   */
  
  void setJointGroupPositions(const std::string &joint_group_name, const double *gstate)
  {
    const JointModelGroup *jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      setJointGroupPositions(jmg, gstate);
  }
  
  void setJointGroupPositions(const std::string &joint_group_name, const std::vector<double> &gstate)
  {
    const JointModelGroup *jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      setJointGroupPositions(jmg, &gstate[0]);
  }
  
  void setJointGroupPositions(const JointModelGroup *group, const std::vector<double> &gstate)
  {
    setJointGroupPositions(group, &gstate[0]);
  }
  
  void setJointGroupPositions(const JointModelGroup *group, const double *gstate);

  void copyJointGroupPositions(const std::string &joint_group_name, std::vector<double> &gstate) const
  {
    const JointModelGroup *jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
    {
      gstate.resize(jmg->getVariableCount());
      copyJointGroupPositions(jmg, &gstate[0]);
    }
  }
  
  void copyJointGroupPositions(const std::string &joint_group_name, double *gstate) const
  {
    const JointModelGroup *jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      copyJointGroupPositions(jmg, gstate);
  }
  
  void copyJointGroupPositions(const JointModelGroup *group, std::vector<double> &gstate) const
  {
    gstate.resize(group->getVariableCount());
    copyJointGroupPositions(group, &gstate[0]);
  }
  
  void copyJointGroupPositions(const JointModelGroup *group, double *gstate) const;
  
  /** @} */

  /** \defgroup setGroupPosition_Fn Getting and setting whole states
   *  @{
   */
  
  void setVariableValues(const sensor_msgs::JointState& msg)
  {
    setVariablePositions(msg.name, msg.position);
    setVariableVelocities(msg.name, msg.velocity);
  }
  
  void setToDefaultValues()
  {
    robot_model_->getVariableDefaultValues(position_);
    dirty_fk_ = robot_model_->getRootJoint();
  }
  
  /** @} */
  
  /** \defgroup RobotStateGetTransforms Updating and getting transforms
   *  @{
   */
  
  /** \brief Update the transforms for the collision bodies. This call is needed before calling collision checking.
      If updating link transforms or joint transorms is needed, the corresponding updates are also triggered. */
  void updateCollisionBodyTransforms();
  
  /** \brief Update the reference frame transforms for links. This call is needed before using the transforms of links for coordinate transforms. */
  void updateLinkTransforms();
  
  /** \brief Update the transforms joints apply given the currently set joint values. */
  void updateJointTransforms();

  /** \brief Update all transforms. */
  void update(bool force = false)
  {
    // make sure we do everything from scratch if needed
    if (force)
      dirty_fk_ = robot_model_->getRootJoint();
    // this actually triggers all needed updates
    updateCollisionBodyTransforms();
  }
  
  /** \brief Update the state after setting a particular link to the input global transform pose.*/
  void updateStateWithLinkAt(const std::string& link_name, const Eigen::Affine3d& transform, bool backward = false)
  {
    updateStateWithLinkAt(robot_model_->getLinkModel(link_name), transform, backward);
  }

  /** \brief Update the state after setting a particular link to the input global transform pose.*/
  void updateStateWithLinkAt(const LinkModel *link, const Eigen::Affine3d& transform, bool backward = false);

  const Eigen::Affine3d& getGlobalLinkTransform(const std::string &link_name)
  {
    return getGlobalLinkTransform(robot_model_->getLinkModel(link_name));
  }
  
  const Eigen::Affine3d& getGlobalLinkTransform(const LinkModel *link)
  {
    updateLinkTransforms();
    return global_link_transforms_[link->getLinkIndex()];
  }
  
  const Eigen::Affine3d& getCollisionBodyTransforms(const std::string &link_name, std::size_t index)
  {
    return getCollisionBodyTransform(robot_model_->getLinkModel(link_name), index);
  }
  
  const Eigen::Affine3d& getCollisionBodyTransform(const LinkModel *link, std::size_t index)
  {
    updateCollisionBodyTransforms();
    return global_collision_body_transforms_[link->getFirstCollisionBodyTransformIndex() + index];
  }
  
  const Eigen::Affine3d& getJointTransform(const std::string &joint_name)
  {
    return getJointTransform(robot_model_->getJointModel(joint_name));
  }
  
  const Eigen::Affine3d& getJointTransform(const JointModel *joint)
  {
    updateJointTransforms();
    return variable_joint_transforms_[joint->getJointIndex()];
  }
  
  const Eigen::Affine3d& getGlobalLinkTransform(const std::string &link_name) const
  {
    return getGlobalLinkTransform(robot_model_->getLinkModel(link_name));
  }
  
  const Eigen::Affine3d& getGlobalLinkTransform(const LinkModel *link) const
  {
    return global_link_transforms_[link->getLinkIndex()];
  }
  
  const Eigen::Affine3d& getCollisionBodyTransform(const std::string &link_name, std::size_t index) const
  {
    return getCollisionBodyTransform(robot_model_->getLinkModel(link_name), index);
  }
  
  const Eigen::Affine3d& getCollisionBodyTransform(const LinkModel *link, std::size_t index) const
  {
    return global_collision_body_transforms_[link->getFirstCollisionBodyTransformIndex() + index];
  }
  
  const Eigen::Affine3d& getJointTransform(const std::string &joint_name) const
  {
    return getJointTransform(robot_model_->getJointModel(joint_name));
  }
  
  const Eigen::Affine3d& getJointTransform(const JointModel *joint) const
  {
    return variable_joint_transforms_[joint->getJointIndex()];
  }
  
  /** \defgroup distanceFunctions Computing distances
   *  @{
   */
  
  double distance(const RobotState &other) const
  {
    return distance(other.getVariablePositions());
  }
  
  double distance(const RobotState &other, const JointModel *joint) const
  {
    return distance(other.getJointPositions(joint), joint);
  }
  
  double distance(const RobotState &other, const JointModelGroup *joint_group) const;
  
  double distance(const double *state_position) const;
  double distance(const double *joint_position, const JointModel *joint) const;  
  double distance(const double *joint_group_position, const JointModelGroup *joint_group) const;
  
  void interpolate(const RobotState &to, double t, RobotState &state)
  {
    interpolate(to.getVariablePositions(), t, state.getVariablePositions());
  }
  
  void interpolate(const RobotState &to, double t, RobotState &state, const JointModel *joint)
  {
    interpolate(to.getJointPositions(joint), t, const_cast<double*>(state.getJointPositions(joint)), joint);
  }
  
  void interpolate(const RobotState &to, double t, RobotState &state, const JointModelGroup *joint_group);
  
  void interpolate(const double *to, double t, double *state);
  void interpolate(const double *to, double t, double *state, const JointModel *joint);
  void interpolate(const double *to, double t, double *state, const JointModelGroup *joint_group);
  
  void enforceBounds();
  void enforceBounds(const JointModel *joint)
  {
    if (joint->enforceBounds(position_ + joint->getFirstVariableIndex()))
      updateMimicJoint(joint);
  }
  
  void enforceBounds(const JointModelGroup *joint_group);
  
  bool satisfiesBounds(double margin = 0.0) const;
  bool satisfiesBounds(const JointModel *joint, double margin = 0.0) const
  {
    return joint->satisfiesBounds(getJointPositions(joint), margin);
  }
  
  bool satisfiesBounds(const JointModelGroup *joint_group, double margin = 0.0) const;
  
  /** @} */
  
  /** \defgroup RobotStateAttachedBodies Managing attached bodies
   *  @{
   */
  
  
  /** \brief Attach a body to this state. Ownership of the memory for the attached body is assumed by the state. */
  void attachBody(AttachedBody *attached_body);
  
  /**
     @brief Attach a body to a link
     @param id The string id associated with the attached body
     @param shapes The shapes that make up the attached body
     @param attach_trans The desired transform between this link and the attached body
     @param touch_links The set of links that the attached body is allowed to touch
     @param link_name The link to attach to
  */
  void attachBody(const std::string &id,
                  const std::vector<shapes::ShapeConstPtr> &shapes,
                  const EigenSTL::vector_Affine3d &attach_trans,
                  const std::set<std::string> &touch_links,
                  const std::string &link_name,
                  const sensor_msgs::JointState &detach_posture = sensor_msgs::JointState());

  /**
     @brief Attach a body to a link
     @param id The string id associated with the attached body
     @param shapes The shapes that make up the attached body
     @param attach_trans The desired transform between this link and the attached body
     @param touch_links The set of links that the attached body is allowed to touch
     @param link_name The link to attach to
  */
  void attachBody(const std::string &id,
                  const std::vector<shapes::ShapeConstPtr> &shapes,
                  const EigenSTL::vector_Affine3d &attach_trans,
                  const std::vector<std::string> &touch_links,
                  const std::string &link_name,
                  const sensor_msgs::JointState &detach_posture = sensor_msgs::JointState())
  {
    std::set<std::string> touch_links_set(touch_links.begin(), touch_links.end());
    attachBody(id, shapes, attach_trans, touch_links_set, link_name, detach_posture);
  }

  /** \brief Get all bodies attached to the model corresponding to this state */
  void getAttachedBodies(std::vector<const AttachedBody*> &attached_bodies) const;

  /** \brief Remove the attached body named \e id. Return false if the object was not found (and thus not removed). Return true on success. */
  bool clearAttachedBody(const std::string &id);

  /** \brief Clear the bodies attached to a specific link */
  void clearAttachedBodies(const LinkModel *link);

  /** \brief Clear the bodies attached to a specific group */
  void clearAttachedBodies(const JointModelGroup *group);

  /** \brief Clear all attached bodies. This calls delete on the AttachedBody instances, if needed. */
  void clearAttachedBodies();

  /** \brief Get the attached body named \e name. Return NULL if not found. */
  const AttachedBody* getAttachedBody(const std::string &name) const;

  /** \brief Check if an attached body named \e id exists in this state */
  bool hasAttachedBody(const std::string &id) const;

  void setAttachedBodyUpdateCallback(const AttachedBodyCallback &callback);
  /** @} */


  const Eigen::Affine3d& getFrameTransform(const std::string &id);
  const Eigen::Affine3d& getFrameTransform(const std::string &id) const;
  bool knowsFrameTransform(const std::string &id) const;
  
  /** @brief Get a MarkerArray that fully describes the robot markers for a given robot.
   *  @param arr The returned marker array
   *  @param link_names The list of link names for which the markers should be created.
   *  @param color The color for the marker
   *  @param ns The namespace for the markers
   *  @param dur The ros::Duration for which the markers should stay visible
   */
  void getRobotMarkers(visualization_msgs::MarkerArray& arr,
                       const std::vector<std::string> &link_names,
                       const std_msgs::ColorRGBA& color,
                       const std::string& ns,
                       const ros::Duration& dur,
                       bool include_attached = false) const;
  
  /** @brief Get a MarkerArray that fully describes the robot markers for a given robot.
   *  @param arr The returned marker array
   *  @param link_names The list of link names for which the markers should be created.
   */
  void getRobotMarkers(visualization_msgs::MarkerArray& arr,
                       const std::vector<std::string> &link_names,
                       bool include_attached = false) const;

  void printStateInfo(std::ostream &out) const;
  
  void printTransforms(std::ostream &out) const;
  
  std::string getStateTreeString(const std::string& prefix = "") const;
  
private:

  void copyFrom(const RobotState &other);
  
  void allocVelocity();
  void allocAcceleration();
  void allocTransforms();
  
  void dirtyFK(int index)
  {
    dirty_fk_ = dirty_fk_ == NULL ? robot_model_->getJointOfVariable(index) : robot_model_->getCommonRoot(dirty_fk_, robot_model_->getJointOfVariable(index));
  }
  
  void dirtyFK(const JointModel *joint)
  {
    dirty_fk_ = dirty_fk_ == NULL ? joint : robot_model_->getCommonRoot(dirty_fk_, joint);
  }
  
  void updateMimicPosition(int index)
  {
    const JointModel *jm = robot_model_->getJointOfVariable(index);
    if (jm)
      updateMimicJoint(jm);
  }
  
  void updateMimicJoint(const JointModel *joint)
  {
    const std::vector<const JointModel*> &mim = joint->getMimicRequests();
    double v = position_[joint->getFirstVariableIndex()];
    for (std::size_t i = 0 ; i < mim.size() ; ++i)
      position_[mim[i]->getFirstVariableIndex()] =  mim[i]->getMimicFactor() * v + mim[i]->getMimicOffset();
  }
  
  void updateLinkTransformsInternal(const JointModel *start);
  
  /** \brief Return the instance of a random number generator */
  random_numbers::RandomNumberGenerator& getRandomNumberGenerator()
  {
    if (!rng_)
      rng_ = new random_numbers::RandomNumberGenerator();
    return *rng_;
  }
  
  void getMissingKeys(const std::map<std::string, double> &variable_map, std::vector<std::string> &missing_variables) const;
  void getStateTreeJointString(std::ostream& ss, const JointModel* jm, const std::string& pfx0, bool last) const;
  void printTransform(const Eigen::Affine3d &transform, std::ostream &out) const;
  
  RobotModelConstPtr        robot_model_;
  int                       called_new_for_;
  
  double                   *position_;
  double                   *velocity_;
  double                   *acceleration_;
  
  const JointModel         *dirty_fk_;
  const JointModel         *dirty_link_transforms_;
  const JointModel         *dirty_collision_body_transforms_;
  
  EigenSTL::vector_Affine3d transforms_;
  Eigen::Affine3d          *variable_joint_transforms_; // this points to an element in transforms_, so it is aligned 
  Eigen::Affine3d          *global_link_transforms_;  // this points to an element in transforms_, so it is aligned 
  Eigen::Affine3d          *global_collision_body_transforms_;  // this points to an element in transforms_, so it is aligned 
  
  /** \brief The attached bodies that are part of this state (from all links) */
  std::map<std::string, AttachedBody*> attached_body_map_;

  /** \brief This event is called when there is a change in the attached bodies for this state;
      The event specifies the body that changed and whether it was just attached or about to be detached. */
  AttachedBodyCallback                 attached_body_update_callback_;

  
  /** \brief For certain operations a state needs a random number generator. However, it may be slightly expensive
      to allocate the random number generator if many state instances are generated. For this reason, the generator
      is allocated on a need basis, by the getRandomNumberGenerator() function. Never use the rng_ member directly, but call
      getRandomNumberGenerator() instead. */
  random_numbers::RandomNumberGenerator *rng_;
};


}
}

#endif
