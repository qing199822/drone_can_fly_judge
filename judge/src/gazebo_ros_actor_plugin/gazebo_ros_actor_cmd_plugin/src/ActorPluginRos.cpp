/*
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
/**
 *  \author David Leins
 *  \date 28th of November 2018
 *  \desc Gazebo ros plugin to steer an actor with twist messages over ros. Based on actor_plugin. 
 */

#include "actor_plugin_ros/ActorPluginRos.hpp"

#include <algorithm>
#include <cmath>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ActorPluginRos)

#define WALKING_ANIMATION "walking"
#define WAVING_ANIMATION "waving"

/////////////////////////////////////////////////
ActorPluginRos::ActorPluginRos()
{
}

/////////////////////////////////////////////////
void ActorPluginRos::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  gazebo_ros_ = GazeboRosPtr(new GazeboRos(_model, _sdf, "ActorRos"));
  gazebo_ros_->isInitialized();

  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  if (_sdf->HasElement("velocity_command_timeout"))
    this->velocity_command_timeout =
        std::max(0.1, _sdf->Get<double>("velocity_command_timeout"));

  if (_sdf->HasElement("init_pose"))
    this->init_pose = _sdf->Get<ignition::math::Pose3d>("init_pose");
  else
    this->init_pose = ignition::math::Pose3d(0, 0, 1.0191, 1.57, 0, 0);

  // this->velocity =20;
  // Make sure the actor stays within bounds
  this->init_pose.Pos().X(std::max(-50.0, std::min(150.0, this->init_pose.Pos().X())));
  this->init_pose.Pos().Y(std::max(-50.0, std::min(50.0, this->init_pose.Pos().Y())));
  this->init_pose.Pos().Z(1.0191);

  // Reset reads init_pose and configures the custom walking trajectory, so it
  // must run after the SDF configuration has been loaded.
  this->Reset();

  // Place the actor immediately. This matches the original plugin behaviour
  // and avoids the case where some actors never receive an early world-update
  // callback and remain invisible. OnUpdate will re-apply the pose if Gazebo
  // overwrites it during insertion.
  this->actor->SetWorldPose(this->init_pose, false, false);

  // Register ROS callbacks and the world update only after every member used
  // by them is initialized. Headless simulation can update the world while
  // Load() is still running.
  cmd_pose_subscriber_ = gazebo_ros_->node()->subscribe("cmd_motion", 1, &ActorPluginRos::CmdPoseCallback, this);
  ROS_INFO_NAMED("ActorRos", "%s: Subscribe to %s", gazebo_ros_->info(), cmd_pose_subscriber_.getTopic().c_str());

  cmd_velocity_subscriber_ = gazebo_ros_->node()->subscribe(
      "cmd_vel", 1, &ActorPluginRos::CmdVelocityCallback, this);
  ROS_INFO_NAMED("ActorRos", "%s: Subscribe to %s", gazebo_ros_->info(),
                 cmd_velocity_subscriber_.getTopic().c_str());

  wave_toggle_service_ = gazebo_ros_->node()->advertiseService("actor_toggle_wave", &ActorPluginRos::ToggleWaveAnimation, this);
  ROS_INFO_NAMED("ActorRos", "%s: Advertise service %s", gazebo_ros_->info(), wave_toggle_service_.getService().c_str());

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&ActorPluginRos::OnUpdate, this, std::placeholders::_1)));
}

/////////////////////////////////////////////////
void ActorPluginRos::Reset()
{
  {
    std::lock_guard<std::mutex> lock(this->command_mutex);
    this->velocity = 0.8;
    this->target = this->init_pose.Pos();
    this->GET_CMD_FLAG = false;
    this->commanded_linear_velocity.Set(0, 0, 0);
    this->velocity_command_active = false;
    this->last_velocity_command_time = 0.0;
  }
  this->current_pose = this->init_pose;
  this->current_yaw = this->init_pose.Rot().Euler().Z();
  this->lastUpdate = 0;
  this->initial_pose_applied = false;

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;
    this->last_linear = ignition::math::Vector3d(0, 0, 0);
    this->last_angle = 0;
    this->actor->SetCustomTrajectory(this->trajectoryInfo);
    this->wave_toggled = false;
  }
}

/////////////////////////////////////////////////
void ActorPluginRos::ChooseNewTarget()
{
  ignition::math::Vector3d newTarget(this->target);
  while ((newTarget - this->target).Length() < 2.0)
  {
    newTarget.X(ignition::math::Rand::DblUniform(-50, 150));
    newTarget.Y(ignition::math::Rand::DblUniform(-50, 50));

    for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
    {
      double dist = (this->world->ModelByIndex(i)->WorldPose().Pos() - newTarget).Length();
      if (dist < 2.0)
      {
        newTarget = this->target;
        break;
      }
    }
  }
  this->target = newTarget;
}

/////////////////////////////////////////////////
void ActorPluginRos::CmdPoseCallback(const ros_actor_cmd_pose_plugin_msgs::ActorMotion::ConstPtr &cmd_msg)
{
  std::lock_guard<std::mutex> lock(this->command_mutex);
  this->target.X(std::max(-50.0, std::min(150.0, static_cast<double>(cmd_msg->x))));
  this->target.Y(std::max(-50.0, std::min(50.0, static_cast<double>(cmd_msg->y))));
  this->target.Z(this->init_pose.Pos().Z());
  this->velocity = std::max(0.0, static_cast<double>(cmd_msg->v));
  this->GET_CMD_FLAG = true;
  this->commanded_linear_velocity.Set(0, 0, 0);
  this->velocity_command_active = false;
  ROS_INFO_THROTTLE(5.0, "%s target set to (%.2f, %.2f) at %.2f m/s",
                 this->actor->GetName().c_str(), this->target.X(),
                 this->target.Y(), this->velocity);
}

/////////////////////////////////////////////////
void ActorPluginRos::CmdVelocityCallback(
    const geometry_msgs::Twist::ConstPtr &cmd_msg)
{
  ignition::math::Vector3d command(cmd_msg->linear.x, cmd_msg->linear.y, 0.0);
  const double requestedSpeed = command.Length();
  const double maxSpeed = 2.0;
  if (requestedSpeed > maxSpeed)
    command *= maxSpeed / requestedSpeed;

  std::lock_guard<std::mutex> lock(this->command_mutex);
  this->commanded_linear_velocity = command;
  this->velocity_command_active = command.Length() > 1e-6;
  this->last_velocity_command_time = ros::Time::now().toSec();
  this->GET_CMD_FLAG = false;

  ROS_INFO_THROTTLE(1.0, "%s velocity command: (%.2f, %.2f) m/s",
                    this->actor->GetName().c_str(), command.X(), command.Y());
}

/////////////////////////////////////////////////
bool ActorPluginRos::ToggleWaveAnimation(ros_actor_cmd_pose_plugin_msgs::ToggleActorWaving::Request &req, ros_actor_cmd_pose_plugin_msgs::ToggleActorWaving::Response &res)
{
  ROS_INFO("request: toggle waving animation");

  this->lastUpdate = 0;

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WAVING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WAVING_ANIMATION << " not found.\n";
    res.exit = false;
    this->wave_toggled = false;
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WAVING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;
    this->last_linear = ignition::math::Vector3d(0, 0, 0);
    this->last_angle = 0;
    this->actor->SetCustomTrajectory(this->trajectoryInfo);
    this->wave_toggled = true;
    res.exit = true;
  }

  ROS_INFO("sending back response: %s", res.exit ? "true" : "false");

  return true;
}

/////////////////////////////////////////////////
void ActorPluginRos::OnUpdate(const common::UpdateInfo &_info)
{
  // Gazebo may overwrite SetWorldPose calls made while the actor is still
  // being inserted. Apply the configured pose once from the update thread,
  // after insertion is complete.
  if (!this->initial_pose_applied)
  {
    this->actor->SetWorldPose(this->current_pose, true, false);
    this->initial_pose_applied = true;
    this->lastUpdate = _info.simTime;
    ROS_INFO_NAMED("ActorRos", "%s placed at (%.2f, %.2f, %.2f)",
                   this->actor->GetName().c_str(), this->init_pose.Pos().X(),
                   this->init_pose.Pos().Y(), this->init_pose.Pos().Z());
    return;
  }

  ignition::math::Vector3d motionTarget;
  double commandedVelocity;
  bool hasCommand;
  ignition::math::Vector3d linearVelocityCommand;
  bool hasVelocityCommand;
  double velocityCommandTime;
  {
    std::lock_guard<std::mutex> lock(this->command_mutex);
    motionTarget = this->target;
    commandedVelocity = this->velocity;
    hasCommand = this->GET_CMD_FLAG;
    linearVelocityCommand = this->commanded_linear_velocity;
    hasVelocityCommand = this->velocity_command_active;
    velocityCommandTime = this->last_velocity_command_time;
  }

  // Ignore negative time jumps after a world reset and cap unusually large
  // steps so a stalled simulation cannot teleport an actor.
  const double dt = std::min(0.1, std::max(
      0.0, (_info.simTime - this->lastUpdate).Double()));

  if (this->wave_toggled)
  {
    this->actor->SetWorldPose(this->current_pose, true, false);
    this->actor->SetScriptTime(this->actor->ScriptTime() +
                               dt);
    this->lastUpdate = _info.simTime;
  }
  else
  {
    if (hasVelocityCommand &&
        _info.simTime.Double() - velocityCommandTime >
            this->velocity_command_timeout)
    {
      std::lock_guard<std::mutex> lock(this->command_mutex);
      if (this->last_velocity_command_time == velocityCommandTime)
      {
        this->commanded_linear_velocity.Set(0, 0, 0);
        this->velocity_command_active = false;
      }
      hasVelocityCommand = false;
      ROS_WARN_NAMED("ActorRos", "%s velocity command timed out; holding position",
                     this->actor->GetName().c_str());
    }

    if (hasVelocityCommand)
    {
      const ignition::math::Vector3d previousPosition =
          this->current_pose.Pos();
      const double desiredYaw = std::atan2(linearVelocityCommand.Y(),
                                           linearVelocityCommand.X()) +
                                IGN_PI_2;
      ignition::math::Angle yawError(desiredYaw - this->current_yaw);
      yawError.Normalize();
      const double maxYawStep = 2.5 * dt;
      this->current_yaw += std::max(-maxYawStep, std::min(
          maxYawStep, yawError.Radian()));
      ignition::math::Angle normalizedYaw(this->current_yaw);
      normalizedYaw.Normalize();
      this->current_yaw = normalizedYaw.Radian();

      this->current_pose.Pos() += linearVelocityCommand * dt;
      this->current_pose.Pos().X(std::max(
          -50.0, std::min(150.0, this->current_pose.Pos().X())));
      this->current_pose.Pos().Y(std::max(
          -50.0, std::min(50.0, this->current_pose.Pos().Y())));
      this->current_pose.Pos().Z(this->init_pose.Pos().Z());
      this->current_pose.Rot() = ignition::math::Quaterniond(
          1.5707, 0, this->current_yaw);

      const double distanceTraveled =
          (this->current_pose.Pos() - previousPosition).Length();
      this->actor->SetWorldPose(this->current_pose, true, false);
      this->actor->SetScriptTime(this->actor->ScriptTime() +
                                 distanceTraveled * this->animationFactor);
      this->lastUpdate = _info.simTime;
      return;
    }

    if (!hasCommand)
    {
      // Actor animation can overwrite its transform even while idle. Keep the
      // last commanded pose authoritative so it never returns to the origin.
      // Throttle to ~1/5 of updates to reduce gzserver load/crash risk.
      if (++this->pose_apply_counter_ % 5 == 0)
      {
        this->actor->SetWorldPose(this->current_pose, true, false);
      }
      this->lastUpdate = _info.simTime;
      return;
    }

    ignition::math::Vector3d displacement =
        motionTarget - this->current_pose.Pos();
    displacement.Z(0.0);
    const double distance = displacement.Length();

    if (commandedVelocity <= 0.0)
    {
      {
        std::lock_guard<std::mutex> lock(this->command_mutex);
        if (this->target == motionTarget)
          this->GET_CMD_FLAG = false;
      }
      this->actor->SetWorldPose(this->current_pose, true, false);
      this->lastUpdate = _info.simTime;
      ROS_INFO_NAMED("ActorRos", "%s holding at (%.2f, %.2f, %.2f)",
                     this->actor->GetName().c_str(), this->current_pose.Pos().X(),
                     this->current_pose.Pos().Y(), this->current_pose.Pos().Z());
      return;
    }

    bool reached = distance <= 0.01;
    double distanceTraveled = 0.0;
    if (reached)
    {
      this->current_pose.Pos() = motionTarget;
    }
    else
    {
      const double desiredYaw =
          std::atan2(displacement.Y(), displacement.X()) + IGN_PI_2;
      ignition::math::Angle yawError(desiredYaw - this->current_yaw);
      yawError.Normalize();
      const double maxYawStep = 2.5 * dt;
      this->current_yaw += std::max(-maxYawStep, std::min(
          maxYawStep, yawError.Radian()));
      ignition::math::Angle normalizedYaw(this->current_yaw);
      normalizedYaw.Normalize();
      this->current_yaw = normalizedYaw.Radian();

      // Turning and translation happen together. Requiring the animated
      // actor's reported yaw to converge first can permanently block motion.
      const double step = std::min(distance, commandedVelocity * dt);
      this->current_pose.Pos() += displacement.Normalized() * step;
      distanceTraveled = step;
      reached = step >= distance - 1e-9;
      if (reached)
        this->current_pose.Pos() = motionTarget;
    }

    this->current_pose.Rot() = ignition::math::Quaterniond(
        1.5707, 0, this->current_yaw);

    // Make sure the actor stays within bounds
    this->current_pose.Pos().X(std::max(
        -50.0, std::min(150.0, this->current_pose.Pos().X())));
    this->current_pose.Pos().Y(std::max(
        -50.0, std::min(50.0, this->current_pose.Pos().Y())));
    this->current_pose.Pos().Z(this->init_pose.Pos().Z());

    // Distance traveled is used to coordinate motion with the walking
    // animation
    this->actor->SetWorldPose(this->current_pose, true, false);

    this->actor->SetScriptTime(this->actor->ScriptTime() +
                               (distanceTraveled * this->animationFactor));

    if (reached)
    {
      {
        std::lock_guard<std::mutex> lock(this->command_mutex);
        if (this->target == motionTarget)
          this->GET_CMD_FLAG = false;
      }
      ROS_INFO_NAMED("ActorRos", "%s holding at (%.2f, %.2f, %.2f)",
                     this->actor->GetName().c_str(), this->current_pose.Pos().X(),
                     this->current_pose.Pos().Y(), this->current_pose.Pos().Z());
    }
    this->lastUpdate = _info.simTime;
  }
}
