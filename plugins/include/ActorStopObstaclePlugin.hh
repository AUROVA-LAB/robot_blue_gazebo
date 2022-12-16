/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#ifndef GAZEBO_PLUGINS_ACTORPLUGIN_HH_
#define GAZEBO_PLUGINS_ACTORPLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"
#include "gazebo/common/KeyFrame.hh"
#include "gazebo/common/Skeleton.hh"
#include "gazebo/common/SkeletonAnimation.hh"

namespace gazebo
{
  struct StopObstacle{
    std::string id;
    double max_x;
    double min_x;
    double max_y;
    double min_y;
  };

  class GZ_PLUGIN_VISIBLE ActorStopObstaclePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: ActorStopObstaclePlugin();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Reset();

    /// \brief Function that is called every update cycle.
    /// \param[in] _info Timing information
    private: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Helper function to avoid obstacles. This implements a very
    /// simple vector-field algorithm.
    /// \param[in] _pos Direction vector that should be adjusted according
    /// to nearby obstacles.
    private: void HandleObstacles(double currentTime);

    private: virtual void LoadTrajectory();

    private: double Distance2Line(ignition::math::Vector3d point, ignition::math::Vector2d a, ignition::math::Vector2d b, double theta);

    private: ignition::math::Vector2d Rotation(ignition::math::Vector3d center,double x, double y, double theta);

    /// \brief Pointer to the parent actor.
    private: physics::ActorPtr actor;

    private: std::vector<physics::TrajectoryInfoPtr> trajInfo;
    private: std::map<unsigned int, common::PoseAnimation *> trajectories;

    /// \brief Pointer to the world, for convenience.
    private: physics::WorldPtr world;

    /// \brief Pointer to the sdf element.
    private: sdf::ElementPtr sdf;

    /// \brief List of connections
    private: std::vector<event::ConnectionPtr> connections;

    /// \brief Obstacle weight (used for vector field)
    private: double stop_distance = 1.0;

    /// \brief Time scaling factor. Used to coordinate translational motion
    /// with the actor's walking animation.
    private: double animationFactor = 1.0;

    /// \brief Time of the last update.
    private: common::Time lastUpdate;

    private: double scriptTime;
    private: double startTime;
    private: double trajectoryLength;
    private: double direction;

    /// \brief List of models to ignore. Used for vector field
    private: std::vector<StopObstacle> obstacleModels;

  };
}
#endif