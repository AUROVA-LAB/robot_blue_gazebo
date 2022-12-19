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

#include <functional>

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Rand.hh>
#include <ignition/math/Vector3.hh>
#include "gazebo/physics/physics.hh"
#include "ActorStopObstaclePlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ActorStopObstaclePlugin)


/////////////////////////////////////////////////
ActorStopObstaclePlugin::ActorStopObstaclePlugin()
{
}

/////////////////////////////////////////////////
void ActorStopObstaclePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&ActorStopObstaclePlugin::OnUpdate, this, std::placeholders::_1)));


  // Read in the stop distance
  if (_sdf->HasElement("stop_distance"))
    this->stop_distance = _sdf->Get<double>("stop_distance");
  else
    this->stop_distance = 1.5;


  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;


  // Read in the other obstacles
  if (_sdf->HasElement("obstacles"))
  {
    sdf::ElementPtr modelElem =
      _sdf->GetElement("obstacles")->GetElement("model");
    while (modelElem)
    {
      // Read the geometry of the obstacle (rectangles in x-y)
      double max_x, min_x, max_y, min_y;
      if (modelElem->HasElement("max_x"))
        max_x = modelElem->Get<double>("max_x");
      else
        max_x = 1;
      if (modelElem->HasElement("min_x"))
        min_x = modelElem->Get<double>("min_x");
      else
        min_x = -1;
      if (modelElem->HasElement("max_y"))
        max_y = modelElem->Get<double>("max_y");
      else
        max_y = 1;
      if (modelElem->HasElement("min_y"))
        min_y = modelElem->Get<double>("min_y");
      else
        min_y = -1;
      this->obstacleModels.push_back({modelElem->Get<std::string>(), max_x,min_x,max_y,min_y});  
      modelElem = modelElem->GetNextElement("model");
    }
  }
  LoadTrajectory();
  this->Reset();
}

void ActorStopObstaclePlugin::LoadTrajectory(){

  // Load all trajectories
  if (sdf->HasElement("trajectory"))
  {
    sdf::ElementPtr trajSdf = sdf->GetElement("trajectory");
    trajInfo = std::vector<physics::TrajectoryInfoPtr>();
    trajectories = std::map<unsigned int, common::PoseAnimation *>();
    while (trajSdf)
    {
      auto trajType = trajSdf->Get<std::string>("type");

      physics::TrajectoryInfoPtr tinfo;
      tinfo.reset(new physics::TrajectoryInfo());
      tinfo->id = trajSdf->Get<int>("id");
      tinfo->type = trajType;

      // Place trajectory into vector according to id order
      int iter=0;
      for(int i=0;i<trajInfo.size();i++){
        if(trajInfo[i]->id > tinfo->id) break;
        iter++;
      }

      unsigned int idx = iter;
      this->trajInfo.insert(trajInfo.begin()+iter, tinfo);

      // Waypoints
      if (trajSdf->HasElement("waypoint"))
      {
        // Fill a map with waypoints time and pose
        std::map<double, ignition::math::Pose3d> points;
        sdf::ElementPtr wayptSdf = trajSdf->GetElement("waypoint");
        while (wayptSdf)
        {
          points[wayptSdf->Get<double>("time")] =
              wayptSdf->Get<ignition::math::Pose3d>("pose");
          wayptSdf = wayptSdf->GetNextElement("waypoint");
        }

        // Get total trajectory duration (last waypoint's time)
        auto last = points.rbegin();

        std::stringstream animName;
        animName << tinfo->type << "_" << tinfo->id;
        common::PoseAnimation *anim = new common::PoseAnimation(animName.str(),
            last->first, false);

        // Create a keyframe for each point
        for (auto pIter = points.begin(); pIter != points.end(); ++pIter)
        {
          common::PoseKeyFrame *key;
          // Force first point always to start at 0s
          if (pIter == points.begin() &&
              !ignition::math::equal(pIter->first, 0.0))
          {
            key = anim->CreateKeyFrame(0.0);
          }
          else
          {
            key = anim->CreateKeyFrame(pIter->first);
          }

          key->Translation(pIter->second.Pos());
          key->Rotation(pIter->second.Rot());
        }

        // `trajInfo` holds information like start, end and duration
        this->trajInfo[idx]->duration = last->first;
        this->trajInfo[idx]->translated = true;

        this->trajectories[this->trajInfo[idx]->id] = anim;

      trajSdf = trajSdf->GetNextElement("trajectory");
      }
    }

    // Finally, (re)fill the times for all trajectories so that they are in a
    // sequence
    double time = 0.0;
    for (unsigned int i = 0; i < this->trajInfo.size(); ++i)
    {
      this->trajInfo[i]->startTime = time;
      time += this->trajInfo[i]->duration;
      this->trajInfo[i]->endTime = time;
    }
    this->trajectoryLength=time;
  }
 
}

/////////////////////////////////////////////////
void ActorStopObstaclePlugin::Reset()
{
  if(trajInfo.size()>0){
    actor->Play();
    this->startTime = this->world->SimTime().Double();
  }
  
}


/////////////////////////////////////////////////
void ActorStopObstaclePlugin::HandleObstacles(double currentTime)
{
  bool obstacle=false;
  for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
  {
    //Search the obstacles models
    physics::ModelPtr model = this->world->ModelByIndex(i);
    for(auto Obstacle : obstacleModels){
      if (Obstacle.id==model->GetName())
      {
        ignition::math::Vector3d rpy = model->WorldPose().Rot().Euler();
        ignition::math::Vector3d center = model->WorldPose().Pos();
        ignition::math::Vector3d actorPose = this->actor->WorldPose().Pos();

        //Calculate the minimum distance to the obstacle (considering a rectangle ABCD)
        ignition::math::Vector2d A=Rotation(center,Obstacle.max_x,Obstacle.max_y,rpy.Z());
        ignition::math::Vector2d B=Rotation(center,Obstacle.max_x,Obstacle.min_y,rpy.Z());
        ignition::math::Vector2d C=Rotation(center,Obstacle.min_x,Obstacle.max_y,rpy.Z());
        ignition::math::Vector2d D=Rotation(center,Obstacle.min_x,Obstacle.min_y,rpy.Z());

        double distance = std::min(Distance2Line(actorPose, A,B,this->direction),Distance2Line(actorPose, C,A,this->direction));
        distance=std::min(distance, Distance2Line(actorPose, C,D,this->direction)); 
        distance=std::min(distance, Distance2Line(actorPose, D,B,this->direction));

        if (distance < stop_distance)
        // There is an obstacle in front.
        {
          obstacle=true;
          if(actor->IsActive()){
            //If the actor is moving, stop and save the animation time.
            scriptTime=currentTime - this->startTime;
            actor->Stop();
          }
          break; //It isn't needed to check the rest of obstacles.
        }
      }
    }
  }
  if(!obstacle && !actor->IsActive()){
    //Continue the animation
    actor->Play();
    actor->SetScriptTime(scriptTime);
    this->startTime = this->world->SimTime().Double()-scriptTime;
  }
}

/////////////////////////////////////////////////
double ActorStopObstaclePlugin::Distance2Line(ignition::math::Vector3d point, ignition::math::Vector2d a, ignition::math::Vector2d b, double theta){

  double vx = b.X()-a.X(); double vy = b.Y()-a.Y();
  double magnitude = std::sqrt(vx*vx+vy*vy);
  double nx = std::cos(theta); double ny = std::sin(theta);
  //The lines are (almost) pararell.
  if(std::abs(nx*vy/magnitude + ny*vx/magnitude)<0.01){
    return this->stop_distance*10;
  }
  double alpha = (a.Y() +vy*(point.X()-a.X())/vx -point.Y())/(ny-nx*vy/vx);
  double beta = (point.X()+alpha*nx-a.X())/vx;

  //Check if it cross the line, with some secure factor, as the actor isn't a point.
  if(beta>-0.15 && beta<1.15 && alpha>0)
    return alpha;

  return this->stop_distance*10;
}

/////////////////////////////////////////////////
ignition::math::Vector2d ActorStopObstaclePlugin::Rotation(ignition::math::Vector3d center,double x, double y, double theta){

  ignition::math::Vector2d result;
  result.X()=center.X()+x*cos(theta)+y*sin(theta);
  result.Y()=center.Y()+x*sin(theta)+y*cos(theta);
  return result;
}

/////////////////////////////////////////////////
void ActorStopObstaclePlugin::OnUpdate(const common::UpdateInfo &_info)
{
  if(trajInfo.size()>0){
    double currentTime = this->world->SimTime().Double();
  
    // Check if the actor have a obstacle in front.
    this->HandleObstacles(currentTime);

    if(this->actor->IsActive()){
      //Play the trajectory and animation
      scriptTime = currentTime - startTime;

      ///DEBUG///
      //if(std::fmod(scriptTime,0.5)==0) gzdbg<<scriptTime<<" "<<currentTime<<" "<<startTime<<std::endl;
      //////////

      //Restart the trajectory when it ends.
      if(scriptTime>=trajectoryLength){
        scriptTime-=trajectoryLength;
        startTime=currentTime;
      }
      
      for(auto tinfo : trajInfo){
        //Search the actual trajectory.
        if(tinfo->startTime<=scriptTime && tinfo->endTime>scriptTime){
          this->actor->SetCustomTrajectory(tinfo);

          if (this->trajectories.find(tinfo->id) != this->trajectories.end()){
            // Get the pose keyframe calculated for this script time
            common::PoseKeyFrame posFrame(0.0);
            this->trajectories[tinfo->id]->SetTime(this->scriptTime-tinfo->startTime);
            this->trajectories[tinfo->id]->GetInterpolatedKeyFrame(posFrame);

            ignition::math::Pose3d pose;
            //Get the next actor pose.
            pose.Pos()=posFrame.Translation();
            pose.Rot()=posFrame.Rotation()*ignition::math::Quaterniond(1.5707, 0, 0);
            pose.Pos().Z(1.0238);

            //Calculate the direction. 
            auto Traveled = (pose.Pos() - this->actor->WorldPose().Pos());
            double distanceTraveled = Traveled.Length();
            this->direction = atan2(Traveled.Y(),Traveled.X());
            //Set the script time depending to the distance, to synchronice the animation. 
            this->actor->SetScriptTime(this->actor->ScriptTime() + (distanceTraveled * this->animationFactor));
        
            this->actor->SetWorldPose(pose,false,false);
          }
          break;
        }
      }
    }
  }
}