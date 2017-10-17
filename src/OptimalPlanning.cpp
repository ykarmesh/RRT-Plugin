/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Luis G. Torres, Jonathan Gammell */
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/MapMetaData.h>
#include <visualization_msgs/Marker.h>
#include "rrt_star/costmap_model.h"
#include "costmap_model.cpp"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>

// For ompl::msg::setLogLevel
#include "ompl/util/Console.h"

// The supported optimal planners, in alphabetical order
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <iostream>
// For boost program options
#include <boost/program_options.hpp>
// For string comparison (boost::iequals)
#include <boost/algorithm/string.hpp>
// For std::make_shared
#include <memory>

#include <fstream>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include "rrt_star/rrt.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

ros::Publisher vis_pub;
ros::Publisher vis_pub1;

/// @cond IGNORE


class Plan{
public:
    Plan(costmap_2d::Costmap2DROS* cmap_ros): initialized_(false)//, private_nh_(nh) ros::NodeHandle nh
    {
      cmap_ros_ = cmap_ros;
      cmap_ = cmap_ros->getCostmap();
      world_model_ = new rrt_star::CostmapModel(*cmap_);
    }

    ~Plan()
    {
      delete world_model_;
    }

    void initrrt()
    {
      if(!initialized_)
      {
        ROS_INFO("Initializing planner with costmap");

        ros::NodeHandle private_nh("~");
        //ros::NodeHandle nh;
        private_nh_ = private_nh;
        private_nh_.param("max_footprint_cost", max_footprint_cost_, 256);
        private_nh_.param("Inscribed_radius", inscribed_radius_, 0.25);
        private_nh_.param("Circumscribed_radius", circumscribed_radius_, 0.25);

        goal_sub = private_nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &Plan::goalCb, this);

        space = ob::StateSpacePtr(new ob::SE2StateSpace());

        // set the bounds
        ob::RealVectorBounds bounds(2);
        bounds.setLow(0, cmap_->getOriginX());
        bounds.setHigh(0, cmap_->getOriginX() + cmap_->getSizeInCellsX() * cmap_->getResolution());
        bounds.setLow(1, cmap_->getOriginY());
        bounds.setHigh(1, cmap_->getOriginY() + cmap_->getSizeInCellsY() * cmap_->getResolution());
        space->as<ob::SE2StateSpace>()->setBounds(bounds);

        // Construct a space information instance for this state space

        si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

        // Set the object used to check which states in the space are valid
        si->setStateValidityChecker(boost::bind(&Plan::isStateValid, this, _1));
        si->setStateValidityCheckingResolution(0.01);

        si->setup();

        // Create a problem instance
        pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

        // Create the optimization objective specified by our command-line argument.
        // This helper function is simply a switch statement.
        pdef->setOptimizationObjective(getPathLengthObjective(si));

        initialized_ = true;

      }
      else
        ROS_WARN("This planner has already been initialized... doing nothing");
    }

    void setGoal(double x, double y, double theta)
    {
        if(prev_goal[0] != x || prev_goal[1] != y || prev_goal[2] != theta)
        {
            ob::ScopedState<> goal(space);
            goal->as<ob::SE2StateSpace::StateType>()->setX(x);
            goal->as<ob::SE2StateSpace::StateType>()->setY(y);
            goal->as<ob::SE2StateSpace::StateType>()->setYaw(theta);
            prev_goal[0] = x;
            prev_goal[1] = y;
            prev_goal[2] = theta;
			      pdef->setGoalState(goal);
        }
    }

    void setStart(double x, double y, double theta)
    {
        ob::ScopedState<> start(space);
        start->as<ob::SE2StateSpace::StateType>()->setX(x);
        start->as<ob::SE2StateSpace::StateType>()->setY(y);
        start->as<ob::SE2StateSpace::StateType>()->setYaw(theta);
        pdef->clearStartStates();
        pdef->addStartState(start);
    }

    bool isStateValid(const ompl::base::State *state)
	  {
      if(!initialized_)
      {
        ROS_ERROR("The planner has not been initialized, please call initrrt() to use the planner");
        return -1.0;
      }

      double costs = 0.0;

      std::vector<geometry_msgs::Point> footprint_spec_ = cmap_ros_->getRobotFootprint();
  		if(footprint_spec_.size() < 3)
  		{	return -1.0;
        ROS_ERROR("Footprint not specified, assuming a point robot model");
      }

      geometry_msgs::Point robot_position;
      double theta;
      robot_position.x = state->as<ob::SE2StateSpace::StateType>()->getX();
      robot_position.y = state->as<ob::SE2StateSpace::StateType>()->getY();
      theta = state->as<ob::SE2StateSpace::StateType>()->getYaw();

  		// build the oriented footprint
  		double cos_th = cos(theta);
  		double sin_th = sin(theta);
  		std::vector<geometry_msgs::Point> oriented_footprint;
  		for(unsigned int i = 0; i < footprint_spec_.size(); i++){
  			geometry_msgs::Point new_pt;
  			new_pt.x = robot_position.x + (footprint_spec_[i].x * cos_th - footprint_spec_[i].y * sin_th);
  			new_pt.y = robot_position.y + (footprint_spec_[i].x * sin_th - footprint_spec_[i].y * cos_th);
  			oriented_footprint.push_back(new_pt);
      }

      // check if the footprint is legal
      costs = world_model_->footprintCost(robot_position, oriented_footprint, inscribed_radius_, circumscribed_radius_);

      if( (costs >= 0) && (costs < max_footprint_cost_) )
      {  return true;
      }
      return false;
    }

    void rrtplan(void)
    {
        ROS_INFO("Planning has started");
        if(!initialized_){
          ROS_ERROR("The planner has not been initialized, please call initrrt() to use the planner");
          return;
        }

        // Construct the optimal planner specified by our command line argument.
        // This helper function is simply a switch statement.
        ob::PlannerPtr optimizingPlanner = std::make_shared<og::RRTstar>(si);

        // Set the problem instance for our planner to solve
        optimizingPlanner->setProblemDefinition(pdef);
        optimizingPlanner->setup();

        // print the settings for this space
		    //si->printSettings(std::cout);
        ob::ScopedState<> goal(space);
        //ob::GoalPtr goal= pdef->getGoal();
	      // print the problem settings
		    pdef->print(std::cout);

	       // attempt to solve the problem within one second of planning time
		    ob::PlannerStatus solved = optimizingPlanner->solve(1.0);

        if (solved)
        {
            // Output the length of the path found
            ROS_INFO_STREAM(optimizingPlanner->getName()<< " found a solution of length "<< pdef->getSolutionPath()->length()
            << " with an optimization objective value of "<< pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()));

            path_ = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
            path_->print(std::cout);
            visualize(path_);
          //}
            pdef->clearSolutionPaths();
            delete path_;
        }
        else
          ROS_ERROR("No solution found");
    }

    void visualize(const og::PathGeometric* path)
    {
        ros::Duration(1).sleep();
        visualization_msgs::Marker marker, line;
        //marker.action = visualization_msgs::Marker::DELETEALL;
  			//vis_pub.publish(marker);
        //rviz_visual_tools::RvizVisualTools rviz_interface("map","/visualization_marker_array");
        //rviz_interface.deleteAllMarkers();

        for (std::size_t idx = 0; idx < path->getStateCount (); idx++)
        {

          // cast the abstract state type to the type we expect
          const auto *rstate = static_cast<const ob::SE2StateSpace::StateType *>(path->getState(idx));
          const ob::SE3StateSpace::StateType *se3state = path->getState(idx)->as<ob::SE3StateSpace::StateType>();

          // extract the second component of the state and cast it to what we expect
          const auto *qstate = static_cast<const ob::SO3StateSpace::StateType *>(path->getState(idx));

          marker.header.frame_id = line.header.frame_id = "map";
          marker.header.stamp = line.header.stamp = ros::Time::now();
          marker.ns = "path";
          line.ns = "pathline";
          marker.id = idx;
          line.id = idx;
          marker.type  = visualization_msgs::Marker::CUBE;
          line.type = visualization_msgs::Marker::LINE_STRIP;
          marker.action = line.action = visualization_msgs::Marker::ADD;
          geometry_msgs::Point p;
          marker.pose.position.x = rstate->getX();
          marker.pose.position.y = rstate->getY();
          marker.pose.position.z = 0;
          p.x = rstate->getX();
          p.y = rstate->getY();
          p.z = 0;
          marker.points.push_back(p);
          line.points.push_back(p);
          marker.pose.orientation.x = qstate->x;
          marker.pose.orientation.y = qstate->y;
          marker.pose.orientation.z = qstate->z;
          marker.pose.orientation.w = qstate->w;
          line.pose.orientation.w = 1;
          marker.scale.x = 0.2;
          marker.scale.y = 0.2;
          marker.scale.z = 0.2;
          line.scale.x = 0.1;
          line.scale.y = 0.1;
          line.scale.z = 0.4;
          marker.color.a = 1.0;
          marker.color.r = 0.0;
          marker.color.g = 1.0;
          marker.color.b = 0.0;
          line.color.a = 0.5;
          line.color.r = 0.0;
          line.color.g = 0.0;
          line.color.b = 1.0;
          vis_pub.publish(marker);
          vis_pub1.publish(line);
          std::cout << "Published marker: " << idx << std::endl;
      }
    }

    ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
    {
        return std::make_shared<ob::PathLengthOptimizationObjective>(si);
    }

    void goalCb(const geometry_msgs::PoseStamped::ConstPtr& goal)
    {
    	  setGoal(goal->pose.position.x, goal->pose.position.y, getTheta(goal->pose.orientation));
        ROS_INFO_STREAM("New goal recieved at ["<< goal->pose.position.x <<"]["<<goal->pose.position.y<<"]");
          //set start state
        tf::Stamped<tf::Pose> global_pose;
        cmap_ros_->getRobotPose(global_pose);

        setStart(global_pose.getOrigin().x(),global_pose.getOrigin().y(), getTheta(global_pose.getRotation()));
        ROS_INFO_STREAM("New start initialized at ["<< global_pose.getOrigin().x() <<"]["<<global_pose.getOrigin().y()<<"]");
        rrtplan();
    }
    double getTheta(geometry_msgs::Quaternion msg)
    {
      tf::Quaternion q(msg.x, msg.y, msg.z, msg.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      return yaw;
    }
    double getTheta(tf::Quaternion q)
    {
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      return yaw;
    }

private:

    ros::NodeHandle private_nh_;

    ob::StateSpacePtr space;

    // construct an instance of  space information from this state space
    ob::SpaceInformationPtr si;

    // create a problem instance
    ob::ProblemDefinitionPtr pdef;

    double prev_goal[3];

    const og::PathGeometric* path_;

    bool initialized_ = false;

    //const std::string outputFile("outputfile");

    costmap_2d::Costmap2DROS* cmap_ros_;

    costmap_2d::Costmap2D* cmap_;

    int max_footprint_cost_;

    double inscribed_radius_, circumscribed_radius_;

    rrt_star::CostmapModel *world_model_;

    ros::Subscriber goal_sub;
};

/*
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getBalancedObjective1(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getBalancedObjective2(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si);
*/

void mapserverCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg, Plan* planptr)
{
    ROS_INFO("New map recieved from Map_Server");
    ros::Duration(1).sleep();
	  planptr->initrrt();
}

int main(int argc, char** argv)
{   ros::init(argc, argv,"optimal_planning_node");

    ompl::msg::setLogLevel(ompl::msg::LOG_DEBUG);

    ros::NodeHandle nh;

    tf::TransformListener tf(ros::Duration(10));

    costmap_2d::Costmap2DROS costmap("global_costmap",tf);

    Plan planner_object(&costmap);

    ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, boost::bind(&mapserverCallback, _1, &planner_object));

    vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    vis_pub1 = nh.advertise<visualization_msgs::Marker>( "visualization_marker_line", 0 );

    ros::spin();
    return 0;
}

/** Returns a structure representing the optimization objective to use
    for optimal motion planning. This method returns an objective
    which attempts to minimize the length in configuration space of
    computed paths. */
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return std::make_shared<ob::PathLengthOptimizationObjective>(si);
}

/** Returns an optimization objective which attempts to minimize path
    length that is satisfied when a path of length shorter than 1.51
    is found. *
ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
    auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    obj->setCostThreshold(ob::Cost(1.51));
    return obj;
}

/** Defines an optimization objective which attempts to steer the
    robot away from obstacles. To formulate this objective as a
    minimization of path cost, we can define the cost of a path as a
    summation of the costs of each of the states along the path, where
    each state cost is a function of that state's clearance from
    obstacles.

    The class StateCostIntegralObjective represents objectives as
    summations of state costs, just like we require. All we need to do
    then is inherit from that base class and define our specific state
    cost function by overriding the stateCost() method.
 *
class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:
    ClearanceObjective(const ob::SpaceInformationPtr& si) :
        ob::StateCostIntegralObjective(si, true)
    {
    }

    // Our requirement is to maximize path clearance from obstacles,
    // but we want to represent the objective as a path cost
    // minimization. Therefore, we set each state's cost to be the
    // reciprocal of its clearance, so that as state clearance
    // increases, the state cost decreases.
    ob::Cost stateCost(const ob::State* s) const override
    {
        return ob::Cost(1 / si_->getStateValidityChecker()->clearance(s));
    }
};

/** Return an optimization objective which attempts to steer the robot
    away from obstacles. *
ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si)
{
    return std::make_shared<ClearanceObjective>(si);
}

/** Create an optimization objective which attempts to optimize both
    path length and clearance. We do this by defining our individual
    objectives, then adding them to a MultiOptimizationObjective
    object. This results in an optimization objective where path cost
    is equivalent to adding up each of the individual objectives' path
    costs.

    When adding objectives, we can also optionally specify each
    objective's weighting factor to signify how important it is in
    optimal planning. If no weight is specified, the weight defaults to
    1.0.
*
ob::OptimizationObjectivePtr getBalancedObjective1(const ob::SpaceInformationPtr& si)
{
    auto lengthObj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    auto clearObj(std::make_shared<ClearanceObjective>(si));
    auto opt(std::make_shared<ob::MultiOptimizationObjective>(si));
    opt->addObjective(lengthObj, 10.0);
    opt->addObjective(clearObj, 1.0);

    return ob::OptimizationObjectivePtr(opt);
}

/** Create an optimization objective equivalent to the one returned by
    getBalancedObjective1(), but use an alternate syntax.
 *
ob::OptimizationObjectivePtr getBalancedObjective2(const ob::SpaceInformationPtr& si)
{
    auto lengthObj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    auto clearObj(std::make_shared<ClearanceObjective>(si));

    return 10.0*lengthObj + clearObj;
}

/** Create an optimization objective for minimizing path length, and
    specify a cost-to-go heuristic suitable for this optimal planning
    problem. *
ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
{
    auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
    return obj;
}

/// @endcond
*/
