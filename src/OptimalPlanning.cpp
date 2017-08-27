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


#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
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


namespace ob = ompl::base;
namespace og = ompl::geometric;

ros::Publisher vis_pub;
/// @cond IGNORE
boost::shared_ptr<const nav_msgs::OccupancyGrid> costmap;
// Our "collision checker". For this demo, our robot's state space
// lies in [0,1]x[0,1], with a circular obstacle of radius 0.25
// centered at (0.5,0.5). Any states lying in this circular region are
// considered "in collision".
class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si) :
        ob::StateValidityChecker(si) {}

    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ob::State* state) const override
    {
        return this->clearance(state) > 0.0;
    }

    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ob::State* state) const override
    {
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const auto* state2D =
            state->as<ob::RealVectorStateSpace::StateType>();

        // Extract the robot's (x,y) position from its state
        double x = state2D->values[0];
        double y = state2D->values[1];

        unsigned int grid_x, grid_y;

        grid_x = (unsigned int)((x - costmap->info.origin.position.x) / costmap->info.resolution);
        grid_y = (unsigned int)((y - costmap->info.origin.position.y) / costmap->info.resolution);
        // Distance formula between two points, offset by the circle's
        // radius
        if(costmap->data.at((int)(grid_x * costmap->info.width + grid_y)) <= 10)
        {
          return 1;
        }
        else
        {
          return 0;
        }
    }
};
void visualize(og::PathGeometric* path);

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);
/*
ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getBalancedObjective1(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getBalancedObjective2(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si);
*/
void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg, const std::string& outputFile)
{
     // construct the state space we are planning in
     auto space(std::make_shared<ob::RealVectorStateSpace>(2));

     //costmap = dynamic_cast<nav_msgs::OccupancyGrid*>(msg);
     costmap = msg;
     // set the bounds
     //std::cout<<"first"<<std::endl;
     ob::RealVectorBounds bounds(2);
     bounds.setLow(0, msg->info.origin.position.x);
     bounds.setHigh(0, msg->info.origin.position.x + msg->info.width * msg->info.resolution);
     bounds.setLow(1, msg->info.origin.position.y);
     bounds.setHigh(1, msg->info.origin.position.y + msg->info.height * msg->info.resolution);
     space->setBounds(bounds);
     std::cout<< msg->info.origin.position.x + msg->info.width * msg->info.resolution<<" "<<msg->info.origin.position.y + msg->info.height * msg->info.resolution<<std::endl;
     // Construct a space information instance for this state space
     auto si(std::make_shared<ob::SpaceInformation>(space));

     // Set the object used to check which states in the space are valid
     si->setStateValidityChecker(std::make_shared<ValidityChecker>(si));

     si->setup();

     // Set our robot's starting state to be the bottom-left corner of
     // the environment, or (0,0).
     ob::ScopedState<> start(space);
     start->as<ob::RealVectorStateSpace::StateType>()->values[0] = 1.0;
     start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 1.0;

     // Set our robot's goal state to be the top-right corner of the
     // environment, or (1,1).
     ob::ScopedState<> goal(space);
     goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 8.0;
     goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 8.0;

     // Create a problem instance
     auto pdef(std::make_shared<ob::ProblemDefinition>(si));

     // Set the start and goal states
     pdef->setStartAndGoalStates(start, goal);
     //std::cout<<"second"<<std::endl;
     // Create the optimization objective specified by our command-line argument.
     // This helper function is simply a switch statement.
     pdef->setOptimizationObjective(getPathLengthObjective(si));

     // Construct the optimal planner specified by our command line argument.
     // This helper function is simply a switch statement.
     ob::PlannerPtr optimizingPlanner = std::make_shared<og::RRTstar>(si);

     // Set the problem instance for our planner to solve
     optimizingPlanner->setProblemDefinition(pdef);
     optimizingPlanner->setup();

     //std::cout<<"third"<<std::endl;

     // attempt to solve the planning problem in the given runtime
     ob::PlannerStatus solved = optimizingPlanner->solve(0.05);

     //std::cout<<"fourth"<<std::endl;

     if (solved)
     {
         // Output the length of the path found
         std::cout
             << optimizingPlanner->getName()
             << " found a solution of length "
             << pdef->getSolutionPath()->length()
             << " with an optimization objective value of "
             << pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()) << std::endl;

         // If a filename was specified, output the path as a matrix to
         // that file for visualization
         if (!outputFile.empty())
         {
             std::ofstream outFile(outputFile.c_str());
             std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->
                 printAsMatrix(outFile);
             outFile.close();

             og::PathGeometric* geometric_path = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));

             visualize(geometric_path);
         }
     }
     else
         std::cout << "No solution found." << std::endl;
}

void visualize(og::PathGeometric* path)
{

     visualization_msgs::Marker marker;
	   //marker.action = visualization_msgs::Marker::DELETEALL;
	   //vis_pub.publish(marker);

     for (std::size_t idx = 0; idx < path->getStateCount (); idx++)
	   {   std::cout<<"iteration "<<idx<<std::endl;
         // cast the abstract state type to the type we expect
		     const ob::SE3StateSpace::StateType *se3state = path->getState(idx)->as<ob::SE3StateSpace::StateType>();
         std::cout<<"iteration "<<idx<<std::endl;
	            // extract the first component of the state and cast it to what we expect
		     const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
         std::cout<<"iteration "<<idx<<std::endl;
	            // extract the second component of the state and cast it to what we expect
	       const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
         std::cout<<"iteration "<<idx<<std::endl;
				 marker.header.frame_id = "world";
				 marker.header.stamp = ros::Time();
				 marker.ns = "path";
				 marker.id = idx;
				 marker.type = visualization_msgs::Marker::CUBE;
				 marker.action = visualization_msgs::Marker::ADD;
				 marker.pose.position.x = pos->values[0];
				 marker.pose.position.y = pos->values[1];
				 marker.pose.position.z = 0;
         std::cout<<"iteration "<<idx<<std::endl;
				 marker.pose.orientation.x = rot->x;
				 marker.pose.orientation.y = rot->y;
			 	 marker.pose.orientation.z = rot->z;
				 marker.pose.orientation.w = rot->w;
         std::cout<<"iteration "<<idx<<std::endl;
				 marker.scale.x = 0.15;
				 marker.scale.y = 0.15;
				 marker.scale.z = 0.15;
				 marker.color.a = 1.0;
				 marker.color.r = 0.0;
				 marker.color.g = 1.0;
			   marker.color.b = 0.0;
				 vis_pub.publish(marker);
				 // ros::Duration(0.1).sleep();
				 std::cout << "Published marker: " << idx << std::endl;
		}
}

int main(int argc, char** argv)
{
    // The parsed arguments
    ros::init(argc, argv,"optimal_planning_node");
    ros::NodeHandle nh;
    std::string outputFile("outputfile");
    //std::cout << "main";
    ompl::msg::setLogLevel(ompl::msg::LOG_DEBUG);

  //  costmap_ros = new costmap_2d::Costmap2DROS("costmap", tf_);

    ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map",1,boost::bind(&costmapCallback, _1, outputFile));

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
