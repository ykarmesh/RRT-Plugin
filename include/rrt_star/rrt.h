#ifndef RRT_PLAN_H
#define RRT_PLAN_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <angles/angles.h>
#include <rrt_star/costmap_model.h>
//#include <std_srvs/Empty.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>

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

namespace rrt_star
{

class RRTPlan : public nav_core::BaseGlobalPlanner
{

public:
    
    RRTPlan();

    RRTPlan(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    ~RRTPlan();

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan);

    void convertPath(const og::PathGeometric* path_, std::vector<geometry_msgs::PoseStamped>& plan);

    bool interpolatePath(std::vector<geometry_msgs::Pose2D>& path);

    bool setStart(const geometry_msgs::Pose start);

    bool setGoal(const geometry_msgs::Pose goal);

    double getTheta(geometry_msgs::Quaternion msg);

    ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);

    bool isStateValid(const ompl::base::State *state);

    bool CollisionChecker(geometry_msgs::Pose2D robot_pose);

    //void visualize(const og::PathGeometric* path);

private:

    ros::NodeHandle private_nh_;

    //ros::NodeHandle move_base_nh_;

    //ros::ServiceClient clear_costmap;

    ob::StateSpacePtr space;  // define the state space for planning

    ob::SpaceInformationPtr si; // construct an instance of  space information from this state space

    ob::ProblemDefinitionPtr pdef;  // create a problem instance

    const og::PathGeometric* path_;

    bool initialized_ = false;

    costmap_2d::Costmap2DROS* costmap_ros_;

    costmap_2d::Costmap2D* costmap_;

    int max_footprint_cost_;

    double inscribed_radius_, circumscribed_radius_,  max_dist_bw_poses_;

    bool intermediate_poses_;

    rrt_star::CostmapModel *world_model_;

    ros::Publisher plan_pub_;

    double prev_goal[2];
    double prev_start[2];

};

};

#endif
