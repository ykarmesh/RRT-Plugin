#include "rrt_star/rrt.h"
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>

PLUGINLIB_EXPORT_CLASS(rrt_star::RRTPlan, nav_core::BaseGlobalPlanner)

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace rrt_star{

  RRTPlan::RRTPlan()
  :costmap_ros_(NULL), initialized_(false){}

  RRTPlan::RRTPlan(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  :costmap_ros_(NULL), initialized_(false)
  {
    initialize(name, costmap_ros);
  }

  RRTPlan::~RRTPlan()
  {
    delete world_model_;
  }

  void RRTPlan::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){

      //ROS_INFO("Initializing planner with costmap");

      ros::NodeHandle private_nh("~/" + name);
      //ros::NodeHandle move_base_nh("~");
      private_nh_ = private_nh;
      //move_base_nh_ = move_base_nh;

      //clear_costmap = move_base_nh_.serviceClient<std_srvs::Empty>("clear_costmaps");
      private_nh_.param("max_footprint_cost", max_footprint_cost_, 256);
      private_nh_.param("Inscribed_radius", inscribed_radius_, 0.25);
      private_nh_.param("Circumscribed_radius", circumscribed_radius_, 0.25);
      private_nh_.param("max_dist_bw_poses", max_dist_bw_poses_, 0.10);
      private_nh_.param("intermediate_poses", intermediate_poses_, true);

      plan_pub_ = private_nh_.advertise<nav_msgs::Path>("plan", 1);

      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros->getCostmap();
      world_model_ = new rrt_star::CostmapModel(*costmap_);

      prev_goal[0] = prev_goal[1] = 0;
      prev_start[0] = prev_start[1] = 0;

      initialized_ = true;

    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }


  bool RRTPlan::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan)
  { 
    if((goal.pose.position.x == prev_goal[0])&&(goal.pose.position.y == prev_goal[1]))
    {   ROS_INFO("Replanning");
      //TODO: What if the costmap has changed while start and goal are the same
      if((start.pose.position.x == prev_start[0])&&(goal.pose.position.y == prev_start[1]))
      { ROS_ERROR("No feasible path possible");
        return false;
      }
    }
    /*std_srvs::Empty srv;
    if(clear_costmap.call(srv))
    { ROS_INFO("Costmap successfully cleared");
    }
    else
    { ROS_ERROR("Failed to clear costmap");
      return false;
    }*/
    space = ob::StateSpacePtr(new ob::SE2StateSpace());

    // set the bounds
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, costmap_->getOriginX());
    bounds.setHigh(0, costmap_->getOriginX() + costmap_->getSizeInCellsX() * costmap_->getResolution());
    bounds.setLow(1, costmap_->getOriginY());
    bounds.setHigh(1, costmap_->getOriginY() + costmap_->getSizeInCellsY() * costmap_->getResolution());
    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    // Construct a space information instance for this state space
    si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

    // Set the function used to check which states in the space are valid
    si->setStateValidityChecker(boost::bind(&RRTPlan::isStateValid, this, _1));
    si->setStateValidityCheckingResolution(0.004);

    si->setup();

    // Create a problem instance
    pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

    // Set the optimization objective
    pdef->setOptimizationObjective(getPathLengthObjective(si));

    if(!setStart(start.pose))
      return false;
    if(!setGoal(goal.pose))
      return false;

    // Set the optimal planner
    ob::PlannerPtr optimizingPlanner = std::make_shared<og::InformedRRTstar>(si);

    // Set the problem instance for our planner to solve
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();

    // print the settings for this space
    //si->printSettings(std::cout);
    //pdef->print(std::cout);

    if (pdef->getStartStateCount() <= 0)
    {
      ROS_ERROR("No start state specified");
      return false;
    }
    else if (!pdef->getGoal())
    {
      ROS_ERROR("No goal specified");
      return false;
    }
     // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = optimizingPlanner->solve(1.25);

    if (solved)
    {
        // Output the length of the path found
        //ROS_INFO_STREAM(optimizingPlanner->getName()<< " found a solution of length "<< pdef->getSolutionPath()->length()
        //<< " with an optimization objective value of "<< pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()));

        path_ = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
        //path_->print(std::cout);

        convertPath(path_, plan);
        pdef->clearSolutionPaths();
        delete path_;
        return true;
    }
    else
    {
      ROS_ERROR("No solution found");
      return false;
    }
  }

  void RRTPlan::convertPath(const og::PathGeometric* path_, std::vector<geometry_msgs::PoseStamped>& plan)
  {
    std::vector<geometry_msgs::Pose2D> temp_path;
    geometry_msgs::Pose2D temp_pose;
    for(int i = 0; i < path_->getStateCount(); i++)
    {
      temp_pose.x = path_->getState(i)->as<ob::SE2StateSpace::StateType>()->getX();
      temp_pose.y = path_->getState(i)->as<ob::SE2StateSpace::StateType>()->getY();
      temp_pose.theta = path_->getState(i)->as<ob::SE2StateSpace::StateType>()->getYaw();
      temp_pose.theta = angles::normalize_angle(temp_pose.theta);

      temp_path.push_back(temp_pose);
    }
    if(intermediate_poses_)
    {
      if(!interpolatePath(temp_path))
        return;
      //ROS_INFO_STREAM("Total no of intermediate poses now equal to"<<temp_path.size());
    }
    geometry_msgs::PoseStamped pose;
    for(int i = 0; i < temp_path.size(); i++)
    {
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = costmap_ros_->getGlobalFrameID();
      tf::Quaternion quat;

      quat = tf::createQuaternionFromYaw(temp_path[i].theta);

      // set position
      pose.pose.position.x = temp_path[i].x;
      pose.pose.position.y = temp_path[i].y;
      pose.pose.position.z = 0.0;

      // set quaternion
      pose.pose.orientation.x = quat.x();
      pose.pose.orientation.y = quat.y();
      pose.pose.orientation.z = quat.z();
      pose.pose.orientation.w = quat.w();

      plan.push_back(pose);
    }
  }

  bool RRTPlan::interpolatePath(std::vector<geometry_msgs::Pose2D>& path)
	{
		std::vector<geometry_msgs::Pose2D> longPath;
		geometry_msgs::Pose2D last_frame, curr_frame, diff_frame, temp_frame;
		double frame_distance, num_insertions;

		if(path.size() < 2)
		{
			ROS_ERROR("Path is not valid. Interpolation not possible. Aborting.");
			return false;
		}

		// init plan with start frame
		longPath.push_back(path[0]);
    CollisionChecker(path[0]);
		// make sure plan is dense enough to be processed by local planner
		for(int i = 1; i < path.size(); i++)
		{
			// check whether current frame is close enough to last frame --> otherwise insert interpolated frames
			last_frame = longPath[(longPath.size()-1)];
			curr_frame = path[i];

			// calc distance between frames
			diff_frame.x = curr_frame.x - last_frame.x;
			diff_frame.y = curr_frame.y - last_frame.y;
			diff_frame.theta = curr_frame.theta - last_frame.theta;
			// normalize angle
			diff_frame.theta = angles::normalize_angle(diff_frame.theta);

			frame_distance = sqrt( diff_frame.x*diff_frame.x + diff_frame.y*diff_frame.y );

			// insert frames until path is dense enough
			if(frame_distance > max_dist_bw_poses_)
			{
				num_insertions = ceil(frame_distance/max_dist_bw_poses_);

				diff_frame.x = diff_frame.x/(num_insertions);
				diff_frame.y = diff_frame.y/(num_insertions);
				diff_frame.theta = diff_frame.theta/(num_insertions);
				for(int j = 1; j <= (int)num_insertions; j++)
				{
					temp_frame.x = last_frame.x + j*diff_frame.x;
					temp_frame.y = last_frame.y + j*diff_frame.y;
					temp_frame.theta = last_frame.theta + j*diff_frame.theta;
					// normalize angle
					temp_frame.theta = angles::normalize_angle(temp_frame.theta);

					// append frame to interpolated path
					longPath.push_back(temp_frame);
          CollisionChecker(temp_frame);
          //ROS_INFO_STREAM("New pose added at ["<< temp_frame.x <<"]["<< temp_frame.y <<"]["<< temp_frame.theta <<"]");
				}
			}

			// finally insert frame from path
			longPath.push_back(curr_frame);
      CollisionChecker(curr_frame);
		}

		path = longPath;
		return true;
	}

  bool RRTPlan::setStart(const geometry_msgs::Pose start)
  {
    geometry_msgs::Pose2D pose;
    double theta;
    pose.x = start.position.x;
    pose.y = start.position.y;
    pose.theta = getTheta(start.orientation);
    /*if(!CollisionChecker(pose))
    {
      ROS_ERROR("Start state is infeasible, cannot create a vaild path");
      return false;
    }*/
    ROS_INFO_STREAM("New start initialized at ["<< pose.x <<"]["<< pose.y <<"]["<<pose.theta<<"]");
    ob::ScopedState<> start_(space);
    start_->as<ob::SE2StateSpace::StateType>()->setX(pose.x);
    start_->as<ob::SE2StateSpace::StateType>()->setY(pose.y);
    start_->as<ob::SE2StateSpace::StateType>()->setYaw(pose.theta);
    prev_start[0] = pose.x;
    prev_start[1] = pose.y;
    pdef->clearStartStates();
    pdef->addStartState(start_);
    return true;
  }

  bool RRTPlan::setGoal(const geometry_msgs::Pose goal)
  {
    geometry_msgs::Pose2D pose;
    pose.x = goal.position.x;
    pose.y = goal.position.y;
    pose.theta = getTheta(goal.orientation);
    /*if(!CollisionChecker(pose))
    {
      ROS_ERROR("Goal state is infeasible, cannot create a vaild path");
      return false;
    }*/
    ROS_INFO_STREAM("New goal recieved at ["<< pose.x <<"]["<< pose.y <<"]["<<pose.theta<<"]");
    ob::ScopedState<> goal_(space);
    goal_->as<ob::SE2StateSpace::StateType>()->setX(pose.x);
    goal_->as<ob::SE2StateSpace::StateType>()->setY(pose.y);
    goal_->as<ob::SE2StateSpace::StateType>()->setYaw(pose.theta);
    prev_goal[0] = pose.x;
    prev_goal[1] = pose.y;
    pdef->setGoalState(goal_);
    return true;
  }

  double RRTPlan::getTheta(geometry_msgs::Quaternion msg)
  {
    tf::Quaternion q(msg.x, msg.y, msg.z, msg.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }

  ob::OptimizationObjectivePtr RRTPlan::getPathLengthObjective(const ob::SpaceInformationPtr& si)
  {
    return std::make_shared<ob::PathLengthOptimizationObjective>(si);
  }

  bool RRTPlan::isStateValid(const ompl::base::State *state)
  {
    geometry_msgs::Pose2D robot_pose;
    robot_pose.x = state->as<ob::SE2StateSpace::StateType>()->getX();
    robot_pose.y = state->as<ob::SE2StateSpace::StateType>()->getY();
    robot_pose.theta = state->as<ob::SE2StateSpace::StateType>()->getYaw();
    return(CollisionChecker(robot_pose));
  }

  bool RRTPlan::CollisionChecker(geometry_msgs::Pose2D robot_pose)
  { // build the oriented footprint
    std::vector<geometry_msgs::Point> footprint_ = costmap_ros_->getRobotFootprint();
    if(footprint_.size() < 3)
    {
      ROS_ERROR("Footprint not specified, assuming a point robot model");
      return false;
    }
    double cos_th = cos(robot_pose.theta);
    double sin_th = sin(robot_pose.theta);
    std::vector<geometry_msgs::Point> oriented_footprint;
    geometry_msgs::Point new_pt, robot_position;
    robot_position.x = robot_pose.x;
    robot_position.y = robot_pose.y;
    //std::cout<<"robot_pose "<<robot_pose.x<<" "<<robot_pose.y;
    unsigned int cell_x, cell_y;
    //get the cell coord of the center point of the robot
    costmap_->worldToMap(robot_pose.x, robot_pose.y, cell_x, cell_y);
    //std::cout<<" cell x "<<cell_x<<" cell y "<<cell_y<<" costmap frame "<<costmap_ros_->getGlobalFrameID()<<"\n";
    //std::cout<<" Pose Cost "<<(int)costmap_->getCost(cell_x, cell_y);

    for(unsigned int i = 0; i < footprint_.size(); i++){
      new_pt.x = robot_pose.x + (footprint_[i].x * cos_th + footprint_[i].y * sin_th);
      new_pt.y = robot_pose.y + (-footprint_[i].x * sin_th + footprint_[i].y * cos_th);
      oriented_footprint.push_back(new_pt);
  }

    // check if the footprint is legal
    double costs = world_model_->footprintCost(robot_position, oriented_footprint, inscribed_radius_, circumscribed_radius_);
    //std::cout<<" cost "<<costs<<std::endl;

    if( (costs >= -1) && (costs < max_footprint_cost_) )
    {
      return true;
    }
    return false;
  }
}
