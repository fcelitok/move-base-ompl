/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 * Modificator: Windel Bouwman
 *********************************************************************/

/*

    Greatly copied from:

    http://ompl.kavrakilab.org/RigidBodyPlanningWithControls_8cpp_source.html

*/

#include <ompl_global_planner.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>



//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(ompl_global_planner::OmplGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace ompl_global_planner
{


OmplGlobalPlanner::OmplGlobalPlanner() :
        _costmap_ros(NULL), _initialized(false), _allow_unknown(true),
        _se2_space(new ob::SE2StateSpace()),
        _velocity_space(new ob::RealVectorStateSpace(1)),
        _space(_se2_space + _velocity_space),
        _costmap_model(NULL)
{
}

void OmplGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    if (!_initialized)
    {
        ros::NodeHandle private_nh("~/" + name);
        _costmap_ros = costmap_ros;
        _frame_id = "map";
        _costmap_model = new base_local_planner::CostmapModel(*_costmap_ros->getCostmap());

        x_size_of_map = _costmap_ros->getCostmap()->getSizeInMetersX(); // x size of map
        y_size_of_map = _costmap_ros->getCostmap()->getSizeInMetersY(); // y size of map

        //take config parameter part
        private_nh.param("allow_unknown", _allow_unknown, true);
        private_nh.param("planner_type", _planner_type, std::string("RRTConnect"));
        private_nh.param("disable_poseArray", _disable_poseArray, false);
        private_nh.param("lethal_cost", _lethal_cost, 200.0);  // default 253.0
        private_nh.param("time_out", _time_out, 5.0);

        _plan_pub = private_nh.advertise<nav_msgs::Path>("plan", 1);

        if (_disable_poseArray == true)
        {
            // _samples_pub = private_nh.advertise<geometry_msgs::PoseArray>("samples", 1);
            _samples_pub = private_nh.advertise<sensor_msgs::PointCloud>("samples",1);
            _invalid_samples_pub = private_nh.advertise<sensor_msgs::PointCloud>("invalid_samples", 1);
        }

        //get the tf prefix
        ros::NodeHandle prefix_nh;
        tf_prefix_ = tf::getPrefixParam(prefix_nh);

        _initialized = true;
        ROS_INFO("Ompl global planner initialized!");
    }
    else
    {
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
    }

}

void OmplGlobalPlanner::get_xy_theta_v(const ob::State *s, double& x, double& y, double& theta, double& v)
{
    const ob::CompoundStateSpace::StateType* compound_state = s->as<ob::CompoundStateSpace::StateType>();
    const ob::SE2StateSpace::StateType* se2state = compound_state->as<ob::SE2StateSpace::StateType>(0);
    const ob::RealVectorStateSpace::StateType* v_state = compound_state->as<ob::RealVectorStateSpace::StateType>(1);

    // Get the values:
    x = se2state->getX();
    y = se2state->getY();
    theta = se2state->getYaw();
    v = (*v_state)[0];
}

// Store x,y and theta into state:
void OmplGlobalPlanner::set_xy_theta_v(ob::State* rs, double x, double y, double theta, double v)
{
    ob::CompoundStateSpace::StateType* compound_state = rs->as<ob::CompoundStateSpace::StateType>();
    ob::SE2StateSpace::StateType* se2state = compound_state->as<ob::SE2StateSpace::StateType>(0);
    ob::RealVectorStateSpace::StateType* v_state = compound_state->as<ob::RealVectorStateSpace::StateType>(1);

    // Set values:
    se2state->setX(x);
    se2state->setY(y);
    se2state->setYaw(theta);
    (*v_state)[0] = v;

    // Make sure angle is (-pi,pi]:
    const ob::SO2StateSpace* SO2 = _se2_space->as<ob::SE2StateSpace>()->as<ob::SO2StateSpace>(1);
    ob::SO2StateSpace::StateType* so2 = se2state->as<ob::SO2StateSpace::StateType>(1);
    SO2->enforceBounds(so2);
}

double OmplGlobalPlanner::calc_cost(const ob::State *state)
{
    double x, y, theta, velocity;
    get_xy_theta_v(state, x, y, theta, velocity);

    // Get the cost of the footprint at the current location:
    double footprint_cost = _costmap_model->footprintCost(x, y, theta, _costmap_ros->getRobotFootprint());

    if (footprint_cost == -1)  // footprint covers at least a lethal obstacle cell
    {
		return 255.0;
    }
	else if (footprint_cost == -2)  // footprint covers at least a no-information cell
	{
		return 0.0;
	}
	else if (footprint_cost == -3)  // footprint is [partially] outside of the map
	{
		return 255.0;
	} else
	{
		return footprint_cost;
	}

}

// Calculate the cost of a motion:
double OmplGlobalPlanner::motion_cost(const ob::State* s1, const ob::State* s2)
{
    // int nd = validSegmentCount(s1, s2);
    // TODO: interpolate?
    double cst = 0;

    std::cout << "OmplGlobalPlanner::motion_cost" << std::endl;

    // cst = 

    return cst;
}

// Check the current state:
bool OmplGlobalPlanner::isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    if (!si->satisfiesBounds(state))
    {
        return false;
    }

    // Get the cost of the footprint at the current location:
    double cost = calc_cost(state);
	double x, y, theta, velocity;
	get_xy_theta_v(state, x, y, theta, velocity);

	// Too high cost:
    if (cost >= _lethal_cost)
    {
        if (_disable_poseArray == true)
        {
            geometry_msgs::Point32 invalid_pose;
            invalid_pose.x = x;
            invalid_pose.y = y;
            invalid_pose.z = 0.0;
            _invalid_samples.points.push_back(invalid_pose);
        }

		return false;
    }

    if (_disable_poseArray == true) {
        geometry_msgs::Point32 pose;
        pose.x = x;
        pose.y= y;
        pose.z = 0.0;
        _samples.points.push_back(pose);
    }

    // Error? Unknown space?
    if (cost < 0)
    {
        std::cout << "COST LOWER THAN ZERO" << std::endl;
        return false;
    }

    return true;
}

// Calculate vehicle dynamics, assume a velocity state, but steering to be instantly possible.
void OmplGlobalPlanner::propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State* result)
{
    // Implement vehicle dynamics:
    double x, y, theta, velocity;
    get_xy_theta_v(start, x, y, theta, velocity);

    const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    double acc = ctrl[0];
    double steer_angle = ctrl[1];

    // Calculate vehicle dynamics:
    double x_n, y_n, theta_n, velocity_n;
    // TODO: elaborate further..
    x_n = x + velocity * duration * cos(theta);
    y_n = y + velocity * duration * sin(theta);
    velocity_n = velocity + acc * duration;

    double vehicle_length = 0.2;
    double lengthInv = 1 / vehicle_length;
    double omega = velocity * lengthInv * tan(steer_angle);
    theta_n = theta + omega * duration;

    // Store new state in result:
    set_xy_theta_v(result, x_n, y_n, theta_n, velocity_n);
}

double calcYaw(const geometry_msgs::Pose pose)
{
    double yaw, pitch, roll;
    tf::Pose pose_tf;
    tf::poseMsgToTF(pose, pose_tf);
    pose_tf.getBasis().getEulerYPR(yaw, pitch, roll);
    return yaw;
}

bool OmplGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan)
{
    boost::mutex::scoped_lock lock(_mutex);
    if (!_initialized)
    {
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;
    std::string global_frame = _frame_id;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
        return false;
    }

    if (tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
        return false;
    }

    //clear the starting cell within the costmap because we know it can't be an obstacle
    tf::Stamped<tf::Pose> start_pose;
    tf::poseStampedMsgToTF(start, start_pose);

    ROS_INFO("Thinking about OMPL path..");
    // Create OMPL problem:
    ob::RealVectorBounds bounds(2);
    double x_size_positive = x_size_of_map/2.0;
    double y_size_positive = y_size_of_map/2.0;
    double x_size_negative = x_size_of_map/(-2.0);
    double y_size_negative = y_size_of_map/(-2.0);
    bounds.setLow(0,x_size_negative);
    bounds.setHigh(0,x_size_positive);
    bounds.setLow(1,y_size_negative);
    bounds.setHigh(1,y_size_positive);
    _se2_space->as<ob::SE2StateSpace>()->setBounds(bounds);

    ob::RealVectorBounds velocity_bounds(1);
    velocity_bounds.setHigh(0.2);
    velocity_bounds.setLow(-0.2);
    _velocity_space->as<ob::RealVectorStateSpace>()->setBounds(velocity_bounds);

    oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(_space, 2));
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-0.3);
    cbounds.setHigh(0.3);
    cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

    // Create space information:
    oc::SpaceInformationPtr si(new oc::SpaceInformation(_space, cspace));

    si->setStateValidityChecker(boost::bind(&OmplGlobalPlanner::isStateValid, this, si.get(), _1));
    si->setStatePropagator(boost::bind(&OmplGlobalPlanner::propagate, this, _1, _2, _3, _4));


    //added because of warnings
    si->setMinMaxControlDuration(1,10);
    si->setPropagationStepSize(0.2);

    // Define problem:
    ob::ScopedState<> ompl_start(_space);
    ompl_start[0] = start.pose.position.x;
    ompl_start[1] = start.pose.position.y;
    ompl_start[2] = calcYaw(start.pose);
    ompl_start[3] = 0; // Speed

    ob::ScopedState<> ompl_goal(_space);
    ompl_goal[0] = goal.pose.position.x;
    ompl_goal[1] = goal.pose.position.y;
    ompl_goal[2] = calcYaw(start.pose);
    ompl_goal[3] = 0; // Speed


    // Optimize criteria:
    //ob::OptimizationObjectivePtr clearance(getClearanceObjective(si));
    //ob::OptimizationObjectivePtr cost_objective(new CostMapObjective(*this, si));
    ob::OptimizationObjectivePtr length_objective(new ob::PathLengthOptimizationObjective(si));
    //ob::OptimizationObjectivePtr objective(new CostMapWorkObjective(*this, si));

    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

    //added from Sauravag fork
    //ob::MultiOptimizationObjective* multOptimObjective = new ob::MultiOptimizationObjective(si);
    //multOptimObjective->addObjective(cost_objective, 5.0);
    //multOptimObjective->addObjective(length_objective, 1.0);

    pdef->setStartAndGoalStates(ompl_start, ompl_goal, 0.05);
    //pdef->setOptimizationObjective(clearance);
    //pdef->setOptimizationObjective(cost_objective);
    //pdef->setOptimizationObjective(cost_objective + length_objective);
    pdef->setOptimizationObjective(length_objective);

    ROS_INFO("Problem defined, running planner");

    if(_planner_type == "RRTConnect"){
        _planner = ob::PlannerPtr(new og::RRTConnect(si));
    }
    else if(_planner_type == "RRTStar"){
        //auto aa(std::make_shared<og::RRTstar>(si));
        //aa->printSettings(std::cout);
        //_planner = ob::PlannerPtr(aa);
        _planner = ob::PlannerPtr(new og::RRTstar(si));
    }
    else if(_planner_type == "PRMStar"){
        _planner = ob::PlannerPtr(new og::PRMstar(si));
    }
    else{
        ROS_ERROR("Planner type did not defined");
    }

    _planner->setProblemDefinition(pdef);
    _planner->setup();


    ob::PlannerStatus solved = _planner->solve(_time_out);

    // Convert path into ROS messages:
    if (solved.asString() == "Exact solution")  // TODO check if all samples were interpolated
    {
        ROS_INFO("Ompl done!");
        ob::PathPtr result_path1 = pdef->getSolutionPath();

        // Cast path into geometric path:
        og::PathGeometric& result_path = static_cast<og::PathGeometric&>(*result_path1);
        result_path.interpolate();

        // Create path:
        plan.push_back(start);

        // Conversion loop from states to messages:
        std::vector<ob::State*>& result_states = result_path.getStates();
        for (std::vector<ob::State*>::iterator it = result_states.begin(); it != result_states.end(); ++it)
        {
            // Get the data from the state:
            double x, y, theta, velocity;
            get_xy_theta_v(*it, x, y, theta, velocity);

            // Place data into the pose:
            geometry_msgs::PoseStamped ps = goal;
            ps.header.stamp = ros::Time::now();
            ps.pose.position.x = x;
            ps.pose.position.y = y;
            plan.push_back(ps);
        }

        plan.push_back(goal);

    }
    else if (solved.asString() == "Approximate solution"){
        ROS_WARN("Did not find Exact Solution");
        //plan.push_back(start);
        //plan.push_back(start);
        return false;
    }
    else
    {
        ROS_ERROR("Failed to determine plan");
    }

    //publish the plan for visualization purposes
    publishPlan(plan);
    return !plan.empty();

}

void OmplGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{
    if (!_initialized) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan 
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());


    if (!path.empty()) {
        gui_path.header.frame_id = path[0].header.frame_id;
        gui_path.header.stamp = path[0].header.stamp;
    }


    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    // for visualisation
    if (_disable_poseArray == true )
    {
        _samples.header.frame_id = "map";
        _samples.header.stamp = ros::Time::now();
        _invalid_samples.header.frame_id = "map";
        _invalid_samples.header.stamp = ros::Time::now();
        _samples_pub.publish(_samples);
        _invalid_samples_pub.publish(_invalid_samples);
        _samples.points.clear();
        _invalid_samples.points.clear();
    }

    _plan_pub.publish(gui_path);

}

/*
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
            return ob::Cost(1 / (si_->getStateValidityChecker()->clearance(s) +
                                 std::numeric_limits<double>::min()));
        }
    };*/

/** Return an optimization objective which attempts to steer the robot
    away from obstacles. */
/*    ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si)
    {
        return std::make_shared<ClearanceObjective>(si);
    }*/



} //end namespace global_planner

