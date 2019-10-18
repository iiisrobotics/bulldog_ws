//---------- pick and place pipeline
#include "pick_and_place/pick_and_place_pipeline.h"
//----------

//---------- moveit
#include <moveit/robot_state/conversions.h>
//----------


namespace  pick_and_place_pipeline
{

PickAndPlacePipeline::PickAndPlacePipeline(ros::NodeHandle node) :
    node_(node)
{
    //
    // load names from ros parameter server
    //
    node_.getParam("end_effector_group_name", end_effector_group_name_);
    node_.getParam("arm_group_name", arm_group_name_);
    node_.getParam("gripper_command_topic", gripper_command_topic_);
    node_.getParam("gripper_status_topic", gripper_status_topic_);

    ROS_DEBUG_STREAM("End effector group name: " << end_effector_group_name_);
    ROS_DEBUG_STREAM("Arm group name: " << arm_group_name_);
    ROS_DEBUG_STREAM("Gripper command topic: " << gripper_command_topic_);
    ROS_DEBUG_STREAM("Gripper status topic: " << gripper_status_topic_);

    //
    // load planning scene
    //
    planning_scene_monitor_ptr_.reset(
        new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    if (!planning_scene_monitor_ptr_->getPlanningScene()) {
        ROS_ERROR_STREAM("Planning scene not configured!");
        return;
    }
    else {
        ROS_DEBUG_STREAM("Planning scene configured!");
    }
    planning_scene_monitor_ptr_->startPublishingPlanningScene(
        planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
        "/move_group/monitored_planning_scene"
    );
    planning_scene_monitor_ptr_->startStateMonitor();
    planning_scene_monitor_ptr_->startSceneMonitor();
    planning_scene_monitor_ptr_->startWorldGeometryMonitor();

    //
    // load robot model
    //
    robot_model_ptr_ = planning_scene_monitor_ptr_->getRobotModel();

    //
    // load arm group
    //
    arm_group_ptr_ = robot_model_ptr_->getJointModelGroup(arm_group_name_);

    //
    // load move group interface
    //
    ROS_DEBUG_STREAM("Loading move group...");
    move_group_ptr_.reset(new moveit::planning_interface::MoveGroupInterface(
        arm_group_name_));
    ROS_DEBUG_STREAM("Move group loaded!");

    //
    // load motion planning pipeline
    //
    ROS_DEBUG_STREAM("Loading planning pipeline...");
    planning_pipeline_ptr_.reset(new planning_pipeline::PlanningPipeline(
        robot_model_ptr_, node_, "planning_plugin", "request_adapters"));
    ROS_DEBUG_STREAM("Planning pipeline loaded!");

    //
    // setup rviz visual tools
    //
    // ROS_DEBUG_STREAM("robot_model_frame: " << robot_model_ptr_->getModelFrame()); // world
    visual_tools_ptr_.reset(new moveit_visual_tools::MoveItVisualTools(
        robot_model_ptr_->getModelFrame(),
        "visual_tools",
        planning_scene_monitor_ptr_
    ));

    visual_tools_ptr_->loadMarkerPub();
    visual_tools_ptr_->loadRobotStatePub("display_robot_state");
    visual_tools_ptr_->loadTrajectoryPub("/move_group/display_planned_path");
    visual_tools_ptr_->loadSharedRobotState();
    visual_tools_ptr_->deleteAllMarkers();
    visual_tools_ptr_->removeAllCollisionObjects();
    visual_tools_ptr_->enableBatchPublishing();
    // visual_tools_ptr_->hideRobot();
    // visual_tools_ptr_->trigger();

    //
    // publish world frame
    //
    visual_tools_ptr_->publishAxisLabeled(Eigen::Affine3d::Identity(), "odom");
    visual_tools_ptr_->trigger();

    //
    // load grasp data
    //
    grasp_data_ptr_.reset(new moveit_grasps::GraspData(
        node_, end_effector_group_name_, robot_model_ptr_
    ));

    //
    // load grasp filter
    //
    {
        moveit::core::RobotStatePtr current_state_ptr;
        planning_scene_monitor::LockedPlanningSceneRO 
        locked_planning_scene_ro_ptr(planning_scene_monitor_ptr_);
        current_state_ptr.reset(new moveit::core::RobotState(
            locked_planning_scene_ro_ptr->getCurrentState())
        );
        grasp_filter_ptr_.reset(new moveit_grasps::GraspFilter(
            current_state_ptr, visual_tools_ptr_)
        );
    }

    //
    // load grasp planner
    //
    grasp_planner_ptr_.reset(new moveit_grasps::GraspPlanner(visual_tools_ptr_));

    //
    // load gripper commander
    //
    gripper_commander_ptr_.reset(
        new robotiq_3f_gripper_commander::Robotiq3FGripperCommander(
            node_, gripper_command_topic_, gripper_status_topic_, 10.0));
}

PickAndPlacePipeline::~PickAndPlacePipeline()
{
    gripper_commander_ptr_->deactivate();
}

bool PickAndPlacePipeline::run(
    std::vector<geometry_msgs::PoseStamped>& grasp_poses)
{
    bool success = false;

    /**
     *  activate gripper
     */
    if (!gripper_commander_ptr_->activate()) {
        ROS_ERROR_STREAM("Activate gripper failed!");
        return success;
    }
    else {
        ROS_DEBUG_STREAM("Activate gripper succeeded");
    }

    /**
     *  move the arm to random valid state
     */
    // moveit::planning_interface::MoveItErrorCode temp_error_code =
    //     moveit::planning_interface::MoveItErrorCode::FAILURE;

    // while (temp_error_code != 
    //     moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    //     move_group_ptr_->setRandomTarget();
    //     temp_error_code = move_group_ptr_->move();
    // }

    //
    // setup target pose
    //
    geometry_msgs::PoseStamped target_pose;
    // target_pose = grasp_poses[0];
    target_pose.header.frame_id = robot_model_ptr_->getModelFrame();
    target_pose.header.stamp = ros::Time::now();
    ROS_DEBUG_STREAM("Robot model frame: " << target_pose.header.frame_id);
    
    // Selected Grasp: 
    //
    // left_arm_gripper_side
    //
    // target_pose.pose.position.x = 0.47719;
    // target_pose.pose.position.y = 0.036068;
    // target_pose.pose.position.z = 0.60215;
    // target_pose.pose.orientation.x = -0.65325;
    // target_pose.pose.orientation.y = 0.65332;
    // target_pose.pose.orientation.z = -0.27044;
    // target_pose.pose.orientation.w = 0.27076;
    //
    // left_arm_gripper_top forward
    // 
    // target_pose.pose.position.x = 0.64906;
    // target_pose.pose.position.y = 0.15092;
    // target_pose.pose.position.z = 0.70773;
    // target_pose.pose.orientation.x = -0.65321;
    // target_pose.pose.orientation.y = 0.65332;
    // target_pose.pose.orientation.z = -0.27047;
    // target_pose.pose.orientation.w = 0.27081;
    // 
    // left_arm_gripper_around
    //
    // target_pose.pose.position.x = 0.2269;
    // target_pose.pose.position.y = 0.4783;
    // target_pose.pose.position.z = 0.71926;
    // target_pose.pose.orientation.x = 0.86246;
    // target_pose.pose.orientation.y = -0.34955;
    // target_pose.pose.orientation.z = -0.15566;
    // target_pose.pose.orientation.w = -0.33128;

    /**
     *  generate grasp candidates
     */
    std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates_ptrs;
    if (!generateGrasps(target_pose, grasp_candidates_ptrs)) {
        ROS_ERROR_STREAM("Grasps generation failed!");
        return success;
    }
    else {
        ROS_DEBUG_STREAM("Grasps generation succeeded");
    }

    /**
     *  generating a seed state for filtering grasps
     */
    moveit::core::RobotStatePtr seed_state_ptr;
    Eigen::Affine3d target_pose_eigen;
    tf::poseMsgToEigen(target_pose.pose, target_pose_eigen);
    ROS_DEBUG_STREAM("Tip link: " << grasp_data_ptr_->parent_link_->getName());
    if (!getIKSolution(
            arm_group_ptr_, 
            target_pose_eigen, 
            seed_state_ptr, 
            grasp_data_ptr_->parent_link_->getName()
        )) {
        ROS_ERROR_STREAM("Get inverse kinematics solution failed!");
        return success;
    }
    else {
        ROS_DEBUG_STREAM("Get inverse kinematics solution succeeded!");
    }

    /**
     *  filter grasp candidates: use the initial seed state to compute inverse 
     *                           kinematics
     */
    // moveit_grasps::GraspCandidatePtr grasp_candidate_ptr = grasp_candidates_ptrs[0];
    // geometry_msgs::PoseStamped grasp_pose = grasp_candidate_ptr->grasp_.grasp_pose;
    // kinematics::KinematicsBaseConstPtr kin_solver_ptr = arm_group_ptr_->getSolverInstance();
    // std::string ik_frame = kin_solver_ptr->getBaseFrame();
    // Eigen::Affine3d link_transform;
    // ROS_DEBUG_STREAM(
    //     "Frame transform from ik_frame: " 
    //     << ik_frame 
    //     << " to robot model frame: " 
    //     << seed_state_ptr->getRobotModel()->getModelFrame()
    // );

    // if (!moveit::core::Transforms::sameFrame(ik_frame, seed_state_ptr->getRobotModel()->getModelFrame()))
    // {
    //     std::cout << ((!ik_frame.empty() && ik_frame[0] == '/') ? ik_frame.substr(1) : ik_frame) << std::endl;
    //     const robot_model::LinkModel* lm =
    //         seed_state_ptr->getLinkModel((!ik_frame.empty() && ik_frame[0] == '/') ? ik_frame.substr(1) : ik_frame);

    //     if (!lm)
    //     {
    //         ROS_ERROR_STREAM("Unable to find frame for link transform");
    //     }

    //     link_transform = seed_state_ptr->getGlobalLinkTransform(lm).inverse();
    // }

    // //
    // // grasp solution
    // //
    // // Transform current pose to frame of planning group
    // Eigen::Affine3d eigen_pose;
    // geometry_msgs::Pose grasp_pose_transformed;
    // tf::poseMsgToEigen(grasp_pose.pose, eigen_pose);
    // eigen_pose = link_transform * eigen_pose;
    // tf::poseEigenToMsg(eigen_pose, grasp_pose_transformed);

    // std::cout << "---------- Grasp pose in the '" << ik_frame << "' frame ----------" << std::endl;
    // std::cout << "x = " << grasp_pose_transformed.position.x << std::endl;
    // std::cout << "y = " << grasp_pose_transformed.position.y << std::endl;
    // std::cout << "z = " << grasp_pose_transformed.position.z << std::endl;
    // std::cout << "o_x = " << grasp_pose_transformed.orientation.x << std::endl;
    // std::cout << "o_y = " << grasp_pose_transformed.orientation.y << std::endl;
    // std::cout << "o_z = " << grasp_pose_transformed.orientation.z << std::endl;
    // std::cout << "o_w = " << grasp_pose_transformed.orientation.w << std::endl;
    // std::cout << "---------------------------------" << std::endl;

    // // search IK solution
    // std::vector<double> ik_seed_state;
    // seed_state_ptr->copyJointGroupPositions(arm_group_ptr_, ik_seed_state);

    // std::cout << "---------- Seed joint value ----------" << std::endl;
    // for (auto it = ik_seed_state.begin(); it != ik_seed_state.end(); it++) {
    //     std::cout << *it << std::endl;
    // }
    // std::cout << "---------------------------------" << std::endl;


    // std::vector<double> ik_solution;
    // moveit_msgs::MoveItErrorCodes error_code;

    // kin_solver_ptr->searchPositionIK(
    //     grasp_pose_transformed, 
    //     ik_seed_state, 
    //     arm_group_ptr_->getDefaultIKTimeout(), // 0.005 in kinematics.yaml
    //     ik_solution, 
    //     error_code
    // );

    // if (error_code.val == moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION) {
    //     // The grasp was valid but the pre-grasp was not
    //     ROS_ERROR_STREAM("No grasp IK solution");
    //     return;
    // }
    // else if (error_code.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT) {
    //     ROS_ERROR_STREAM("Grasp IK Timed Out");
    //     return;
    // }
    // else if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
    //     ROS_ERROR_STREAM("Unknown MoveItErrorCode from IK solver: " << error_code.val);
    //     return;
    // }
    // else {
    //     ik_seed_state = ik_solution;
    //     grasp_candidate_ptr->grasp_ik_solution_ = ik_solution;
    //     ROS_DEBUG_STREAM("Grasp IK solution found!");
    // }

    // std::cout << "---------- IK solution ----------" << std::endl;
    // for (auto it = ik_solution.begin(); it != ik_solution.end(); it++) {
    //     std::cout << *it << std::endl;
    // }
    // std::cout << "---------------------------------" << std::endl;

    // //
    // // pre-grasp solution
    // //
    // // convert to a pre-grasp
    // const std::string& ee_parent_link_name = grasp_candidate_ptr->grasp_data_->ee_jmg_->getEndEffectorParentGroup().second;
    // geometry_msgs::PoseStamped pre_grasp_pose = moveit_grasps::GraspGenerator::getPreGraspPose(
    //     grasp_candidate_ptr, ee_parent_link_name);

    // // Transform current pose to frame of planning group
    // geometry_msgs::Pose pre_grasp_pose_transformed;
    // tf::poseMsgToEigen(pre_grasp_pose.pose, eigen_pose);
    // eigen_pose = link_transform * eigen_pose;
    // tf::poseEigenToMsg(eigen_pose, pre_grasp_pose_transformed);

    // kin_solver_ptr->searchPositionIK(
    //     pre_grasp_pose_transformed, 
    //     ik_seed_state, 
    //     arm_group_ptr_->getDefaultIKTimeout(),
    //     ik_solution, 
    //     error_code
    // );

    // if (error_code.val == moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION) {
    //     ROS_ERROR_STREAM("No pre-grasp IK solution");
    //     return;
    // }
    // else if (error_code.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT) {
    //     ROS_ERROR_STREAM("Pre-grasp IK Timed Out");
    //     return;
    // }
    // else if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
    //     ROS_ERROR_STREAM("Unknown MoveItErrorCode from IK solver: " << error_code.val);
    //     return;
    // }
    // else {
    //     grasp_candidate_ptr->pregrasp_ik_solution_ = ik_solution;
    //     ROS_DEBUG_STREAM("Pre-grasp IK solution found!");
    // }

    // std::cout << "---------- Pre-grasp IK solution ----------" << std::endl;
    // for (auto it = ik_solution.begin(); it != ik_solution.end(); it++) {
    //     std::cout << *it << std::endl;
    // }
    // std::cout << "---------------------------------" << std::endl;

    //
    // grasp filter
    //
    bool filter_pregrasps = true;
    if (!grasp_filter_ptr_->filterGrasps(
            grasp_candidates_ptrs, 
            planning_scene_monitor_ptr_, 
            arm_group_ptr_, 
            seed_state_ptr,
            filter_pregrasps)) {
        ROS_ERROR_STREAM("Grasps filtering failed!");
        return success;
    }
    else {
        ROS_DEBUG_STREAM("Grasps filtering succeeded");
    }
    if (!grasp_filter_ptr_->removeInvalidAndFilter(grasp_candidates_ptrs)) {
        ROS_ERROR_STREAM("Grasps filtering remove all grasps!");
        return success;
    }
    ROS_INFO_STREAM(grasp_candidates_ptrs.size() << " grasps remain after filtering");

    std::cout << "---------- IK solution from moveit_grasps ----------" << std::endl;
    for (auto it = grasp_candidates_ptrs[0]->grasp_ik_solution_.begin(); 
        it != grasp_candidates_ptrs[0]->grasp_ik_solution_.end(); 
        it++) {
        std::cout << *it << std::endl;
    }
    std::cout << "---------------------------------" << std::endl;

    std::cout << "---------- Pre-grasp IK solution from moveit_grasps ----------" << std::endl;
    for (auto it = grasp_candidates_ptrs[0]->pregrasp_ik_solution_.begin(); 
        it != grasp_candidates_ptrs[0]->pregrasp_ik_solution_.end(); 
        it++) {
        std::cout << *it << std::endl;
    }
    std::cout << "---------------------------------" << std::endl;

    /**
     *  pre-approach motion planning
     */
    moveit_grasps::GraspCandidatePtr grasp_candidate_ptr = 
        grasp_candidates_ptrs[0];

    //
    // critical waypoints
    //
    EigenSTL::vector_Affine3d eigen_waypoints;
    moveit_grasps::GraspGenerator::getGraspWaypoints(
        grasp_candidate_ptr, eigen_waypoints);

    visual_tools_ptr_->publishAxisLabeled(eigen_waypoints[0], "pre-grasp");
    visual_tools_ptr_->publishAxisLabeled(eigen_waypoints[1], "grasp");
    visual_tools_ptr_->publishAxisLabeled(eigen_waypoints[2], "lifted");
    visual_tools_ptr_->publishAxisLabeled(eigen_waypoints[3], "retreat");
    visual_tools_ptr_->trigger();

    moveit::core::RobotState pre_grasp_state(robot_model_ptr_);
    pre_grasp_state.setJointGroupPositions(
        arm_group_ptr_, grasp_candidate_ptr->pregrasp_ik_solution_);

    moveit::planning_interface::MoveItErrorCode move_group_error_code;
    moveit::planning_interface::MoveGroupInterface::Plan pre_approach_plan;
    move_group_error_code = planTargetState(pre_grasp_state, pre_approach_plan);
    if (move_group_error_code == 
        moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_DEBUG_STREAM("Pre-approach planning succeeded");
    }
    else if (move_group_error_code == 
        moveit::planning_interface::MoveItErrorCode::TIMED_OUT) {
        ROS_ERROR_STREAM("Pre-approach planning timeout!");
        return success;        
    }
    else {
        ROS_ERROR_STREAM("Unknown pre-approach planning error!");
        return success;
    }

    //
    // execute the pre-approach plan
    //
    visual_tools_ptr_->publishRobotState(
        pre_grasp_state, rviz_visual_tools::colors::GREEN);
    visual_tools_ptr_->trigger();

    return false;

    visual_tools_ptr_->prompt("Press NEXT to show the pre-approach motion planning\n");
    visual_tools_ptr_->publishTrajectoryPath(pre_approach_plan.trajectory_, 
        pre_approach_plan.start_state_, true);
    visual_tools_ptr_->trigger();

    move_group_ptr_->execute(pre_approach_plan);
    move_group_ptr_->setStartStateToCurrentState();

    /**
     *  planning Cartesian path
     */
    const double eef_step = 0.01;
    const double jump_threshold = 4.0;

    //
    // compute the approach Cartesian path
    //
    geometry_msgs::Pose pre_grasp_waypoint, grasp_waypoint;
    std::vector<geometry_msgs::Pose> approach_waypoints;
    tf::poseEigenToMsg(eigen_waypoints[0], pre_grasp_waypoint);
    tf::poseEigenToMsg(eigen_waypoints[1], grasp_waypoint);
    approach_waypoints.push_back(pre_grasp_waypoint);
    approach_waypoints.push_back(grasp_waypoint);

    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    move_group_error_code = planCartesianPath(
        approach_waypoints, approach_plan);
    if (move_group_error_code == 
        moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_DEBUG_STREAM("Approach planning succeeded");
    }
    else if (move_group_error_code == 
        moveit::planning_interface::MoveItErrorCode::TIMED_OUT) {
        ROS_ERROR_STREAM("Approach planning timeout!");
        return success;
    }
    else {
        ROS_ERROR_STREAM("Unknown approach planning error!");
        return success;
    }

    //
    // execute the approach plan
    //
    visual_tools_ptr_->prompt("Press NEXT to show the approach motion planning\n");
    visual_tools_ptr_->publishTrajectoryPath(approach_plan.trajectory_, 
        approach_plan.start_state_, true);
    visual_tools_ptr_->trigger();    

    move_group_ptr_->execute(approach_plan);
    move_group_ptr_->setStartStateToCurrentState();

    //
    // close gripper
    //
    if (!gripper_commander_ptr_->close()) {
        ROS_ERROR_STREAM("Close gripper failed!");
    }
    else {
        ROS_DEBUG_STREAM("Close gripper succeeded");
    }

    //
    // compute the lift Cartesian path
    //
    geometry_msgs::Pose lifted_waypoint;
    std::vector<geometry_msgs::Pose> lift_waypoints;
    tf::poseEigenToMsg(eigen_waypoints[2], lifted_waypoint);
    lift_waypoints.push_back(grasp_waypoint);
    lift_waypoints.push_back(lifted_waypoint);

    moveit::planning_interface::MoveGroupInterface::Plan lift_plan;
    move_group_error_code = planCartesianPath(
        lift_waypoints, lift_plan);
    if (move_group_error_code == 
        moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_DEBUG_STREAM("Lift planning succeeded");
    }
    else if (move_group_error_code == 
        moveit::planning_interface::MoveItErrorCode::TIMED_OUT) {
        ROS_ERROR_STREAM("Lift planning timeout!");
        return success;
    }
    else {
        ROS_ERROR_STREAM("Unknown lift planning error!");
        return success;
    }

    //
    // execute the lift plan
    //
    visual_tools_ptr_->prompt("Press NEXT to show the lift motion planning\n");
    visual_tools_ptr_->publishTrajectoryPath(lift_plan.trajectory_, 
        lift_plan.start_state_, true);
    visual_tools_ptr_->trigger();

    move_group_ptr_->execute(lift_plan);
    move_group_ptr_->setStartStateToCurrentState();

    //
    // compute the retreat Cartesian path
    //
    geometry_msgs::Pose retreat_waypoint;
    tf::poseEigenToMsg(eigen_waypoints[3], retreat_waypoint);
    std::vector<geometry_msgs::Pose> retreat_waypoints;
    retreat_waypoints.push_back(lifted_waypoint);
    retreat_waypoints.push_back(retreat_waypoint);

    moveit::planning_interface::MoveGroupInterface::Plan retreat_plan;
    move_group_error_code = planCartesianPath(
        retreat_waypoints, retreat_plan);
    if (move_group_error_code == 
        moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_DEBUG_STREAM("Retreat planning succeeded");
    }
    else if (move_group_error_code == 
        moveit::planning_interface::MoveItErrorCode::TIMED_OUT) {
        ROS_ERROR_STREAM("Retreat planning timeout!");
        return success;
    }
    else {
        ROS_ERROR_STREAM("Unknown retreat planning error!");
        return success;
    }

    //
    // execute the retreat plan
    //
    visual_tools_ptr_->prompt("Press NEXT to show the lift motion planning\n");
    visual_tools_ptr_->publishTrajectoryPath(retreat_plan.trajectory_, 
        retreat_plan.start_state_, true);
    visual_tools_ptr_->trigger();

    move_group_ptr_->execute(retreat_plan);
    move_group_ptr_->setStartStateToCurrentState();

    /**
     *  post-retreat motion planning
     */
    moveit::core::RobotState default_state(robot_model_ptr_);
    default_state = getNamedTargetState("left_arm_default");

    moveit::planning_interface::MoveGroupInterface::Plan post_retreat_plan;
    move_group_error_code = planTargetState(default_state, post_retreat_plan);
    if (move_group_error_code == 
        moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_DEBUG_STREAM("Post-retreat planning succeeded");
    }
    else if (move_group_error_code == 
        moveit::planning_interface::MoveItErrorCode::TIMED_OUT) {
        ROS_ERROR_STREAM("Post-retreat planning timeout!");
        return success;
    }
    else {
        ROS_ERROR_STREAM("Unknown pre-approach planning error!");
        return success;
    }

    // execute the post-retreat plan
    visual_tools_ptr_->prompt("Press NEXT to show the post-retreat motion planning\n");
    visual_tools_ptr_->publishTrajectoryPath(post_retreat_plan.trajectory_, 
        post_retreat_plan.start_state_, true);
    visual_tools_ptr_->trigger();

    move_group_ptr_->execute(post_retreat_plan);
    move_group_ptr_->setStartStateToCurrentState();

    // open gripper
    if (!gripper_commander_ptr_->open()) {
        ROS_ERROR_STREAM("Open gripper failed!");
        return success;
    }
    else {
        ROS_DEBUG_STREAM("Open gripper succeeded");
    }

    /**
     *  deactivate gripper
     */
    if (!gripper_commander_ptr_->deactivate()) {
        ROS_ERROR_STREAM("Gripper deactivation failed!");
        return success;
    }
    else {
        ROS_DEBUG_STREAM("Gripper deactivation succeeded");
    }

    success = true;
    return success;
}

bool PickAndPlacePipeline::generateGrasps(
    geometry_msgs::PoseStamped& grasp_pose,
    std::vector<moveit_grasps::GraspCandidatePtr>& grasp_candidates_ptrs)
{
    //
    // justify the end effector type
    //
    if (grasp_data_ptr_->end_effector_type_ != moveit_grasps::FINGER) {
        ROS_ERROR_STREAM("End effector type must be FINGER!");
        return false;
    }
    else {
        ROS_DEBUG_STREAM("End effector type confirmed!");
    }

    //
    // convert grasp_pose to Eigen::Affine3d matrix
    //
    Eigen::Affine3d grasp_pose_eigen;
    tf::poseMsgToEigen(grasp_pose.pose, grasp_pose_eigen);

    //
    // the grasp
    //
    moveit_msgs::Grasp grasp;

    //
    // name the grasp
    //
    static std::size_t grasp_id = 0;
    grasp.id = "grasp" + boost::lexical_cast<std::string>(grasp_id);
    grasp_id++;

    //
    // pregrasp approach - aligned with the parent link of the end effector 
    //                     group
    //
    Eigen::Vector3d pre_grasp_approach_vector = 
        -1.0 * grasp_data_ptr_->grasp_pose_to_eef_pose_.translation();
    pre_grasp_approach_vector.normalize();

    grasp.pre_grasp_approach.desired_distance = 
        grasp_data_ptr_->grasp_max_depth_ + 
        grasp_data_ptr_->approach_distance_desired_;
    grasp.pre_grasp_approach.min_distance = 0.0;// NOT IMPLEMENTED
    
    grasp.pre_grasp_approach.direction.header.frame_id = 
        grasp_data_ptr_->parent_link_->getName();
    grasp.pre_grasp_approach.direction.header.stamp = ros::Time::now();
    grasp.pre_grasp_approach.direction.vector.x = 
        pre_grasp_approach_vector.x();
    grasp.pre_grasp_approach.direction.vector.y = 
        pre_grasp_approach_vector.y();
    grasp.pre_grasp_approach.direction.vector.z = 
        pre_grasp_approach_vector.z();

    //
    // postgrasp retreat - aligned with the parent link of the end effector
    //                     group 
    //
    grasp.post_grasp_retreat.desired_distance = 
        grasp_data_ptr_->grasp_max_depth_ + 
        grasp_data_ptr_->retreat_distance_desired_;
    grasp.post_grasp_retreat.min_distance = 0.0;// NOT IMPLEMENTED
    
    grasp.post_grasp_retreat.direction.header.frame_id = 
        grasp_data_ptr_->parent_link_->getName();
    grasp.post_grasp_retreat.direction.header.stamp = ros::Time::now();
    grasp.post_grasp_retreat.direction.vector.x = 
        -1.0 * pre_grasp_approach_vector.x();
    grasp.post_grasp_retreat.direction.vector.y = 
        -1.0 * pre_grasp_approach_vector.y();
    grasp.post_grasp_retreat.direction.vector.z = 
        -1.0 * pre_grasp_approach_vector.z();

    //
    // grasp pose - aligend with the base frame of the entire robot
    //
    grasp.grasp_pose.header.frame_id = grasp_data_ptr_->base_link_;
    grasp.grasp_pose.header.stamp = ros::Time::now();

    // Eigen::Affine3d end_effector_pose_eigen = 
    //     grasp_pose_eigen * grasp_data_ptr_->grasp_pose_to_eef_pose_;
    Eigen::Affine3d end_effector_pose_eigen = 
        grasp_pose_eigen;
    tf::poseEigenToMsg(end_effector_pose_eigen, grasp.grasp_pose.pose);

    //
    // pregrasp posture - open the gripper
    //
    grasp.pre_grasp_posture = grasp_data_ptr_->pre_grasp_posture_;

    //
    // grasp posture - close the gripper
    //
    grasp.grasp_posture = grasp_data_ptr_->grasp_posture_;

    //
    // add and check the grasp candidates
    //
    grasp_candidates_ptrs.push_back(moveit_grasps::GraspCandidatePtr(
        new moveit_grasps::GraspCandidate(
            grasp, grasp_data_ptr_, Eigen::Affine3d()
        )
    ));

    if (!grasp_candidates_ptrs.size()) {
        ROS_WARN_STREAM("Generated 0 grasps");
        return false;
    }
    else {
        ROS_INFO_STREAM("Generated " << grasp_candidates_ptrs.size() << " grasps");
    }

    return true;
}

bool PickAndPlacePipeline::planGrasps(
    std::vector<moveit_grasps::GraspCandidatePtr>& grasp_candidates_ptrs, 
    moveit_grasps::GraspCandidatePtr& valid_grasp_candidate_ptr)
{
    moveit::core::RobotStatePtr pre_grasp_state_ptr;
    {
        planning_scene_monitor::LockedPlanningSceneRO 
        locked_planning_scene_ro_ptr(planning_scene_monitor_ptr_);
        pre_grasp_state_ptr.reset(new moveit::core::RobotState(
            locked_planning_scene_ro_ptr->getCurrentState())
        );
    }

    bool verbose_cartesian_filtering = true;
    bool success = false; 
    for (; !grasp_candidates_ptrs.empty(); grasp_candidates_ptrs.pop_back()) {
        valid_grasp_candidate_ptr = grasp_candidates_ptrs.front();
        valid_grasp_candidate_ptr->getPreGraspState(pre_grasp_state_ptr);

        if (!grasp_planner_ptr_->planApproachLiftRetreat(
            valid_grasp_candidate_ptr, 
            pre_grasp_state_ptr, 
            planning_scene_monitor_ptr_,
            verbose_cartesian_filtering)) {
            ROS_WARN_STREAM("Failed to plan approach lift retreat for " 
                << valid_grasp_candidate_ptr->grasp_.id);
            continue;
        }

        ROS_DEBUG_STREAM("Succeeded to plan approach lift retreat for "
            << valid_grasp_candidate_ptr->grasp_.id);
        success = true;
        break;
    } 

    return success;
}

moveit::planning_interface::MoveItErrorCode
PickAndPlacePipeline::planTargetState(
    moveit::core::RobotState& target_state,
    moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    const unsigned int num_planning_attempts = 5;// 5 times
    const double planning_time = 1.5;// 1.5s
    const double goal_tolerance = 0.01;// 0.01m

    move_group_ptr_->setNumPlanningAttempts(num_planning_attempts);
    move_group_ptr_->setPlanningTime(planning_time);
    move_group_ptr_->setGoalTolerance(goal_tolerance);

    move_group_ptr_->setStartStateToCurrentState();
    move_group_ptr_->setJointValueTarget(target_state);

    moveit::planning_interface::MoveItErrorCode error_code = 
        move_group_ptr_->plan(plan);
    
    return error_code;
}

moveit::planning_interface::MoveItErrorCode
PickAndPlacePipeline::planCartesianPath(
    std::vector<geometry_msgs::Pose>& waypoints,
    moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const double& eef_step,
    const double& jump_threshold)
{
    moveit::planning_interface::MoveItErrorCode error_code;
    moveit_msgs::RobotTrajectory trajectory;
    move_group_ptr_->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory, true, &error_code);

    moveit::core::RobotState current_state(
        *move_group_ptr_->getCurrentState());
    moveit_msgs::RobotState start_state;
    moveit::core::robotStateToRobotStateMsg(current_state, start_state);
    plan.trajectory_ = trajectory;
    plan.start_state_ = start_state;

    return error_code;
}

void PickAndPlacePipeline::visualizeGrasp(
    moveit_grasps::GraspCandidatePtr& valid_grasp_candidate_ptr,
    moveit::planning_interface::MoveGroupInterface::Plan& approach_plan,
    moveit::planning_interface::MoveGroupInterface::Plan& lift_plan,
    moveit::planning_interface::MoveGroupInterface::Plan& retreat_plan,
    moveit::planning_interface::MoveGroupInterface::Plan& pre_approach_plan,
    moveit::planning_interface::MoveGroupInterface::Plan& post_retreat_plan)
{
    //
    // get the pregrasp, grasp, lifted, and retreat waypoints
    //
    EigenSTL::vector_Affine3d waypoints;
    moveit_grasps::GraspGenerator::getGraspWaypoints(
        valid_grasp_candidate_ptr, waypoints);

    //
    // visualize waypoints
    //
    visual_tools_ptr_->publishAxisLabeled(waypoints[0], "pre-grasp");
    visual_tools_ptr_->publishAxisLabeled(waypoints[1], "grasp");
    visual_tools_ptr_->publishAxisLabeled(waypoints[2], "lifted");
    visual_tools_ptr_->publishAxisLabeled(waypoints[3], "retreat");
    visual_tools_ptr_->trigger();

    //
    // visualize the pregrasp, grasp, lifted, and retreat states
    //
    visual_tools_ptr_->prompt("Press NEXT to show pre-grasp state");
    visual_tools_ptr_->publishRobotState(
        valid_grasp_candidate_ptr->
            segmented_cartesian_traj_[moveit_grasps::APPROACH].front(),
        rviz_visual_tools::YELLOW
    );

    visual_tools_ptr_->prompt("Press NEXT to show grasp state");
    moveit::core::RobotStatePtr grasp_state_ptr(
        valid_grasp_candidate_ptr->
        segmented_cartesian_traj_[moveit_grasps::APPROACH].back());
    valid_grasp_candidate_ptr->getGraspStateClosedEEOnly(grasp_state_ptr);
    visual_tools_ptr_->publishRobotState(grasp_state_ptr, rviz_visual_tools::GREEN);

    visual_tools_ptr_->prompt("Press NEXT to show lifted state");
    visual_tools_ptr_->publishRobotState(
        valid_grasp_candidate_ptr->
            segmented_cartesian_traj_[moveit_grasps::LIFT].back(),
        rviz_visual_tools::GREEN
    );

    visual_tools_ptr_->prompt("Press NEXT to show retreat state");
    visual_tools_ptr_->publishRobotState(
        valid_grasp_candidate_ptr->
            segmented_cartesian_traj_[moveit_grasps::RETREAT].back(),
        rviz_visual_tools::YELLOW
    );

    visual_tools_ptr_->prompt("Press NEXT to show the motion planning");
    visual_tools_ptr_->trigger();

    //
    // visualize the motion planning - pre-approach, grasp, post-regreat
    //
    // moveit::core::RobotStatePtr pre_grasp_str(
    //     valid_grasp_candidate_ptr->
    //         segmented_cartesian_traj_[moveit_grasps::APPROACH].front()
    // );
    // valid_grasp_candidate_ptr->getGraspStateOpenEEOnly(pre_grasp_str);
    visual_tools_ptr_->publishTrajectoryPath(pre_approach_plan.trajectory_, 
        pre_approach_plan.start_state_, true);
    ros::Duration(0.5).sleep();

    visual_tools_ptr_->publishTrajectoryPath(approach_plan.trajectory_,
        approach_plan.start_state_, true);
    ros::Duration(0.05).sleep();
    visual_tools_ptr_->publishTrajectoryPath(lift_plan.trajectory_,
        lift_plan.start_state_, true);
    ros::Duration(0.05).sleep();
    visual_tools_ptr_->publishTrajectoryPath(retreat_plan.trajectory_,
        retreat_plan.start_state_, true);
    ros::Duration(1.5).sleep();

    visual_tools_ptr_->publishTrajectoryPath(post_retreat_plan.trajectory_, 
        post_retreat_plan.start_state_, true);
    ros::Duration(1.5).sleep();

    return;
}

bool PickAndPlacePipeline::getIKSolution(
    const moveit::core::JointModelGroup* group_ptr, 
    Eigen::Affine3d& target_pose, 
    moveit::core::RobotStatePtr& solution_state_ptr, 
    const std::string& tip
)
{
    //
    // get current state
    //
    {
        planning_scene_monitor::LockedPlanningSceneRO
        locked_planning_scene_ro_ptr(planning_scene_monitor_ptr_);
        solution_state_ptr.reset(new moveit::core::RobotState(
            locked_planning_scene_ro_ptr->getCurrentState())
        );
    }
    
    //
    // group state validity callback function
    //
    moveit::core::GroupStateValidityCallbackFn group_state_validity_callback = 
        boost::bind(&PickAndPlacePipeline::isStateValid, this, _1, _2, _3);

    //
    // solve the inverse kinematics problem
    //
    const std::size_t attempts = 5;
    const double timeout = 0.1;
    return solution_state_ptr->setFromIK(
        group_ptr, 
        target_pose, 
        tip, 
        attempts, 
        timeout,
        group_state_validity_callback
    );
}

moveit::core::RobotState PickAndPlacePipeline::getNamedTargetState(
    const std::string& target_name)
{
    moveit::core::RobotState robot_state(robot_model_ptr_);
    std::vector<std::string> joint_names = 
        arm_group_ptr_->getVariableNames();
    std::map<std::string, double> joint_position_map = 
        move_group_ptr_->getNamedTargetValues(target_name);
    std::vector<double> joint_positions;
    for (auto joint_name : joint_names) {
        joint_positions.push_back(joint_position_map[joint_name]);
    }
    robot_state.setJointGroupPositions(arm_group_ptr_, joint_positions);

    return robot_state;
}

bool PickAndPlacePipeline::isStateValid(
    moveit::core::RobotState* robot_state_ptr,
    const moveit::core::JointModelGroup* group_ptr,
    const double* ik_solution)
{
    robot_state_ptr->setJointGroupPositions(group_ptr, ik_solution);
    robot_state_ptr->update();

    planning_scene_monitor::LockedPlanningSceneRO 
    locked_planning_scene_ro_ptr(planning_scene_monitor_ptr_);
    return locked_planning_scene_ro_ptr->isStateValid(
        *robot_state_ptr, group_ptr->getName());
}

} // namespace pick_and_place_pipeline
