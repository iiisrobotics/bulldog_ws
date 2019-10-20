#ifndef PICK_AND_PLACE_PIPELINE_H
#define PICK_AND_PLACE_PIPELINE_H

//---------- movieit
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_grasps/grasp_data.h>
#include <moveit_grasps/grasp_planner.h>
//----------

//---------- robotiq 3 finger gripper commander
#include <robotiq_3f_gripper_commander/robotiq_3f_gripper_commander.h>
//----------

namespace pick_and_place_pipeline
{

class PickAndPlacePipeline final
{

private:
    /**
     *  @attrib node_: ros node
     */
    ros::NodeHandle node_;

    /**
     *  @attrib visual_tools_ptr_: visualization tools
     */
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_ptr_;

    /**
     *  @attrib grasp_data_ptr_: gripper configuration
     */
    moveit_grasps::GraspDataPtr grasp_data_ptr_;

    /**
     *  @attrib grasp_filter_ptr_: grasp filter
     */
    moveit_grasps::GraspFilterPtr grasp_filter_ptr_;

    /**
     *  @attrib grasp_planner_: grasp planner
     */
    moveit_grasps::GraspPlannerPtr grasp_planner_ptr_;

    /**
     *  @attrib planning_scene_monitor_ptr_: planning scene monitor
     */
    planning_scene_monitor::PlanningSceneMonitorPtr 
        planning_scene_monitor_ptr_;

    /**
     *  @attrib move_group_ptr_: move group interface
     */
    moveit::planning_interface::MoveGroupInterfacePtr move_group_ptr_;

    /**
     *  @attrib planning_pipeline_ptr_: motion planning
     */
    planning_pipeline::PlanningPipelinePtr planning_pipeline_ptr_;

    /**
     *  @attrib clear_octomap_client_ptr_: ros seriver client to clear octomap
     */
    ros::ServiceClientPtr clear_octomap_client_ptr_;

    /**
     *  @attrib end_effector_name_: name of the end effector group
     */
    std::string end_effector_group_name_;

    /**
     *  @attrib arm_group_name_: name of the arm group
     */
    std::string arm_group_name_;

    /**
     *  @attrib gripper_command_topic_: topic to send the gripper control
     *                                  command
     */
    std::string gripper_command_topic_;

    /**
     *  @attrib gripper_status_topic_: topic to receive the gripper status
     */
    std::string gripper_status_topic_;

    /**
     *  @attrib clear_octomap_service_: service to clear the octomap
     */
    std::string clear_octomap_service_;

    /**
     *  @attrib arm_group_ptr_: arm group
     */
    const robot_model::JointModelGroup* arm_group_ptr_;

    /**
     *  @attrib robot_model_ptr_: robot model
     */
    robot_model::RobotModelConstPtr robot_model_ptr_;

    /**
     *  @attrib gripper_commander_: Robotiq 3 finger gripper commander
     */
    robotiq_3f_gripper_commander::Robotiq3FGripperCommanderPtr 
        gripper_commander_ptr_;

protected:
public:

private:
    /**
     *  @brief  generateGrasps: generate grasp candidates with respect to
     *                          target pose
     *  @param  grasp_poses: the target grasping poses
     *  @param  grasp_candidate_ptrs: the generated grasp candidates
     *  @return success: true on success
     */
    bool generateGrasps(
        std::vector<geometry_msgs::PoseStamped>& grasp_poses,
        std::vector<moveit_grasps::GraspCandidatePtr>& grasp_candidate_ptrs);

    /**
     *  @brief  generateSeedStates: generate seed state for inverse kinematics
     *                              filtering of each grasp candidate
     *  @param  grasp_candidate_ptrs: grasp candidates
     *  @param  seed_state_ptrs: seed state of each grasp candidates
     *  @return success: true on success
     */
    bool generateSeedStates(
        std::vector<moveit_grasps::GraspCandidatePtr>& grasp_candidate_ptrs,
        std::vector<moveit::core::RobotStatePtr>& seed_state_ptrs);

    /**
     *  @brief  filterGrasps: filter grasp candidates
     *  @param  grasp_candidate_ptrs: grasp candidates
     *  @param  seed_state_ptrs: seed state of each grasp candidates
     *  @param  filter_pregrasps: filter pre-grasp poses or not
     *  @return success: true on success
     */
    bool filterGrasps(
        std::vector<moveit_grasps::GraspCandidatePtr>& grasp_candidate_ptrs,
        std::vector<moveit::core::RobotStatePtr>& seed_state_ptrs,
        bool filter_pregrasps = true);

    /**
     *  @brief  planGrasps: plan the trajectories of the grasp candidates
     *  @param  grasp_candidate_ptrs: the grasp candidates
     *  @param  valid_grasp_candidate_ptr: the valid grasp candidate
     *  @return success: true on success
     */
    bool planGrasps(
        std::vector<moveit_grasps::GraspCandidatePtr>& grasp_candidate_ptrs,
        moveit_grasps::GraspCandidatePtr& vaild_grasp_candidate_ptr);

    /**
     *  @brief  planTargetState: plan the trajectory to target state
     *  @param  target_state: the terminated state of the trajectory
     *  @param  plan: the trajectory motion plan
     *  @return error_code: error code of motion planning given by the move
     *                      group interface
     */
    moveit::planning_interface::MoveItErrorCode planTargetState(
        moveit::core::RobotState& target_state,
        moveit::planning_interface::MoveGroupInterface::Plan& plan);

    /**
     *  @brief  planCartesianPath: plan a Cartesian path along the waypoints
     *  @param  waypoints: waypoints on the path
     *  @param  eef_step: the max step between two points on the path
     *  @param  jump_threshold: No more than jump_threshold is allowed as
     *                          change in distance in the configuration space
     *                          of the robot
     */
    moveit::planning_interface::MoveItErrorCode planCartesianPath(
        std::vector<geometry_msgs::Pose>& waypoints,
        moveit::planning_interface::MoveGroupInterface::Plan& plan,
        const double& eef_step = 0.01,
        const double& jump_threshold = 4.0);

    /**
     *  @brief  visualizeGrasp: visualize the grasping pose by moveit visual
     *                          tools
     *  @param  valid_grasp_candidate_ptr: the valid grasp candidate
     *  @param  approach_plan: the approach trajectory motion plan
     *  @param  pre_approach_plan: the pre-approach trajectory motion plan
     *  @param  post_retreat_plan: the post-retreat trajectory motion plan
     */
    void visualizeGrasp(
        moveit_grasps::GraspCandidatePtr& valid_grasp_candidate_ptr,
        moveit::planning_interface::MoveGroupInterface::Plan& approach_plan,
        moveit::planning_interface::MoveGroupInterface::Plan& lift_plan,
        moveit::planning_interface::MoveGroupInterface::Plan& retreat_plan,
        moveit::planning_interface::MoveGroupInterface::Plan& pre_approach_plan,
        moveit::planning_interface::MoveGroupInterface::Plan& post_retreat_plan);

    /**
     *  @brief  getIKSolution: get an inverse kinematics solution from the
     *                         specified joint model group to target pose
     *  @param  group_ptr: the specified group to manipulate
     *  @param  target_pose: the target pose of the last link of the joint
     *                       model group
     *  @param  solution_state_ptr: solution robot state
     *  @param  tip: the name of the frame for which IK is attempted
     *  @return success: true on success
     */
    bool getIKSolution(
        const moveit::core::JointModelGroup* group_ptr,
        Eigen::Affine3d& target_pose, 
        robot_state::RobotStatePtr& solution_state_ptr, 
        const std::string& tip);

    /**
     *  @brief  getNamedTargetState: get the robot state of a named target pose
     *  @param  target_name: name of the target pose
     *  @return robot_state: robot state of the target pose
     */
    moveit::core::RobotState getNamedTargetState(
        const std::string& target_name);

    /**
     *  @brief  isStateValid: justify whether the IK solution gives a valid
     *                        robot state
     *  @param  planning_scene: the locked read only planning scene
     *  @param  visual_tools: visualization tools
     *  @param  robot_state_ptr: current robot state
     *  @param  group: joint model group to manipulate
     *  @param  ik_solution: inverse kinematics solution
     *  @return success: true on success
     */
    bool isStateValid(
        robot_state::RobotState* robot_state_ptr,
        const moveit::core::JointModelGroup* group_ptr,
        const double* ik_solution);

protected:
public:
    /**
     *  @brief  PickAndPlacePipeline: default constructor
     *  @param  node: ros node of this pipeline
     */
    PickAndPlacePipeline(ros::NodeHandle node = ros::NodeHandle());

    /**
     *  @brief  PickAndPlacePipeline: copy constructor (deprecated)
     *  @param  pick_and_place_pipeline: a <PickAndPlacePipeline> instantiation
     *                                   to copy
     */
    PickAndPlacePipeline(
        const PickAndPlacePipeline &pick_and_place_pipeline) = delete;

    /**
     *  @brief  PickAndPlacePipeline: destructor
     */
    ~PickAndPlacePipeline();

    /**
     *  @brief  operator=: assignment operator (deprecated)
     *  @param  pick_and_place_pipeline: a <PickAndPlacePipeline> instantiation
     *                                   to copy
     *  @return *this: this instantiation
     */
    PickAndPlacePipeline &operator=(
        const PickAndPlacePipeline& pick_and_place_pipeline) = delete;

    /**
     *  @brief  run: run the pick and place pipeline
     *  @param  grasp_poses: all possible grasp poses
     *  @return success: true on success
     */
    bool run(std::vector<geometry_msgs::PoseStamped>& grasp_poses);

};

} // namespace pick_and_place_pipeline

#endif // PICK_AND_PLACE_PIPELINE_H
