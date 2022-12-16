#include <ros/ros.h>
#include <ros/node_handle.h>
#include <std_srvs/Trigger.h>
#include <exception>
#include <string>
#include <vector>
#include <Eigen/Geometry>

#include <robot_motion_planning/PlanTrajectory.h>
#include <robot_motion_planning/planner_utils.h>

#include <tesseract_environment/environment.h>
#include <tesseract_monitoring/environment_monitor.h>

#include <eigen_conversions/eigen_msg.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_planning_server/tesseract_planning_server.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/interface_utils.h>
#include <tesseract_visualization/markers/toolpath_marker.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_geometry/mesh_parser.h>
#include <tesseract_rosutils/plotting.h>

#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_plan_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_no_ik_plan_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_plan_profile.h>

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>

#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>

#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>

static const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
static const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";
static const std::string MONITOR_NAMESPACE = "environment_monitor";
static const std::string MONITOR_ENVIRONMENT_NAMESPACE = "robot_environment";
static const std::string PLAN_PROCESS_SRV_NAME = "plan_process";
static const std::string SIMPLE_PLAN_PROCESS_SRV_NAME = "simple_plan_process";
static const std::string CSV_FILEPATH = "csv_filepath";

namespace robot_motion_planning
{
/** @brief Generate a program for the tesseract motion planner using a vector of waypoints*/
tesseract_planning::CompositeInstruction createProgram(const std::vector<Eigen::Isometry3d> toolpath,
                                                       const YAML::Node config_yaml)
{
    // Defining variables needed for creation of the composite instruction program
    // Define plan profile names
    std::string freespace_plan_profile = "FREESPACE";
    std::string raster_plan_profile = "RASTER";

    // Define starting and ending joint states in radians
    std::vector<double> starting_joint_state = config_yaml["starting_joint_state"].as<std::vector<double>>();
    std::vector<double> ending_joint_state = config_yaml["ending_joint_state"].as<std::vector<double>>();

    // Joint names of the robot, corresponds directly to joint states in terms of order
    std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };

    // Apply a set transform on all loaded points
    YAML::Node translation_yaml = getYaml<YAML::Node>(config_yaml, "translation");
    YAML::Node rotation_yaml = getYaml<YAML::Node>(config_yaml, "rotation");
    double x_translation = getYaml<double>(translation_yaml, 0);
    double y_translation = getYaml<double>(translation_yaml, 1);
    double z_translation = getYaml<double>(translation_yaml, 2);
    double x_rotation = getYaml<double>(rotation_yaml, 0);
    double y_rotation = getYaml<double>(rotation_yaml, 1);
    double z_rotation = getYaml<double>(rotation_yaml, 2);
    Eigen::Isometry3d wp_transform = Eigen::Isometry3d::Identity();
    wp_transform *= Eigen::AngleAxisd(x_rotation, Eigen::Vector3d::UnitX());
    wp_transform *= Eigen::AngleAxisd(y_rotation, Eigen::Vector3d::UnitY());
    wp_transform *= Eigen::AngleAxisd(z_rotation, Eigen::Vector3d::UnitZ());
    wp_transform.translation().x() = x_translation;
    wp_transform.translation().y() = y_translation;
    wp_transform.translation().z() = z_translation;

    // Create manipulator given manipulator group, tf frame of the waypoints, tcp frame
    tesseract_planning::ManipulatorInfo manip_info("manipulator", "world", "paint_gun");

    // Construction composite instruction (ci)
    tesseract_planning::CompositeInstruction ci("process_program", tesseract_planning::CompositeInstructionOrder::ORDERED,
                                                manip_info);

    // Adding information of waypoints to the ci
    // Define start instruction of ci (Where does the motion plan start)
    tesseract_planning::StateWaypoint swp1(joint_names, toVectorXd(starting_joint_state));
    tesseract_planning::PlanInstruction start_instruction(swp1, tesseract_planning::PlanInstructionType::START,
                                                          freespace_plan_profile);
    ci.setStartInstruction(start_instruction);

    // Create first freespace ci to first waypoint then add this to the overall ci
    // Define first waypoint by getting the first value from the toolpath
    tesseract_planning::CartesianWaypoint wp1 = wp_transform * toolpath.front();
    // Make a plan instruction of freespace type to this waypoint
    tesseract_planning::PlanInstruction plan_f0(wp1, tesseract_planning::PlanInstructionType::FREESPACE,
                                                freespace_plan_profile);
    plan_f0.setDescription("from_start_plan");  // This description can be set to anything, just an identifier
    // Create from_start composite because tesseract expects the from start to be a composite
    tesseract_planning::CompositeInstruction from_start;
    from_start.setDescription("from_start");  // This description can be set to anything, just an identifier
    from_start.push_back(plan_f0);  // Add the first waypoint freespace plan insruction to the from_start composite
    from_start.setProfile(freespace_plan_profile);  // Define profile name of the from start construction
    ci.push_back(from_start);                       // Add this newly created composite instruction to the overall ci

    // Create a raster ci by adding all the points sequentionally then add this to the overall ci
    tesseract_planning::CompositeInstruction raster_seg;
    raster_seg.setDescription("Raster");
    raster_seg.setProfile(raster_plan_profile);
    // Iterate over every point in the toolpath starting with the second point because the freespace plan went to the
    // first point already
    for (std::size_t i = 1; i < toolpath.size(); ++i)
    {
        tesseract_planning::CartesianWaypoint wp = wp_transform * toolpath[i];  // Apply transform to each waypoint
        raster_seg.push_back(
            tesseract_planning::PlanInstruction(wp, tesseract_planning::PlanInstructionType::LINEAR, raster_plan_profile));
    }
    ci.push_back(raster_seg);  // Add fully constructed raster segment CI to the overall ci

    // Create a final freespace ci back to the home joint state then add this to the overall ci
    tesseract_planning::StateWaypoint swpe(joint_names, toVectorXd(ending_joint_state));

    // Add to end instruction, freespace from end of raster to start position
    tesseract_planning::PlanInstruction retreat(swpe, tesseract_planning::PlanInstructionType::FREESPACE,
                                                freespace_plan_profile);
    retreat.setDescription("to_end_plan");
    tesseract_planning::CompositeInstruction to_end;
    to_end.setDescription("to_end");
    to_end.push_back(retreat);
    to_end.setProfile(freespace_plan_profile);
    ci.push_back(to_end);

    return ci;
}

void outputRasterData(const tesseract_planning::CompositeInstruction& ci,
                      const tesseract_scene_graph::StateSolver &state_solver,
                      const std::string tcp_frame = "paint_gun")
{
  // save joint states to a csv file
  std::ofstream outfile;
  outfile.open("/home/tcappellari/ros/rpi_ws/src/robot_motion_opt_tesseract/robot_motion_planning/task_data/tesseract_output/movej_waypoints.csv");

  auto joint_trajectory = tesseract_planning::toJointTrajectory(ci);
  outfile << "t,j1,j2,j3,j4,j5,j6" << std::endl;
  for (const auto& joint_state : joint_trajectory)
  {
    outfile << joint_state.time;
    for (Eigen::Index i=0; i < joint_state.position.size(); ++i)
    {
      outfile << "," << joint_state.position(i);
    }
    outfile << std::endl;
  }
  outfile.close();

  // save end effector positions to a csv file
  std::ofstream e_outfile;
  e_outfile.open("/home/tcappellari/ros/rpi_ws/src/robot_motion_opt_tesseract/robot_motion_planning/task_data/tesseract_output/end_waypoints.csv");
  e_outfile << "trans_x,trans_y,trans_z,rot_x,rot_y,rot_z," << std::endl;

  auto mv = tesseract_planning::flatten(ci, tesseract_planning::moveFilter);

  for (std::size_t i = 0; i < mv.size(); i++)
  {
      auto curr_wp = mv[i].get().as<tesseract_planning::MoveInstruction>().getWaypoint().as<tesseract_planning::StateWaypoint>();
      tesseract_scene_graph::SceneState ss_curr = state_solver.getState(curr_wp.joint_names, curr_wp.position);
      Eigen::Isometry3d curr_pose = ss_curr.link_transforms[tcp_frame];

      e_outfile << curr_pose.translation().x() << ",";
      e_outfile << curr_pose.translation().y() << ",";
      e_outfile << curr_pose.translation().z() << ",";
      auto rot = curr_pose.rotation().eulerAngles(0, 1, 2);
      e_outfile << rot.x() << ",";
      e_outfile << rot.y() << ",";
      e_outfile << rot.z() << ",";
      e_outfile << std::endl;
  }

  e_outfile.close();
}

class MotionPlanner
{
public:
    MotionPlanner(ros::NodeHandle& nh)
        : nh_(nh)
        , env_(std::make_shared<tesseract_environment::Environment>())
        , monitor_(std::make_shared<tesseract_monitoring::EnvironmentMonitor>(env_, MONITOR_NAMESPACE))
    {
        // Start tesseract environemnt monitor (RVIZ connection)
        monitor_->startMonitoringEnvironment(MONITOR_ENVIRONMENT_NAMESPACE);
        monitor_->setEnvironmentPublishingFrequency(1.0);

        // Start planning server service
        process_plan_server_ = nh_.advertiseService(PLAN_PROCESS_SRV_NAME, &MotionPlanner::planProcessCallback, this);
        simple_process_plan_server_ =
            nh_.advertiseService(SIMPLE_PLAN_PROCESS_SRV_NAME, &MotionPlanner::simplePlanProcessCallback, this);

        // Load in urdf
        std::string urdf_xml_string, srdf_xml_string;
        nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
        nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

        // load csv
        nh_.getParam(CSV_FILEPATH, csv_file);

        // Initialize tesseract environment
        tesseract_common::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
        if (!env_->init(urdf_xml_string, srdf_xml_string, locator))
        {
            ROS_ERROR_STREAM("Failed to initialize tesseract environment.");
        }
    }

    bool doMotionPlanning(YAML::Node config_yaml)
    {
        // Load in csv points to a vector of Eigen::Isometry3d
        YAML::Node waypoint_config = getYaml<YAML::Node>(config_yaml, "waypoints");
        int freq = getYaml<int>(waypoint_config, "downsample_frequency");

        std::vector<Eigen::Isometry3d> toolpath = loadPoints(csv_file, freq);
        ROS_INFO_STREAM("Loaded " << toolpath.size() << " points");

        // Create a tesseract composite instruction from the vector of waypoints
        tesseract_planning::CompositeInstruction ci = createProgram(toolpath, waypoint_config);

        // Save text file of the composite instruction to the /tmp directory for debugging purposes
        tesseract_planning::Serialization::toArchiveFileXML(ci, "/tmp/tesseract_ci_input.xml");

        // Create plotter object so that motion plans and toolpaths can be visualized in RVIZ
        tesseract_rosutils::ROSPlottingPtr plotter =
            std::make_shared<tesseract_rosutils::ROSPlotting>(env_->getSceneGraph()->getRoot());

        // Plot all the waypoints in RVIZ before planning begins
        plotter->waitForConnection();
        if (plotter->isConnected())
        {
            ros::Rate(3).sleep();
            plotter->clear();
            tesseract_common::Toolpath toolpath_ci = tesseract_planning::toToolpath(ci, *env_);
            plotter->plotMarker(tesseract_visualization::ToolpathMarker(toolpath_ci));
        }

        // Create planning server
        std::size_t planning_threads = 1;
        tesseract_planning::ProcessPlanningServer planning_server(
            std::make_shared<tesseract_planning::ProcessEnvironmentCache>(env_), planning_threads);

        // Update planning profiles
        planning_server.loadDefaultProcessPlanners();
        loadPlannerProfiles(planning_server, config_yaml);
        generateProcessTaskflow(planning_server, getYaml<YAML::Node>(config_yaml, "taskflow"));

        // Create tesseract request
        tesseract_planning::ProcessPlanningRequest tesseract_request;
        tesseract_request.instructions = ci;  // Populate request with generated composite instruction
        // Specify how tesseract should interpret the generated ci with the name parameters, see more at the following link
        // https://github.com/tesseract-robotics/tesseract_planning/blob/master/tesseract_process_managers/include/tesseract_process_managers/core/process_planning_request.h
        tesseract_request.name = "RobotProcessTaskflow";

        // Set logging level for what prints out to terminal
        console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);
        if (getYaml<bool>(config_yaml, "debug"))
            console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

        // Call tesseract planning server with generated composite instruction
        auto tesseract_response = planning_server.run(tesseract_request);
        tesseract_response.wait();

        // Get results from tesseract planning and check if successful
        if (!tesseract_response.interface->isSuccessful())
        {
            ROS_ERROR("Process planner failed");
            return false;
        }

        // Extract result as a CI from the tessesract response
        tesseract_planning::CompositeInstruction ci_res =
            (*tesseract_response.results).as<tesseract_planning::CompositeInstruction>();

        // Save text file of the composite instruction to the /tmp directory for debugging purposes
        tesseract_planning::Serialization::toArchiveFileXML(ci_res, "/tmp/tesseract_ci_output.xml");

        for (const auto& instr : ci_res)
        {
          ROS_INFO_STREAM("Instruction: " << instr.getDescription() << ", type: " << instr.getType().name());
        }
        auto state_solver = (*env_).getStateSolver();;
        outputRasterData(ci_res.at(1).as<tesseract_planning::CompositeInstruction>(), *state_solver); //The single raster

        // Visualize results in rviz
        plotter->waitForConnection();
        if (plotter->isConnected())
        {
            ros::Rate(3).sleep();
            plotter->clear();
            tesseract_common::Toolpath toolpath = tesseract_planning::toToolpath(ci_res, *env_);
            tesseract_common::JointTrajectory trajectory = tesseract_planning::toJointTrajectory(ci_res);

            plotter->plotMarker(tesseract_visualization::ToolpathMarker(toolpath));
            plotter->plotTrajectory(trajectory, *env_->getStateSolver());
        }
        return true;
    }

    bool planProcessCallback(std_srvs::Trigger::Request&,
                             std_srvs::Trigger::Response& res)
    {
        // Load planner config yaml file
        std::string support_path = ros::package::getPath("robot_motion_planning");
        std::string config_fp = support_path + "/config/planner_config.yaml";
        YAML::Node config_yaml = YAML::LoadFile(config_fp);

        bool success = doMotionPlanning(config_yaml);

        res.success = success;
        return true;
    }

    bool simplePlanProcessCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
    {
        // Load simple planner config yaml file
        std::string support_path = ros::package::getPath("robot_motion_planning");
        std::string simple_config_fp = support_path + "/config/simple_planner_config.yaml";
        YAML::Node simple_config_yaml = YAML::LoadFile(simple_config_fp);
        YAML::Node config_yaml = generatePlanConfigFromSimplePlanConfig(simple_config_yaml);

        std::ofstream fout("/tmp/generated_planner_config.yaml");
        fout << config_yaml;
        fout.close();

        bool success = doMotionPlanning(config_yaml);

        res.success = success;
        return true;
    }

private:
    void loadPlannerProfiles(tesseract_planning::ProcessPlanningServer& planning_server, YAML::Node& config_yaml)
    {
        // Load in config yaml
        YAML::Node simple_planner_config = getYaml<YAML::Node>(config_yaml, "simple_planner");
        YAML::Node descartes_config = getYaml<YAML::Node>(config_yaml, "descartes");
        YAML::Node ompl_config = getYaml<YAML::Node>(config_yaml, "ompl");
        YAML::Node trajopt_config = getYaml<YAML::Node>(config_yaml, "trajopt");

        // Get profiles dictionary from the planning server so that we can add custom profiles
        tesseract_planning::ProfileDictionary::Ptr profiles = planning_server.getProfiles();

        // Simple Planner (interpolation) Profile, see the following link for details
        // https://github.com/tesseract-robotics/tesseract_planning/blob/master/tesseract_motion_planners/
        // core/include/tesseract_motion_planners/simple/profile/simple_planner_lvs_plan_profile.h
        double longest_valid_state_length = getYaml<double>(simple_planner_config, "longest_valid_state_length");
        double longest_valid_translation = getYaml<double>(simple_planner_config, "longest_valid_translation");
        double longest_valid_rotation = getYaml<double>(simple_planner_config, "longest_valid_rotation");
        int min_steps = getYaml<int>(simple_planner_config, "min_steps");
        int max_steps = getYaml<int>(simple_planner_config, "max_steps");

        auto simple_plan_profile = std::make_shared<tesseract_planning::SimplePlannerLVSNoIKPlanProfile>(
            longest_valid_state_length, longest_valid_translation, longest_valid_rotation, min_steps, max_steps);

        profiles->addProfile<tesseract_planning::SimplePlannerPlanProfile>(
            tesseract_planning::profile_ns::SIMPLE_DEFAULT_NAMESPACE, "RASTER", simple_plan_profile);
        profiles->addProfile<tesseract_planning::SimplePlannerPlanProfile>(
            tesseract_planning::profile_ns::SIMPLE_DEFAULT_NAMESPACE, "FREESPACE", simple_plan_profile);

        // Descartes Plan Profile
        // https://github.com/tesseract-robotics/tesseract_planning/blob/master/tesseract_motion_planners/
        // descartes/include/tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h
        auto descartes_plan_profile = std::make_shared<tesseract_planning::DescartesDefaultPlanProfile<float>>();

        descartes_plan_profile->allow_collision = getYaml<bool>(descartes_config, "allow_collisions");
        double descartes_collsion_margin = getYaml<double>(descartes_config, "collision_margin");
        double sampling_angle = getYaml<double>(descartes_config, "sampling_angle");
        descartes_plan_profile->num_threads = getYaml<int>(descartes_config, "number_threads");

        if (descartes_plan_profile->num_threads > static_cast<int>(std::thread::hardware_concurrency()))
            descartes_plan_profile->num_threads = static_cast<int>(std::thread::hardware_concurrency());
        descartes_plan_profile->enable_collision = !descartes_plan_profile->allow_collision;
        descartes_plan_profile->vertex_collision_check_config.contact_manager_config.margin_data =
            tesseract_collision::CollisionMarginData(descartes_collsion_margin);
        descartes_plan_profile->target_pose_sampler = [sampling_angle](const Eigen::Isometry3d& tool_pose) {
            return tesseract_planning::sampleToolZAxis(tool_pose, sampling_angle);
        };

        profiles->addProfile<tesseract_planning::DescartesPlanProfile<float>>(
            tesseract_planning::profile_ns::DESCARTES_DEFAULT_NAMESPACE, "RASTER", descartes_plan_profile);
        profiles->addProfile<tesseract_planning::DescartesPlanProfile<float>>(
            tesseract_planning::profile_ns::DESCARTES_DEFAULT_NAMESPACE, "FREESPACE", descartes_plan_profile);

        // OMPL Plan Profile, see the following link for details
        // https://github.com/tesseract-robotics/tesseract_planning/blob/master/tesseract_motion_planners/
        // ompl/include/tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h
        auto ompl_plan_profile = std::make_shared<tesseract_planning::OMPLDefaultPlanProfile>();
        auto rrt_plan = std::make_shared<tesseract_planning::RRTConnectConfigurator>();

        rrt_plan->range = getYaml<double>(ompl_config, "range");
        std::size_t num_solvers = getYaml<std::size_t>(ompl_config, "num_solvers");
        double ompl_collision_margin = getYaml<double>(ompl_config, "collision_margin");
        ompl_plan_profile->max_solutions = getYaml<int>(ompl_config, "max_solutions");
        ompl_plan_profile->planning_time = getYaml<double>(ompl_config, "max_allowed_planning_time");
        ompl_plan_profile->simplify = getYaml<bool>(ompl_config, "simplify");

        ompl_plan_profile->planners.clear();
        for (std::size_t i = 0; i < num_solvers; i++)
            ompl_plan_profile->planners.push_back(rrt_plan);
        ompl_plan_profile->collision_check_config.contact_manager_config.margin_data.setDefaultCollisionMargin(
            ompl_collision_margin);

        profiles->addProfile<tesseract_planning::OMPLPlanProfile>(tesseract_planning::profile_ns::OMPL_DEFAULT_NAMESPACE,
                                                                  "RASTER", ompl_plan_profile);
        profiles->addProfile<tesseract_planning::OMPLPlanProfile>(tesseract_planning::profile_ns::OMPL_DEFAULT_NAMESPACE,
                                                                  "FREESPACE", ompl_plan_profile);

        // Trajopt Composite Profile, see the following link for details
        // https://github.com/tesseract-robotics/tesseract_planning/blob/master/tesseract_motion_planners/
        // trajopt/include/tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h
        auto trajopt_composite_profile = std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();
        YAML::Node trajopt_collision_config = trajopt_config["collision"];
        YAML::Node trajopt_velocity_config = trajopt_config["smooth_velocities"];
        YAML::Node trajopt_acceleration_config = trajopt_config["smooth_accelerations"];
        YAML::Node trajopt_jerk_config = trajopt_config["smooth_jerks"];

        trajopt_composite_profile->longest_valid_segment_length = getYaml<double>(trajopt_config, "longest_valid_segment_"
                                                                                                  "length");
        trajopt_composite_profile->longest_valid_segment_fraction = getYaml<double>(trajopt_config, "longest_valid_segment_"
                                                                                                    "fraction");
        bool collision_enabled = getYaml<bool>(trajopt_collision_config, "enabled");
        bool collision_constraint = getYaml<bool>(trajopt_collision_config, "constraint");
        trajopt_composite_profile->collision_cost_config.safety_margin = getYaml<double>(trajopt_collision_config, "margi"
                                                                                                                   "n");
        trajopt_composite_profile->collision_cost_config.safety_margin_buffer =
            getYaml<double>(trajopt_collision_config, "margin_buffer");
        trajopt_composite_profile->collision_cost_config.coeff = getYaml<double>(trajopt_collision_config, "coeff");
        trajopt_composite_profile->collision_cost_config.use_weighted_sum = getYaml<bool>(trajopt_collision_config, "weight"
                                                                                                                    "ed_"
                                                                                                                    "sum");
        trajopt_composite_profile->collision_constraint_config.safety_margin =
            getYaml<double>(trajopt_collision_config, "margin");
        trajopt_composite_profile->collision_constraint_config.safety_margin_buffer =
            getYaml<double>(trajopt_collision_config, "margin_buffer");
        trajopt_composite_profile->collision_constraint_config.coeff = getYaml<double>(trajopt_collision_config, "coeff");
        trajopt_composite_profile->collision_constraint_config.use_weighted_sum =
            getYaml<bool>(trajopt_collision_config, "weighted_sum");
        trajopt_composite_profile->smooth_velocities = getYaml<bool>(trajopt_velocity_config, "enabled");
        std::vector<double> velocity_coeff = getYaml<std::vector<double>>(trajopt_velocity_config, "coeff");
        trajopt_composite_profile->smooth_accelerations = getYaml<bool>(trajopt_acceleration_config, "enabled");
        std::vector<double> accceleration_coeff = getYaml<std::vector<double>>(trajopt_acceleration_config, "coeff");
        trajopt_composite_profile->smooth_jerks = getYaml<bool>(trajopt_jerk_config, "enabled");
        std::vector<double> jerk_coeff = getYaml<std::vector<double>>(trajopt_jerk_config, "coeff");

        trajopt_composite_profile->collision_cost_config.enabled = !collision_constraint && collision_enabled;
        trajopt_composite_profile->collision_constraint_config.enabled = collision_constraint && collision_enabled;
        trajopt_composite_profile->velocity_coeff = toVectorXd(velocity_coeff);
        trajopt_composite_profile->acceleration_coeff = toVectorXd(velocity_coeff);
        trajopt_composite_profile->jerk_coeff = toVectorXd(velocity_coeff);
        trajopt_composite_profile->contact_test_type = tesseract_collision::ContactTestType::ALL;

        profiles->addProfile<tesseract_planning::TrajOptCompositeProfile>(
            tesseract_planning::profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "RASTER", trajopt_composite_profile);
        profiles->addProfile<tesseract_planning::TrajOptCompositeProfile>(
            tesseract_planning::profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "FREESPACE", trajopt_composite_profile);

        // Trajopt Plan Profile, see the following link for details
        // https://github.com/tesseract-robotics/tesseract_planning/blob/master/tesseract_motion_planners/
        // trajopt/include/tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h
        auto trajopt_plan_profile = std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();
        YAML::Node trajopt_waypoint_config = trajopt_config["waypoint"];

        bool waypoint_constraint = getYaml<bool>(trajopt_waypoint_config, "constraint");
        std::vector<double> waypoint_coeff = getYaml<std::vector<double>>(trajopt_waypoint_config, "coeff");

        trajopt_plan_profile->cartesian_coeff = toVectorXd(waypoint_coeff);
        if (waypoint_constraint)
            trajopt_plan_profile->term_type = trajopt::TermType::TT_CNT;
        else
            trajopt_plan_profile->term_type = trajopt::TermType::TT_COST;

        profiles->addProfile<tesseract_planning::TrajOptPlanProfile>(
            tesseract_planning::profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "RASTER", trajopt_plan_profile);
        profiles->addProfile<tesseract_planning::TrajOptPlanProfile>(
            tesseract_planning::profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "FREESPACE", trajopt_plan_profile);
    }

    ros::NodeHandle nh_;

    // ROS Service Server for triggering motion planning
    ros::ServiceServer process_plan_server_;
    ros::ServiceServer simple_process_plan_server_;

    // Tesseract required components for keeping up with and publishing the environment state
    tesseract_environment::Environment::Ptr env_;
    tesseract_monitoring::EnvironmentMonitor::Ptr monitor_;

    // csv file
    std::string csv_file;
};
}  // namespace robot_motion_planning

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;
    robot_motion_planning::MotionPlanner motion_planner(nh);

    ros::spin();
}
