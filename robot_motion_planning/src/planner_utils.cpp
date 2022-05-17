#include <robot_motion_planning/planner_utils.h>

std::vector<Eigen::Isometry3d> robot_motion_planning::loadPoints(const std::string& filepath, int& freq)
{
    // Initialize points vector
    std::vector<Eigen::Isometry3d> points;

    // Load csv
    std::ifstream file_(filepath);

    std::string line;
    std::string separator = ",";
    int rowIdx = 0;

    // Iterate over each line creating an Eigen::Isometry3d, assume each waypoint has no rotation relative to the part
    // frame
    while (std::getline(file_, line))
    {
        Eigen::Isometry3d point = Eigen::Isometry3d::Identity();
        if ((rowIdx % freq) == 0)
        {
            std::vector<std::string> tokens;
            boost::split(tokens, line, boost::is_any_of(separator));
            if (tokens.size() == 6)
            {
                Eigen::Quaterniond rot;
                rot = Eigen::AngleAxisd(std::stod(tokens[3]), Eigen::Vector3d::UnitX())
                      * Eigen::AngleAxisd(std::stod(tokens[4]), Eigen::Vector3d::UnitY())
                      * Eigen::AngleAxisd(std::stod(tokens[5]), Eigen::Vector3d::UnitZ());
                point = rot;
                point.translation().x() = std::stod(tokens[0]) / 1000;
                point.translation().y() = std::stod(tokens[1]) / 1000;
                point.translation().z() = std::stod(tokens[2]) / 1000;
                points.push_back(point);
            }
        }
        rowIdx++;
    }

    // Add new point to points
    return points;
}

YAML::Node robot_motion_planning::generatePlanConfigFromSimplePlanConfig(const YAML::Node& simple_plan_config)
{
    // Create plan config that will be returned
    YAML::Node plan_config;

    // Get two relevant nodes from the simple config file
    YAML::Node simple_waypoints_config = getYaml<YAML::Node>(simple_plan_config, "waypoints");
    YAML::Node collision_waypoints_config = getYaml<YAML::Node>(simple_plan_config, "collision");

    // Define variables that are reused
    std::vector<double> coeff_vector(6, 1);
    bool smooth_transitions = getYaml<bool>(simple_waypoints_config, "smooth_transitions");
    bool fixed_orientation = getYaml<bool>(simple_waypoints_config, "fixed_orientation");
    bool collision_enabled = getYaml<bool>(collision_waypoints_config, "enabled");
    double collision_margin = getYaml<double>(collision_waypoints_config, "margin");

    // Construct waypoint config for main plan config
    YAML::Node waypoints_config;
    waypoints_config["downsample_frequency"] = getYaml<YAML::Node>(simple_waypoints_config, "downsample_frequency");
    waypoints_config["translation"] = getYaml<YAML::Node>(simple_waypoints_config, "translation");
    waypoints_config["rotation"] = getYaml<YAML::Node>(simple_waypoints_config, "rotation");
    waypoints_config["starting_joint_state"] = getYaml<YAML::Node>(simple_waypoints_config, "starting_joint_state");
    waypoints_config["ending_joint_state"] = getYaml<YAML::Node>(simple_waypoints_config, "ending_joint_state");

    // Construct waypoint config for main plan config
    YAML::Node simple_planner_config;
    simple_planner_config["longest_valid_state_length"] = 0.1;
    simple_planner_config["longest_valid_translation"] = 0.1;
    simple_planner_config["longest_valid_rotation"] = 0.1;
    simple_planner_config["min_steps"] = 1;
    simple_planner_config["max_steps"] = 1;

    // Construct descartes config for main plan config
    YAML::Node descartes_config;
    descartes_config["allow_collisions"] = !collision_enabled;
    descartes_config["collision_margin"] = collision_margin;
    descartes_config["fixed_sampler"] = fixed_orientation;
    descartes_config["sampling_angle"] = getYaml<YAML::Node>(simple_waypoints_config, "sampling_angle");
    descartes_config["number_threads"] = static_cast<int>(std::thread::hardware_concurrency());

    // Construct ompl config for main plan config
    YAML::Node ompl_config;
    ompl_config["range"] = 0.1;
    ompl_config["num_solvers"] = 3;
    ompl_config["max_solutions"] = 1;
    ompl_config["max_allowed_planning_time"] = 30.0;
    if (collision_enabled)
        ompl_config["collision_margin"] = collision_margin;
    else
        ompl_config["collision_margin"] = -1.0;
    ompl_config["simplify"] = false;

    // Construct trajopt config for main plan config
    YAML::Node trajopt_config;
    trajopt_config["longest_valid_segment_length"] = 0.05;
    trajopt_config["longest_valid_segment_fraction"] = 0.01;
    trajopt_config["collision"]["enabled"] = collision_enabled;
    trajopt_config["collision"]["constraint"] = true;
    trajopt_config["collision"]["margin"] = collision_margin;
    trajopt_config["collision"]["margin_buffer"] = collision_margin + 0.02;
    trajopt_config["collision"]["coeff"] = 2;
    trajopt_config["collision"]["weighted_sum"] = true;
    trajopt_config["smooth_velocities"]["enabled"] = smooth_transitions;
    trajopt_config["smooth_velocities"]["coeff"] = coeff_vector;
    trajopt_config["smooth_accelerations"]["enabled"] = smooth_transitions;
    trajopt_config["smooth_accelerations"]["coeff"] = coeff_vector;
    trajopt_config["smooth_jerks"]["enabled"] = smooth_transitions;
    trajopt_config["smooth_jerks"]["coeff"] = coeff_vector;
    trajopt_config["waypoint"]["constraint"] = false;
    std::vector<double> waypoint_coeff_vector(6, 5);
    if (!fixed_orientation)
        waypoint_coeff_vector[5] = 0;
    trajopt_config["waypoint"]["coeff"] = waypoint_coeff_vector;

    // Add all yaml nodes to main config yaml
    plan_config["waypoints"] = waypoints_config;
    plan_config["simple_planner"] = simple_planner_config;
    plan_config["descartes"] = descartes_config;
    plan_config["ompl"] = ompl_config;
    plan_config["trajopt"] = trajopt_config;
    plan_config["taskflow"] = getYaml<YAML::Node>(simple_plan_config, "taskflow");
    plan_config["debug"] = false;

    return plan_config;
}

tesseract_planning::TaskflowGenerator::UPtr
robot_motion_planning::createFreespaceGenerator(const YAML::Node& config_yaml)
{
    using namespace tesseract_planning;

    // Create all the bools to know what processes are used
    bool use_ompl = getYaml<bool>(config_yaml, "ompl");
    bool use_trajopt = getYaml<bool>(config_yaml, "trajopt");
    bool use_contact_check = getYaml<bool>(config_yaml, "contact_check");
    bool use_time_parameterization = getYaml<bool>(config_yaml, "time_parameterization");

    // Initialize taskflow that will be returned
    auto tf = std::make_unique<GraphTaskflow>("RobotFreespaceTaskflow");

    // Check that the input is good
    int check_input_task = tf->addNode(std::make_unique<CheckInputTaskGenerator>(), true);

    // See if input has a seed
    int has_seed_task = tf->addNode(std::make_unique<HasSeedTaskGenerator>(), true);

    // Perform joint interpolation on input
    auto interpolator = std::make_shared<SimpleMotionPlanner>();
    int interpolator_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(interpolator), true);
    ;

    // Check that a minimum length is achieved for planning
    int seed_min_length_task = tf->addNode(std::make_unique<SeedMinLengthTaskGenerator>());

    // OMPL planner which can solve freespace motions
    auto ompl_planner = std::make_shared<OMPLMotionPlanner>();
    int ompl_task;
    if (use_ompl)
        ompl_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(ompl_planner), true);

    // TrajOpt planner which can solve freespace motions or optimize existing seeds
    auto trajopt_planner = std::make_shared<TrajOptMotionPlanner>();
    int trajopt_task;
    if (use_trajopt)
        trajopt_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(trajopt_planner), true);

    // Perform a contact check at and inbetween individual joint states
    int contact_check_task;
    if (use_contact_check)
        contact_check_task = tf->addNode(std::make_unique<DiscreteContactCheckTaskGenerator>(), true);

    // Perform time parameterization to assign timestamps, velocities, and accelerations
    int time_parameterization_task;
    if (use_time_parameterization)
        time_parameterization_task = tf->addNode(std::make_unique<IterativeSplineParameterizationTaskGenerator>(), true);

    // Make all connections based on what is turned on
    tf->addEdges(check_input_task, { tesseract_planning::GraphTaskflow::ERROR_NODE, has_seed_task });

    tf->addEdges(has_seed_task, { interpolator_task, seed_min_length_task });
    tf->addEdges(interpolator_task, { tesseract_planning::GraphTaskflow::ERROR_NODE, seed_min_length_task });

    if (use_ompl)
        tf->addEdges(seed_min_length_task, { ompl_task });
    else if (use_trajopt)
        tf->addEdges(seed_min_length_task, { trajopt_task });
    else if (use_contact_check)
        tf->addEdges(seed_min_length_task, { contact_check_task });
    else if (use_time_parameterization)
        tf->addEdges(seed_min_length_task, { time_parameterization_task });
    else
        tf->addEdges(seed_min_length_task, { GraphTaskflow::DONE_NODE });

    if (use_ompl && use_trajopt)
        tf->addEdges(ompl_task, { GraphTaskflow::ERROR_NODE, trajopt_task });
    else if (use_ompl && use_contact_check)
        tf->addEdges(ompl_task, { GraphTaskflow::ERROR_NODE, contact_check_task });
    else if (use_ompl && use_time_parameterization)
        tf->addEdges(ompl_task, { GraphTaskflow::ERROR_NODE, time_parameterization_task });
    else if (use_ompl)
        tf->addEdges(ompl_task, { GraphTaskflow::ERROR_NODE, GraphTaskflow::DONE_NODE });

    if (use_trajopt && use_contact_check)
        tf->addEdges(trajopt_task, { GraphTaskflow::ERROR_NODE, contact_check_task });
    else if (use_trajopt && use_time_parameterization)
        tf->addEdges(trajopt_task, { GraphTaskflow::ERROR_NODE, time_parameterization_task });
    else if (use_trajopt)
        tf->addEdges(trajopt_task, { GraphTaskflow::ERROR_NODE, GraphTaskflow::DONE_NODE });

    if (use_contact_check && use_time_parameterization)
        tf->addEdges(contact_check_task, { GraphTaskflow::ERROR_NODE, time_parameterization_task });
    if (use_contact_check)
        tf->addEdges(contact_check_task, { GraphTaskflow::ERROR_NODE, GraphTaskflow::DONE_NODE });

    if (use_time_parameterization)
        tf->addEdges(time_parameterization_task, { GraphTaskflow::ERROR_NODE, GraphTaskflow::DONE_NODE });

    return tf;
}

tesseract_planning::TaskflowGenerator::UPtr
robot_motion_planning::createRasterGenerator(const YAML::Node& config_yaml)
{
    using namespace tesseract_planning;

    // Create all the bools to know what processes are used
    bool use_descartes = getYaml<bool>(config_yaml, "descartes");
    bool use_trajopt = getYaml<bool>(config_yaml, "trajopt");
    bool use_contact_check = getYaml<bool>(config_yaml, "contact_check");
    bool use_time_parameterization = getYaml<bool>(config_yaml, "time_parameterization");

    // Initialize taskflow that will be returned
    auto tf = std::make_unique<GraphTaskflow>("RobotRasterTaskflow");

    // Check that the input is good
    int check_input_task = tf->addNode(std::make_unique<CheckInputTaskGenerator>(), true);

    // See if input has a seed
    int has_seed_task = tf->addNode(std::make_unique<HasSeedTaskGenerator>(), true);

    // Perform Cartesian interpolation on input
    auto interpolator = std::make_shared<SimpleMotionPlanner>();
    int interpolator_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(interpolator), true);
    ;

    // Check that a minimum length is achieved for planning
    int seed_min_length_task = tf->addNode(std::make_unique<SeedMinLengthTaskGenerator>());

    // Descartes planner which can find an optimal path through a series of waypoints
    auto descartes_planner = std::make_shared<DescartesMotionPlannerF>();
    int descartes_task;
    if (use_descartes)
        descartes_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(descartes_planner), true);

    // TrajOpt planner which can solve freespace motions or optimize existing seeds
    auto trajopt_planner = std::make_shared<TrajOptMotionPlanner>();
    int trajopt_task;
    if (use_trajopt)
        trajopt_task = tf->addNode(std::make_unique<MotionPlannerTaskGenerator>(trajopt_planner), true);

    // Perform a contact check at and inbetween individual joint states
    int contact_check_task;
    if (use_contact_check)
        contact_check_task = tf->addNode(std::make_unique<DiscreteContactCheckTaskGenerator>(), true);

    // Perform time parameterization to assign timestamps, velocities, and accelerations
    int time_parameterization_task;
    if (use_time_parameterization)
        time_parameterization_task = tf->addNode(std::make_unique<IterativeSplineParameterizationTaskGenerator>(), true);

    // Make all connections based on what is turned on
    tf->addEdges(check_input_task, { tesseract_planning::GraphTaskflow::ERROR_NODE, has_seed_task });

    tf->addEdges(has_seed_task, { interpolator_task, seed_min_length_task });
    tf->addEdges(interpolator_task, { tesseract_planning::GraphTaskflow::ERROR_NODE, seed_min_length_task });

    if (use_descartes)
        tf->addEdges(seed_min_length_task, { descartes_task });
    else if (use_trajopt)
        tf->addEdges(seed_min_length_task, { trajopt_task });
    else
        throw(std::runtime_error("Must have either trajopt or descartes turned on"));

    if (use_descartes && use_trajopt)
        tf->addEdges(descartes_task, { GraphTaskflow::ERROR_NODE, trajopt_task });
    else if (use_descartes && use_contact_check)
        tf->addEdges(descartes_task, { GraphTaskflow::ERROR_NODE, contact_check_task });
    else if (use_descartes && use_time_parameterization)
        tf->addEdges(descartes_task, { GraphTaskflow::ERROR_NODE, time_parameterization_task });
    else if (use_descartes)
        tf->addEdges(descartes_task, { GraphTaskflow::ERROR_NODE, GraphTaskflow::DONE_NODE });

    if (use_trajopt && use_contact_check)
        tf->addEdges(trajopt_task, { GraphTaskflow::ERROR_NODE, contact_check_task });
    else if (use_trajopt && use_time_parameterization)
        tf->addEdges(trajopt_task, { GraphTaskflow::ERROR_NODE, time_parameterization_task });
    else if (use_trajopt)
        tf->addEdges(trajopt_task, { GraphTaskflow::ERROR_NODE, GraphTaskflow::DONE_NODE });

    if (use_contact_check && use_time_parameterization)
        tf->addEdges(contact_check_task, { GraphTaskflow::ERROR_NODE, time_parameterization_task });
    if (use_contact_check)
        tf->addEdges(contact_check_task, { GraphTaskflow::ERROR_NODE, GraphTaskflow::DONE_NODE });

    if (use_time_parameterization)
        tf->addEdges(time_parameterization_task, { GraphTaskflow::ERROR_NODE, GraphTaskflow::DONE_NODE });

    return tf;
}

void robot_motion_planning::generateProcessTaskflow(tesseract_planning::ProcessPlanningServer& planning_server,
                                                         const YAML::Node& config_yaml)
{
    using namespace tesseract_planning;

    // Create freespace taskflows for freespace moves and transitions
    TaskflowGenerator::UPtr freespace_task = createFreespaceGenerator(getYaml<YAML::Node>(config_yaml, "freespace"));
    TaskflowGenerator::UPtr transition_task = createFreespaceGenerator(getYaml<YAML::Node>(config_yaml, "freespace"));

    // Create raster taskflow for raster moves
    TaskflowGenerator::UPtr raster_task = createRasterGenerator(getYaml<YAML::Node>(config_yaml, "raster"));

    // Create overall taskflow for full process plan
    TaskflowGenerator::UPtr robotTaskflow =
        std::make_unique<RasterTaskflow>(std::move(freespace_task), std::move(transition_task), std::move(raster_task));

    // Register newly created taskflow to the planning server so it can be called
    planning_server.registerProcessPlanner("RobotProcessTaskflow", std::move(robotTaskflow));

    return;
}
