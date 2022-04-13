#include <ros/ros.h>
#include <ros/node_handle.h>
#include <std_srvs/Trigger.h>

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
static const std::string MONITOR_NAMESPACE = "environment_monitor";
static const std::string MONITOR_ENVIRONMENT_NAMESPACE = "robotcloud_environment";
static const std::string PLAN_PROCESS_ACTION_NAME = "plan_process";

namespace robot_motion_planning
{

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
        process_plan_server_ = nh_.advertiseService(PLAN_PROCESS_ACTION_NAME, &MotionPlanner::planProcessCallback, this);

        // Load in urdf and srdf
        std::string urdf_xml_string, srdf_xml_string;
        nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);

        // Initialize tesseract environment
        tesseract_common::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
        if (!env_->init(urdf_xml_string, srdf_xml_string, locator))
        {
            ROS_ERROR_STREAM("Failed to initialize tesseract environment.");
        }
    }

    bool doMotionPlanning(YAML::Node config_yaml)
    {
        // Load in points.csv to a vector of Eigen::Isometry3d
        std::string support_path = ros::package::getPath("rpi_abb_irb6640_180_255_support");
//        std::string filepath = support_path + "/config/points.csv";
//        YAML::Node waypoint_config = getYaml<YAML::Node>(config_yaml, "waypoints");
//        int freq = getYaml<int>(waypoint_config, "downsample_frequency");

//        std::vector<Eigen::Isometry3d> toolpath = loadPoints(filepath, freq);
//        ROS_INFO_STREAM("Loaded " << toolpath.size() << " points");

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
        tesseract_request.name = "RobotCloudProcessTaskflow";

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

    bool planProcessCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
    {
        // Load planner config yaml file
        std::string support_path = ros::package::getPath("robotcloud_support");
        std::string config_fp = support_path + "/config/planner_config.yaml";
        YAML::Node config_yaml = YAML::LoadFile(config_fp);

        bool success = doMotionPlanning(config_yaml);

        res.success = success;
        return true;
    }

    std::vector<Eigen::Isometry3d> loadPoints(const std::string& filepath, int& freq)
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
                if (tokens.size() == 3)
                {
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

    ros::NodeHandle nh_;

    // ROS Service Server for triggering motion planning
    ros::ServiceServer process_plan_server_;

    // Tesseract required components for keeping up with and publishing the environment state
    tesseract_environment::Environment::Ptr env_;
    tesseract_monitoring::EnvironmentMonitor::Ptr monitor_;
};
} // robot_motion_planning namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;
    robot_motion_planning::MotionPlanner motion_planner(nh);

    ros::spin();
}
