#ifndef PLANNER_UTILS_H
#define PLANNER_UTILS_H

#include <tf2_eigen/tf2_eigen.h>

#include <thread>
#include <fstream>
#include <iostream>

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/iterator.h>
#include <yaml-cpp/node/impl.h>

#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>

#include <tesseract_process_managers/core/process_planning_server.h>

#include <tesseract_process_managers/task_generators/check_input_task_generator.h>
#include <tesseract_process_managers/task_generators/has_seed_task_generator.h>
#include <tesseract_process_managers/task_generators/iterative_spline_parameterization_task_generator.h>
#include <tesseract_process_managers/task_generators/motion_planner_task_generator.h>
#include <tesseract_process_managers/task_generators/seed_min_length_task_generator.h>
#include <tesseract_process_managers/task_generators/discrete_contact_check_task_generator.h>

#include <tesseract_process_managers/taskflow_generators/raster_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/graph_taskflow.h>

#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>

#include <tesseract_process_managers/task_profiles/contact_check_profile.h>

namespace robot_motion_planning
{
/** @brief Convert vector of double to an Eigen Vector of equal length */
Eigen::VectorXd toVectorXd(std::vector<double> values)
{
    Eigen::VectorXd v(values.size());
    for (std::size_t i = 0; i < values.size(); ++i)
    {
        v[int(i)] = values[i];
    }
    return v;
};

/** @brief Convert an Eigen Vector to a vector of double of equal length */
std::vector<double> fromVectorXd(Eigen::VectorXd values)
{
    std::vector<double> v;
    for (std::size_t i = 0; i < values.size(); ++i)
    {
        v.push_back(values[int(i)]);
    }
    return v;
};

/** @brief Get a value from a specified field in a yaml file and give more detailed error handling */
template <class T>
T getYaml(const YAML::Node nd, const std::size_t key)
{
    T output;
    if (!nd[key])
    {
        std::string error_string = (boost::format("Unable to parse YAML field %u, Error: Does not exist") % key).str();
        throw(std::runtime_error(error_string));
    }
    try
    {
        output = nd[key].as<T>();
    }
    catch (YAML::TypedBadConversion<T>& ex)
    {
        std::string error_string = (boost::format("Unable to parse YAML field %u, Error: %s") % key % ex.what()).str();
        throw(std::runtime_error(error_string));
    }
    return output;
};

/** @brief Get a value from a specified field in a yaml file and give more detailed error handling */
template <class T>
T getYaml(const YAML::Node nd, const std::string key)
{
    T output;
    if (!nd[key])
    {
        std::string error_string = (boost::format("Unable to parse YAML field %s, Error: Does not exist") % key).str();
        throw(std::runtime_error(error_string));
    }
    try
    {
        output = nd[key].as<T>();
    }
    catch (YAML::TypedBadConversion<T>& ex)
    {
        std::string error_string = (boost::format("Unable to parse YAML field %s, Error: %s") % key % ex.what()).str();
        throw(std::runtime_error(error_string));
    }
    return output;
};

/**
 * @brief loadPoints Generates a vector of waypoints from a csv file, downsampling based on freq
 * @param filepath Filepath to the csv file
 * @param freq Add 1 waypoint every freq number of waypoints
 * @return A vector of waypoints in the form Eigen::Isometry3d
 */
std::vector<Eigen::Isometry3d> loadPoints(const std::string& filepath, int& freq);

/**
 * @brief generatePlanConfigFromSimplePlanConfig convert a simple config file to a full one
 * @param simple_plan_config
 * @return
 */
YAML::Node generatePlanConfigFromSimplePlanConfig(const YAML::Node& simple_plan_config);

/**
 * @brief createFreespaceGenerator Function that creates the taskflow process for freespace motions based on a yaml
 * @param config_yaml Configuration of what processes to use
 * @return
 */
tesseract_planning::TaskflowGenerator::UPtr createFreespaceGenerator(const YAML::Node& config_yaml);

/**
 * @brief createRasterGenerator Function that creates the taskflow process for raster motions based on a yaml
 * @param config_yaml Configuration of what processes to use
 * @return
 */
tesseract_planning::TaskflowGenerator::UPtr createRasterGenerator(const YAML::Node& config_yaml);

/**
 * @brief generateProcessTaskflow Function that creates the overall taskflow process for the full plan based on a yaml
 * @param planning_server Planning server that this taskflow needs to be added to
 * @param config_yaml Configuration of what processes to use
 */
void generateProcessTaskflow(tesseract_planning::ProcessPlanningServer& planning_server, const YAML::Node& config_yaml);

}  // namespace robot_motion_planning

#endif  // PLANNER_UTILS_H
