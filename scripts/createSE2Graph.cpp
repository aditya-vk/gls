// Standard C++ libraries
#include <iostream>
#include <cmath>
#include <chrono>
#include <string>
#include <fstream>
#include <sstream>
#include <queue>
#include <Eigen/Dense>

// Boost libraries
#include <boost/shared_ptr.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/function.hpp>
#include <boost/program_options.hpp>

// PRL libraries
#include <aikido/io/CatkinResourceRetriever.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>

#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionOption.hpp>
#include <dart/collision/CollisionDetector.hpp>

namespace po = boost::program_options;

using dart::collision::CollisionDetectorPtr;
using dart::collision::CollisionGroup;
using dart::dynamics::SkeletonPtr;
using aikido::rviz::WorldInteractiveMarkerViewer;

static const std::string topicName("dart_markers");
static const std::string baseFrameName("map");

// =====================================================================================
/// Displays a message to the user and waits for input.
/// \param[in] message Message to show the user.
void waitForUser(std::string message)
{
  std::string completeMessage = message + " Press [Enter] to continue.";
  std::cout << completeMessage << std::endl;
  std::cin.get();
}

// =====================================================================================
/// Collision checks the robot and the world.
/// \param[in] world The robot's environment.
/// \param[in] robot The robot skeleton.
/// \param[in] haltonPoint The configuration of the robot to be checked for collision.
bool isPointValid(
  SkeletonPtr world,
  SkeletonPtr robot,
  std::shared_ptr<CollisionGroup> worldGroup,
  std::shared_ptr<CollisionGroup> robotGroup,
  CollisionDetectorPtr collisionDetector,
  ::dart::collision::CollisionResult collisionResult,
  ::dart::collision::CollisionOption collisionOptions,
  const Eigen::VectorXd haltonPoint
  )
{
  return true;
  // Convert positions to Eigen. DART To OMPL settings: angles, positions: ax, az, ay, x, z, y.
  Eigen::VectorXd positions(6);
  positions << 0.0, haltonPoint(2), 0.0, haltonPoint(0), 0.0, haltonPoint(1);

  // Set Positions for the robot.
  robot->setPositions(positions);

  // Check collisions and return the result.
  bool collisionStatus = collisionDetector->collide(worldGroup.get(), robotGroup.get(), collisionOptions,
        &collisionResult);

  return !collisionStatus;
}

// =====================================================================================
/// Creates a DART skeleton from URDF.
/// \param[in] resourceRetreiver Retrieves the URDF file.
/// \param[in] uri Location of the URDF.
/// \param[in] transform The pose of the skeleton.
const SkeletonPtr makeBodyFromURDF(
    const std::shared_ptr<aikido::io::CatkinResourceRetriever>
        resourceRetriever,
    const std::string& uri,
    const Eigen::Isometry3d& transform)
{
  dart::utils::DartLoader urdfLoader;
  const SkeletonPtr skeleton = urdfLoader.parseSkeleton(uri, resourceRetriever);

  if (!skeleton)
    throw std::runtime_error("unable to load '" + uri + "'");

  dynamic_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0))
      ->setTransform(transform);
  return skeleton;
}

// =====================================================================================
/// Generates a Halton Point given the index
/// \param[in] index Index in the sequence.
/// \param[in] offset The offset to be applied to all the points in the sequence.
/// \param[in] lowerLimits Lower positional limits of the robot joints.
/// \param[in] upperLimits Upper positional limits of the robot joints.
Eigen::VectorXd haltonSequence(
    int index,
    Eigen::VectorXd offset,
    Eigen::VectorXd lowerLimits,
    Eigen::VectorXd upperLimits)
{
  Eigen::VectorXd difference(3);
  difference = upperLimits - lowerLimits;

  // Catering to only ADA and HERB for now.
  Eigen::VectorXd bases(3);
  bases << 2, 3, 5;

  // Generate the halton config
  Eigen::VectorXd config(3);
  for (int i = 0; i < 3; ++i)
  {
    int base = bases[i];
    int tempIndex = index;
    double result = 0;
    double f = 1;

    while (tempIndex > 0)
    {
      f = f/base;
      result = result + f*(tempIndex % base);
      tempIndex = tempIndex/base;
    }
    config[i] = result;
  }

  // Offset the halton sequence
  config = config + offset;

  // Wrap Around
  for (int i = 0; i < 3; ++i)
  {
    if (config[i] > 1.0)
      config[i] = config[i] - 1.0;
    if (config[i] < 0.0)
      config[i] = 1.0 + config[i];
  }

  // Scale the configurations
  Eigen::VectorXd scaledConfig(3);
  for (int i = 0; i < 3; ++i)
  {
    scaledConfig(i) = lowerLimits(i) + config(i) * difference(i);
  }
  return scaledConfig;
}

// =====================================================================================
void generateHaltonPoints(SkeletonPtr world, SkeletonPtr robot, std::size_t numSamples, 
                Eigen::VectorXd lowerLimits, Eigen::VectorXd upperLimits, double threshold, bool knn)
{

  CollisionDetectorPtr collisionDetector
      = dart::collision::FCLCollisionDetector::create();

  ::dart::collision::CollisionOption collisionOptions
      = ::dart::collision::CollisionOption(
          false,
          1,
          std::make_shared<::dart::collision::BodyNodeCollisionFilter>());

  ::dart::collision::CollisionResult collisionResult;

  std::shared_ptr<CollisionGroup> worldGroup
      = collisionDetector->createCollisionGroup(world.get());

  std::shared_ptr<CollisionGroup> robotGroup
    = collisionDetector->createCollisionGroup(robot.get());


  // Holds the vertices. Index is the line number. Content is the configuration.
  std::string vertexFile = "/home/adityavk/workspaces/lab-ws/src/planning_dataset/free_vertices.txt";

  // Holds the edge information. <source vertex ID> <target vertex ID> <length>
  std::string edgesFile = "/home/adityavk/workspaces/lab-ws/src/planning_dataset/free_edges.txt";

  // Holds each edge's source and target configurations. Useful to visualize the edges.
  // Not necessary for graph generation.
  std::string edgesVizFile = "/home/adityavk/workspaces/lab-ws/src/planning_dataset/free_edges_viz.txt";

  // Generate configurations.
  std::size_t numVertices = 0;
  int index = 1;
  std::srand((unsigned int) time(0));

  // Generate a uniform offset
  Eigen::VectorXd offset = Eigen::VectorXd::Random(3)*0;

  std::vector<Eigen::VectorXd> configurations;
  while (true)
  {
    if (numVertices % 100 == 0)
      std::cout << numVertices << std::endl;

    Eigen::VectorXd config = haltonSequence(index, offset, lowerLimits, upperLimits);

    index = index + 1;

    if(!isPointValid(world, robot, worldGroup, robotGroup, collisionDetector, collisionResult, collisionOptions, config))
      continue;
    else
      configurations.emplace_back(config);

    numVertices = numVertices + 1;
    if (numVertices == numSamples)
      break;
  }

  // Insert all the vertices into the vertexFile.
  std::ofstream vertexLogFile;
  vertexLogFile.open(vertexFile, std::ios_base::app);

  for(const auto& config : configurations)
    vertexLogFile << config[0] << " " << config[1] << " " << config[2] << std::endl;

  vertexLogFile.close();

  // =================================================
  // Vertices generated. Now we try to make the edges.
  // =================================================
  std::ofstream edgesLogFile;
  edgesLogFile.open(edgesFile, std::ios_base::app);

  std::ofstream edgesVizLogFile;
  edgesVizLogFile.open(edgesVizFile, std::ios_base::app);

  assert(configurations.size() == numSamples);

  std::size_t connectedEdges = 0;

  std::map<double, std::size_t> neighbors;
  if (knn)
  {
    for (std::size_t i = 0; i < configurations.size()-1; ++i)
    {
      if (i % 100 == 0)
        std::cout << "Currently on " << i << std::endl;
      for (std::size_t j = i+1; j < configurations.size(); ++j)
      {
        // Check if vertices are closeby.
        Eigen::VectorXd tangent = configurations[i] - configurations[j];
        double linearDistance;
        double angularDistance;

        // Difference between R^2 positions
        linearDistance = (tangent.head<2>()).norm();

        // Difference between angles
        angularDistance = std::abs(tangent[2]);
        double distance = linearDistance + angularDistance;

        neighbors.insert(std::pair<double, std::size_t>(distance, j));
      }

      // Now that all the edges have been collected to a single vertex,
      // choose the top 8 edges.

      int currentEdges = 0;
      for (auto it = neighbors.begin(); it != neighbors.end(); ++it)
      {
        if (it->first > 20)
          continue;

        edgesLogFile << i << " " << it->second << " " << it->first << std::endl;

        for (int index = 0; index < 3; ++index)
        {
          edgesVizLogFile << configurations[i](index) << " ";
        }

        for (int index = 0; index < 3; ++index)
        {
          edgesVizLogFile << configurations[it->second](index) << " ";
        }
        edgesVizLogFile << std::endl;

        currentEdges++;
        if (currentEdges == 8)
          break;
      }
      connectedEdges = connectedEdges + currentEdges;
      neighbors.clear();
    }
    edgesLogFile.close();
    std::cout << "Total Vertices " << configurations.size() << " Edges " << connectedEdges << std::endl;
  }

  else
  {
    for (std::size_t i = 0; i < configurations.size()-1; ++i)
    {
      if (i % 100 == 0)
        std::cout << "Currently on " << i << std::endl;
      for (std::size_t j = i+1; j < configurations.size(); ++j)
      {
        // Check if vertices are closeby.
        double distance = (configurations[i] - configurations[j]).norm();

        if (distance > threshold)
          continue;

        connectedEdges++;

        edgesLogFile << i << " " << j << " " << distance << std::endl;

        for (int index = 0; index < 3; ++index)
        {
          edgesVizLogFile << configurations[i](index) << " ";
        }

        for (int index = 0; index < 3; ++index)
        {
          edgesVizLogFile << configurations[j](index) << " ";
        }
        edgesVizLogFile << std::endl;

      }
    }
    edgesLogFile.close();
    std::cout << "Total Vertices " << configurations.size() << " Edges " << connectedEdges << std::endl;
  }
}

// =====================================================================================
int main(int argc, char *argv[])
{
  po::options_description desc("HERB Planner Options");
  desc.add_options()
      ("help,h", "produce help message")
      ("world,w", po::value<std::string>()->required(), "world to load")
      ("robot,r", po::value<std::string>()->required(), "robot to load")
      ("number,n", po::value<std::size_t>()->required(), "number of vertices in the graph")
      ("lower,l", po::value<std::vector<double>>()->required(), "lower limits")
      ("upper,u", po::value<std::vector<double>>()->required(), "upper limits")
  ;

  // Read arguments
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).style(
  po::command_line_style::unix_style ^ po::command_line_style::allow_short
  ).run(), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }

  std::string worldName(vm["world"].as<std::string>());
  std::string robotName(vm["robot"].as<std::string>());
  std::size_t numSamples(vm["number"].as<std::size_t>());
  std::vector<double> lLimits(vm["lower"].as<std::vector<double>>());
  std::vector<double> uLimits(vm["upper"].as<std::vector<double>>());

  /// HERB ENVIRONMENT
  ROS_INFO("Starting ROS node.");
  ros::init(argc, argv, "ompl_app");
  ros::NodeHandle nh("~");

  // Create AIKIDO World
  aikido::planner::WorldPtr env(new aikido::planner::World("ompl_env"));

  // Visualization topics
  static const std::string execTopicName = topicName + "/exec";

  // Start the RViz viewer.
  ROS_INFO_STREAM("Starting viewer. Please subscribe to the '" << execTopicName << "' InteractiveMarker topic in RViz.");
  WorldInteractiveMarkerViewer viewer(env, execTopicName, baseFrameName);
  viewer.setAutoUpdate(true);

  // Resolves package:// URIs by emulating the behavior of 'catkin_find'.
  const auto resourceRetriever = std::make_shared<aikido::io::CatkinResourceRetriever>();
  const std::string worldURDFUri = "package://pr_assets/data/ompl/" + worldName + ".urdf";
  const std::string robotURDFUri = "package://pr_assets/data/ompl/" + robotName + ".urdf";

  // Initial perception
  SkeletonPtr world;
  Eigen::Isometry3d worldPose;

  SkeletonPtr robot;
  Eigen::Isometry3d robotPose;

  // Poses for world
  worldPose = Eigen::Isometry3d::Identity();

  // Poses for robot
  robotPose = Eigen::Isometry3d::Identity();

  // Load objects
  world = makeBodyFromURDF(resourceRetriever, worldURDFUri, worldPose);
  robot = makeBodyFromURDF(resourceRetriever, robotURDFUri, robotPose);

  // Add all objects to World
  env->addSkeleton(world);
  env->addSkeleton(robot);

  // Set the right limits.
  Eigen::VectorXd lowerLimits(3);
  Eigen::VectorXd upperLimits(3);
  lowerLimits << -73, -179, -3.14;
  upperLimits << 300, 168, 3.14;

  Eigen::VectorXd difference(3);
  difference = upperLimits - lowerLimits;
  double threshold = std::pow(numSamples, -1.0/3)*std::max(difference(0), difference(1));
  std::cout << "Threshold Used: " << threshold << std::endl;

  waitForUser("The environment has been setup. Press key to start generating the graph");

  // Generate the Halton Points.
  generateHaltonPoints(world, robot, numSamples, lowerLimits, upperLimits, threshold, true);

  waitForUser("Press enter to exit");
  return 0;
}
