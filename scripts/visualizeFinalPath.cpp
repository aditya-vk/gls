// Standard C++ libraries
#include <iostream>
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

// OMPL base libraries
#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>

// PRL libraries
#include <aikido/constraint/dart/TSR.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/constraint/dart/CollisionFree.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/io/CatkinResourceRetriever.hpp>
#include <aikido/planner/ompl/GeometricStateSpace.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionOption.hpp>
#include <dart/collision/CollisionDetector.hpp>

// Custom header files
#include "gls/GLS.hpp"

namespace po = boost::program_options;

using dart::collision::CollisionDetectorPtr;
using dart::collision::CollisionGroup;
using dart::dynamics::SkeletonPtr;
using aikido::rviz::WorldInteractiveMarkerViewer;

using aikido::constraint::dart::CollisionFree;
using aikido::statespace::dart::MetaSkeletonStateSpace;

static const std::string topicName("dart_markers");
static const std::string configDataURI("package://pr_assets/data/objects/tag_data.json");
static const std::string baseFrameName("map");

void waitForUser(std::string message)
{
  std::string completeMessage = message + " Press [Enter] to continue.";
  std::cout << completeMessage << std::endl;
  std::cin.get();
}

/// Reads states from file
///
/// \param[in] path Path to the source file.
/// \return vector of states
std::vector<Eigen::VectorXd> readStatesFromFile(std::string path)
{
  std::ifstream inputFile(path);

  std::vector<Eigen::VectorXd> configurations;
  if (inputFile)
  {
    while (true)
    {
      std::string line;
      double value;

      std::getline(inputFile, line);

      if(line[0] == '#')
        continue;

      std::stringstream ss(
            line, std::ios_base::out|std::ios_base::in|std::ios_base::binary);

      if (!inputFile)
        break;
      Eigen::VectorXd row(3);
      int index = 0;
      while (ss >> value)
      {
        row(index) = value;
        index++;
      }
      configurations.emplace_back(row);
    }
  }

  return configurations;
}

bool isEigenPointValid(
  SkeletonPtr world,
  SkeletonPtr robot,
  std::shared_ptr<CollisionGroup> worldGroup,
  std::shared_ptr<CollisionGroup> robotGroup,
  CollisionDetectorPtr collisionDetector,
  ::dart::collision::CollisionResult collisionResult,
  ::dart::collision::CollisionOption collisionOptions,
  Eigen::VectorXd& pos
  )
{
  // Convert positions to Eigen. DART To OMPL settings: angles, positions: ax, az, ay, x, z, y.
  Eigen::VectorXd positions(6);
  positions << 0.0, pos(2), 0.0, pos(0), 0.0, pos(1);

  // Set Positions for the robot.
  robot->setPositions(positions);

  // Check collisions and return the result.
  bool collisionStatus = collisionDetector->collide(worldGroup.get(), robotGroup.get(), collisionOptions,
        &collisionResult);

  return !collisionStatus;
}

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


int main(int argc, char *argv[])
{
  po::options_description desc("HERB Planner Options");
  desc.add_options()
      ("help,h", "produce help message")
      ("world,w", po::value<std::string>()->required(), "world to load")
      ("robot,r", po::value<std::string>()->required(), "robot to load")
      ("graph,g", po::value<std::string>()->required(), "graph to load")
  ;

  // Read arguments
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }

  std::string worldName(vm["world"].as<std::string>());
  std::string robotName(vm["robot"].as<std::string>());
  std::string graphName(vm["graph"].as<std::string>());

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

  // Set the collision model.
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

  waitForUser("The environment has been setup. Press key to start planning");

  auto stateList = readStatesFromFile("/home/adityavk/programs/omplapp-1.4.2-Source/resources/3D/Apartment_gls.path");

  for (std::size_t i = 0; i < stateList.size(); ++i)
  {
    waitForUser("Set State");
    std::cout << "Validity: " << isEigenPointValid(world, robot, worldGroup, robotGroup, collisionDetector, collisionResult, collisionOptions, stateList[i]) << std::endl;
  }

  waitForUser("Press enter to exit");
  return 0;
}
