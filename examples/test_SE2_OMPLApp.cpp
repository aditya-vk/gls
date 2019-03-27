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
static const std::string markerTopicName("/apriltags/marker_array");
static const std::string configDataURI("package://pr_assets/data/objects/tag_data.json");
static const std::string herbFrameName("herb_frame");
static const std::string tableName("table127");
static const std::string shelfName("shelf");
static const std::string baseFrameName("map");

void waitForUser(std::string message)
{
  std::string completeMessage = message + " Press [Enter] to continue.";
  std::cout << completeMessage << std::endl;
  std::cin.get();
}

bool isPointValid(
  SkeletonPtr world,
  SkeletonPtr robot,
  Eigen::VectorXd& positions
  // const ompl::base::State* state
  )
{
  // Obtain the values from state.
  // double * values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;

  // Convert positions to Eigen.
  // Eigen::VectorXd positions(6);
  // positions << values[0], 0.0, values[1], 0.0, values[2], 0.0;

  // Set Positions for the robot.
  robot->setPositions(positions);

  // Set up collision constraints for planning.
  // TODO (avk): check if these are required.
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

  // Check collisions and return the result.
  bool collisionStatus = collisionDetector->collide(worldGroup.get(), robotGroup.get(), collisionOptions,
        &collisionResult);

  return !collisionStatus;
}

ompl::base::ScopedState<ompl::base::RealVectorStateSpace>
make_state(const ompl::base::StateSpacePtr space, Eigen::VectorXd vals)
{
   ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state(space);
   double * values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   for (std::size_t i = 0; i < space->getDimension(); i++)
   {
     values[i] = vals[i];
   }
   return state;
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
      ("planner,p", po::value<std::string>()->required(), "Path to Graph")
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

  std::string plannerType(vm["planner"].as<std::string>());

  /// HERB ENVIRONMENT
  ROS_INFO("Starting ROS node.");
  ros::init(argc, argv, "ompl_app");
  ros::NodeHandle nh("~");

  // Create AIKIDO World
  aikido::planner::WorldPtr env(new aikido::planner::World("exec"));

  // Visualization topics
  static const std::string execTopicName = topicName + "/exec";

  // Start the RViz viewer.
  ROS_INFO_STREAM("Starting viewer. Please subscribe to the '" << execTopicName << "' InteractiveMarker topic in RViz.");
  WorldInteractiveMarkerViewer viewer(env, execTopicName, baseFrameName);
  viewer.setAutoUpdate(true);

  // Resolves package:// URIs by emulating the behavior of 'catkin_find'.
  const auto resourceRetriever = std::make_shared<aikido::io::CatkinResourceRetriever>();
  const std::string worldURDFUri("package://pr_assets/data/ompl/2D/BugTrap_planar_env.urdf");
  const std::string robotURDFUri("package://pr_assets/data/ompl/2D/car1_planar_robot.urdf");

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

  waitForUser("The environment has been setup. Press key to start planning");

  // DART To OMPL settings: angles, positions: ax, az, ay, x, z, y.
  Eigen::VectorXd pos(6);
  pos << 0, 0, 0.0, -55.0, 0.0, -55.0;
  waitForUser("Check for collision 1");
  std::cout << "corner: " << isPointValid(world, robot, pos);

  pos << 0, 0, 0.0, 0.0, 0.0, 0.0;
  waitForUser("Check for collision 2");
  std::cout << "center: " << isPointValid(world, robot, pos);

  waitForUser("Press enter to exit");
  return 0;
}