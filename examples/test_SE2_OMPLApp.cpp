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

bool isPointValid(
  SkeletonPtr world,
  SkeletonPtr robot,
  const ompl::base::State* state
  )
{
  return true;
  // Obtain the values from state.
  auto se2State = state->as<ompl::base::SE2StateSpace::StateType>();

  // Convert positions to Eigen. DART To OMPL settings: angles, positions: ax, az, ay, x, z, y.
  Eigen::VectorXd positions(6);
  positions << se2State->getX(), 0.0, se2State->getY(), 0.0, se2State->getYaw(), 0.0;

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
  const std::string worldURDFUri = "package://pr_assets/data/ompl/2D/" + worldName + ".urdf";
  const std::string robotURDFUri = "package://pr_assets/data/ompl/2D/" + robotName + ".urdf";

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

  // Define the state space: SE(2)
  auto space = std::make_shared<ompl::base::SE2StateSpace>();
  ompl::base::RealVectorBounds bounds(2);

  bounds.setLow(0, -55.0);
  bounds.setLow(1, -55.0);

  bounds.setHigh(0, 55.0);
  bounds.setHigh(1, 55.0);

  space->setBounds(bounds);
  space->setup();

  // Space Information
  ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
  std::function<bool(const ompl::base::State*)> isStateValid = std::bind(
        isPointValid, world, robot, std::placeholders::_1);
  si->setStateValidityChecker(isStateValid);
  si->setup();

  // Problem Definition
  ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));

  ompl::base::ScopedState<ompl::base::SE2StateSpace> start(si);
  start->setX(7.02);
  start->setY(-12.0);
  start->setYaw(0.0);

  ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(start);
  goal->setX(-36.98);
  goal->setY(-10);
  goal->setYaw(2.25);
  
  pdef->addStartState(start);
  pdef->setGoalState(goal);

  // Setup planner
  gls::GLS planner(si);
  planner.setConnectionRadius(1.5);
  planner.setCollisionCheckResolution(0.1);
  planner.setRoadmap("/home/adityavk/workspaces/lab-ws/src/planning_dataset/se2.graphml");

  auto event = std::make_shared<gls::event::ShortestPathEvent>();
  auto selector = std::make_shared<gls::selector::ForwardSelector>();
  planner.setEvent(event);
  planner.setSelector(selector);

  planner.setup();
  planner.setProblemDefinition(pdef);

  // Solve the motion planning problem
  ompl::base::PlannerStatus status;
  waitForUser("Solve the problem.");
  status = planner.solve(ompl::base::plannerNonTerminatingCondition());
  waitForUser("Solved the problem.");

  // Obtain required data if plan was successful
  if (status == ompl::base::PlannerStatus::EXACT_SOLUTION)
  {
    auto path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(
      pdef->getSolutionPath());
    path->printAsMatrix(std::cout);
  }

  waitForUser("Press enter to exit");
  return 0;
}
