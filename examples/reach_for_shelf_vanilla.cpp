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
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>

// PRL libraries
#include <aikido/constraint/dart/TSR.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/constraint/dart/CollisionFree.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/io/CatkinResourceRetriever.hpp>
#include <aikido/planner/ompl/GeometricStateSpace.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/planner/ompl/OMPLConfigurationToConfigurationPlanner.hpp>
#include <aikido/planner/ConfigurationToConfigurationPlanner.hpp>
#include <aikido/planner/ConfigurationToConfiguration.hpp>

#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionOption.hpp>
#include <dart/collision/CollisionDetector.hpp>

#include <libherb/Herb.hpp>
#include <pr_tsr/glass.hpp>

// roscpp
#include "ros/ros.h"

// Custom header files
#include "gls/GLS.hpp"

namespace po = boost::program_options;

using dart::collision::CollisionDetectorPtr;
using dart::collision::CollisionGroup;
using dart::dynamics::SkeletonPtr;
using aikido::rviz::WorldInteractiveMarkerViewer;
using aikido::planner::ConfigurationToConfiguration;
using aikido::planner::ompl::OMPLConfigurationToConfigurationPlanner;

using aikido::constraint::dart::CollisionFree;
using aikido::statespace::dart::MetaSkeletonStateSpace;

static const std::string topicName("dart_markers");
static const std::string markerTopicName("/apriltags/marker_array");
static const std::string configDataURI("package://pr_assets/data/objects/tag_data.json");
static const std::string herbFrameName("herb_frame");
static const std::string tableName("table127");
static const std::string shelfName("shelf");
static const std::string baseFrameName("map");

bool mExecute = false;
double mCollisionCheckTime = 0;

// ===================================================================================================
void waitForUser(std::string message)
{
  std::string completeMessage = message + " Press [Enter] to continue.";
  std::cout << completeMessage << std::endl;
  std::cin.get();
}

// ===================================================================================================
bool isPointValid(const aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
                  const aikido::constraint::TestablePtr constraint,
                  const ompl::base::State* state)
{
  std::chrono::time_point<std::chrono::system_clock> startTime{std::chrono::system_clock::now()};
  DART_UNUSED(stateSpace);
  const auto* st = static_cast<const aikido::planner::ompl::GeometricStateSpace::StateType*>(state);
  const auto testState = st->mState;

  bool valid = constraint->isSatisfied(testState);

  std::chrono::time_point<std::chrono::system_clock> endTime{std::chrono::system_clock::now()};
  std::chrono::duration<double> elapsedSeconds{endTime-startTime};
  mCollisionCheckTime += elapsedSeconds.count();
  return valid;
}

// ===================================================================================================
ompl::base::ScopedState<ompl::base::RealVectorStateSpace>
make_state(const ompl::base::StateSpacePtr space, Eigen::VectorXd vals)
{
   ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state(space);
   double * values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   for (int i = 0; i < space->getDimension(); i++)
   {
     values[i] = vals[i];
   }
   return state;
}

// ===================================================================================================
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

// ===================================================================================================
void shortcutAndLog(ompl::base::ProblemDefinitionPtr pdef, double plantime, double timelimit, double costlimit)
{
  auto path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(
      pdef->getSolutionPath());
  double originalpathlength = path->length();

  std::chrono::time_point<std::chrono::system_clock> time_before{std::chrono::system_clock::now()};
  std::chrono::time_point<std::chrono::system_clock> time_current{std::chrono::system_clock::now()};
  std::chrono::duration<double> time_limit = std::chrono::duration<double>(timelimit);

  ompl::geometric::PathSimplifier simplifier{pdef->getSpaceInformation()};
  while (time_current - time_before <= time_limit && path->length() > costlimit)
  {
    auto shortened = simplifier.shortcutPath(*path, 1, 1, 0.33, 0.0);
    time_current = std::chrono::system_clock::now();
  }
  std::chrono::duration<double> shortcutTime{time_current - time_before};

  std::string filename = "/home/prl/workspaces/lab-ws/src/planning_dataset/results/rrt_reach_for_shelf_vanilla.txt";

  std::ofstream logFile;
  logFile.open(filename, std::ios_base::app);
  logFile << plantime << " " << shortcutTime.count() << " " << mCollisionCheckTime << " " << originalpathlength << " " << path->length() << std::endl;
}

// ===================================================================================================
void logInformation(ompl::base::ProblemDefinitionPtr pdef, int graphsize, double totaltime, double collisionchecktime, std::size_t evals, std::size_t rewires)
{
  std::string filename = "/home/prl/workspaces/lab-ws/src/planning_dataset/results/lazysp_reach_for_shelf_vanilla.txt";
  auto path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(
      pdef->getSolutionPath());
  double originalpathlength = path->length();

  std::ofstream logFile;
  logFile.open(filename, std::ios_base::app);
  logFile << graphsize << " " << totaltime << " " << mCollisionCheckTime << " " << originalpathlength << " " << evals << " " << rewires << std::endl;
}

// ===================================================================================================
int main(int argc, char *argv[])
{

  po::options_description desc("HERB Planner Options");
  desc.add_options()
      ("help,h", "produce help message")
      ("roadmapfile,f", po::value<std::string>()->required(), "Path to Graph")
      ("planner,p", po::value<std::string>()->required(), "Planner")
      ("execute,e", po::bool_switch(&mExecute), "Execute the plan on HERB")
      ("limit,l", po::value<double>()->required(), "Limit on the solution cost")
      ("graphsize,g", po::value<int>()->required(), "Graph Size")
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

  std::string plannerName(vm["planner"].as<std::string>());
  std::string graph_file(vm["roadmapfile"].as<std::string>());
  double solutionlimit(vm["limit"].as<double>());
  int graphsize(vm["graphsize"].as<int>());


// =======================
// POSSIBLE CONFIGURATIONS
// =======================

  Eigen::VectorXd leftRelaxedHome(7);
  leftRelaxedHome << 0.54, -1.50, 0.26, 1.90, 1.16, 0.87, 1.43;

  Eigen::VectorXd rightRelaxedHome(7);
  rightRelaxedHome << 5.65, -1.50, -0.26, 1.96, -1.15, 0.87, -1.43;

  Eigen::VectorXd rightFarLeftReach(7);
  rightFarLeftReach << 5.30, 1.50, -0.12, 0.87, 1.21, 0.34, 0.33;

// =======================
// POSSIBLE CONFIGURATIONS
// =======================

  /// HERB ENVIRONMENT
  ROS_INFO("Starting ROS node.");
  ros::init(argc, argv, "herb");
  ros::NodeHandle nh("~");

  // Create AIKIDO World
  aikido::planner::WorldPtr env(new aikido::planner::World("exec"));

  // Robot Setup
  herb::Herb robot(env, true);
  auto robotSkeleton = robot.getMetaSkeleton();

  // Visualization topics
  static const std::string execTopicName = topicName + "/exec";

//  // Start the RViz viewer.
//  ROS_INFO_STREAM("Starting viewer. Please subscribe to the '" << execTopicName << "' InteractiveMarker topic in RViz.");
//  WorldInteractiveMarkerViewer viewer(env, execTopicName, baseFrameName);
//  viewer.setAutoUpdate(true);

  // Obtain the space to plan in
  auto leftArm = robot.getLeftArm()->getMetaSkeleton();
  auto rightArm = robot.getRightArm()->getMetaSkeleton();

  auto leftArmSpace = std::make_shared<MetaSkeletonStateSpace>(leftArm.get());
  auto rightArmSpace = std::make_shared<MetaSkeletonStateSpace>(rightArm.get());

  // Resolves package:// URIs by emulating the behavior of 'catkin_find'.
  const auto resourceRetriever = std::make_shared<aikido::io::CatkinResourceRetriever>();
  const std::string tableURDFUri("package://pr_assets/data/furniture/table.urdf");
  const std::string shelfURDFUri("package://pr_assets/data/furniture/bookcase.urdf");
  const std::string pitcherURDFUri("package://pr_assets/data/objects/rubbermaid_ice_guard_pitcher.urdf");
  const std::string roofURDFUri("package://pr_assets/data/furniture/table.urdf");

  // Initial perception
  SkeletonPtr table;
  Eigen::Isometry3d tablePose;
  SkeletonPtr shelf;
  Eigen::Isometry3d shelfPose;

  SkeletonPtr pitcher;
  Eigen::Isometry3d pitcherPose;
  SkeletonPtr roof;
  Eigen::Isometry3d roofPose;

  // Poses for table
  tablePose = Eigen::Isometry3d::Identity();
  tablePose.translation() = Eigen::Vector3d(0.85, -0.4, 0);
  Eigen::Matrix3d rot;
  rot = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
  tablePose.linear() = rot;

  // Poses for Shelf
  shelfPose = Eigen::Isometry3d::Identity();
  shelfPose.translation() = Eigen::Vector3d(-0.0, 0.75, 0);
  rot = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
       * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
       * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
  shelfPose.linear() = rot;

    // Poses for Shelf
  pitcherPose = Eigen::Isometry3d::Identity();
  pitcherPose.translation() = tablePose.translation() + 
    Eigen::Vector3d(0.0, 0.0, 0.73);

    // Poses for Oven
  roofPose = Eigen::Isometry3d::Identity();
  roofPose.translation() = tablePose.translation() + 
    Eigen::Vector3d(0.0, 0.0, 2.3);
  rot = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())
       * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY())
       * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
  roofPose.linear() = rot;

  // Load objects
  table = makeBodyFromURDF(resourceRetriever, tableURDFUri, tablePose);
  shelf = makeBodyFromURDF(resourceRetriever, shelfURDFUri, shelfPose);
  pitcher = makeBodyFromURDF(resourceRetriever, pitcherURDFUri, pitcherPose);
  roof = makeBodyFromURDF(resourceRetriever, roofURDFUri, roofPose);

  // Add all objects to World
  env->addSkeleton(table);
  env->addSkeleton(shelf);
  env->addSkeleton(pitcher);
  env->addSkeleton(roof);

  // Set up collision constraints for planning.
  CollisionDetectorPtr collisionDetector
      = dart::collision::FCLCollisionDetector::create();

  std::shared_ptr<CollisionGroup> rightArmGroup
      = collisionDetector->createCollisionGroup(
        robot.getRightArm()->getMetaSkeleton().get(),
        robot.getRightHand()->getMetaSkeleton().get());

  std::shared_ptr<CollisionGroup> leftArmGroup
      = collisionDetector->createCollisionGroup(
        robot.getLeftArm()->getMetaSkeleton().get(),
        robot.getLeftHand()->getMetaSkeleton().get());

  std::shared_ptr<CollisionGroup> envGroup
      = collisionDetector->createCollisionGroup(
                                                  table.get(),
                                                  shelf.get(),
                                                  pitcher.get(),
                                                  roof.get()
                                                );

  auto rightCollisionFreeConstraint = std::make_shared<CollisionFree>(
        rightArmSpace, robot.getRightArm()->getMetaSkeleton(), collisionDetector);
  rightCollisionFreeConstraint->addPairwiseCheck(rightArmGroup, envGroup);
  rightCollisionFreeConstraint->addPairwiseCheck(rightArmGroup, leftArmGroup);
  auto fullCollisionConstraint = robot.getFullCollisionConstraint(rightArmSpace, 
    robot.getRightArm()->getMetaSkeleton(), rightCollisionFreeConstraint);

  /// Planner Setup

  rightArm->setPositions(rightRelaxedHome);
  leftArm->setPositions(leftRelaxedHome);
  robot.getRightHand()->executePreshape("closed").wait();

  // Define the state space: R^7
  auto space = std::make_shared<ompl::base::RealVectorStateSpace>(7);
  ompl::base::RealVectorBounds bounds(7);

  bounds.setLow(0, 0.54);
  bounds.setLow(1, -2.00);
  bounds.setLow(2, -2.80);
  bounds.setLow(3, -0.9);
  bounds.setLow(4, -4.76);
  bounds.setLow(5, -1.60);
  bounds.setLow(6, -3.00);

  bounds.setHigh(0, 5.75);
  bounds.setHigh(1, 2.00);
  bounds.setHigh(2, 2.80);
  bounds.setHigh(3, 3.10);
  bounds.setHigh(4, 1.24);
  bounds.setHigh(5, 1.60);
  bounds.setHigh(6, 3.00);

  space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
  space->setup();

  // Space Information
  ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
  std::function<bool(const ompl::base::State*)> isStateValid = std::bind(
        isPointValid, rightArmSpace, fullCollisionConstraint, std::placeholders::_1);
  si->setStateValidityChecker(isStateValid);
  si->setStateValidityCheckingResolution(0.9);
  si->setup();

  // Problem Definition
  ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
  pdef->addStartState(make_state(space, rightRelaxedHome));
  pdef->setGoalState(make_state(space, rightFarLeftReach));

  // Call the requested planner.
  ompl::base::PlannerStatus status;
  if (plannerName == "lazysp")
  {
    // Setup planner
    gls::GLS planner(si);
    planner.setConnectionRadius(1.5);
    planner.setRoadmap(graph_file);

    auto event = std::make_shared<gls::event::ShortestPathEvent>();
    auto selector = std::make_shared<gls::selector::ForwardSelector>();
    planner.setEvent(event);
    planner.setSelector(selector);

    planner.setup();
    planner.setProblemDefinition(pdef);

    std::chrono::time_point<std::chrono::system_clock> startTime{std::chrono::system_clock::now()};
    status = planner.solve(ompl::base::plannerNonTerminatingCondition());
    std::chrono::time_point<std::chrono::system_clock> endTime{std::chrono::system_clock::now()};
    std::chrono::duration<double> elapsedSeconds{endTime-startTime};
    double plantime = elapsedSeconds.count();

    if (status == ompl::base::PlannerStatus::EXACT_SOLUTION)
      logInformation(pdef, graphsize, plantime, mCollisionCheckTime, planner.getNumberOfEdgeEvaluations(), planner.getNumberOfEdgeRewires());
  }

  else if (plannerName == "birrt")
  {
    ompl::geometric::RRTConnect planner(si);
    planner.setRange(8.0);
    planner.setup();
    planner.setProblemDefinition(pdef);

    std::chrono::time_point<std::chrono::system_clock> startTime{std::chrono::system_clock::now()};
    status = planner.solve(ompl::base::plannerNonTerminatingCondition());
    std::chrono::time_point<std::chrono::system_clock> endTime{std::chrono::system_clock::now()};
    std::chrono::duration<double> elapsedSeconds{endTime-startTime};
    double plantime = elapsedSeconds.count();

    if (status == ompl::base::PlannerStatus::EXACT_SOLUTION)
      shortcutAndLog(pdef, plantime, 20.0, solutionlimit);
  }
  else
  {
    throw std::invalid_argument("Planner not supported. Available planners: {LazySP, BiRRT, BIT*}");
  }

  // Obtain required data if plan was successful
  if (mExecute)
  {
    if (status == ompl::base::PlannerStatus::EXACT_SOLUTION)
    {
      // Get planner data if required
      auto path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(
        pdef->getSolutionPath());
      std::size_t pathSize = path->getStateCount();

      auto untimedTrajectory = std::make_shared<aikido::trajectory::Interpolated>(
           std::move(rightArmSpace), std::move(std::make_shared<aikido::statespace::GeodesicInterpolator>(rightArmSpace)));
      for (std::size_t idx = 0; idx < pathSize; ++idx)
      {
       const auto* st = static_cast<aikido::planner::ompl::GeometricStateSpace::StateType*>(path->getState(idx));
       untimedTrajectory->addWaypoint(idx, st->mState);
      }

      auto timedTrajectory = robot.retimePath(rightArm, untimedTrajectory.get());

      waitForUser("Press key to execute");
      robot.executeTrajectory(std::move(timedTrajectory)).wait();
    }
  }

  return 0;
}
