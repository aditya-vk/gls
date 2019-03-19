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

void waitForUser(std::string message)
{
  std::string completeMessage = message + " Press [Enter] to continue.";
  std::cout << completeMessage << std::endl;
  std::cin.get();
}

bool isPointValid(const aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
                  const aikido::constraint::TestablePtr constraint,
                  const ompl::base::State* state)
{
  DART_UNUSED(stateSpace);
  const auto* st = static_cast<const aikido::planner::ompl::GeometricStateSpace::StateType*>(state);
  const auto testState = st->mState;
  if (!constraint->isSatisfied(testState))
    return false;
  else
    return true;
}

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

// =======================
// POSSIBLE CONFIGURATIONS
// =======================

  Eigen::VectorXd leftRelaxedHome(7);
  leftRelaxedHome << 0.54, -1.50, 0.26, 1.90, 1.16, 0.87, 1.43;

  Eigen::VectorXd rightRelaxedHome(7);
  rightRelaxedHome << 5.65, -1.50, -0.26, 1.96, -1.15, 0.87, -1.43;

  Eigen::VectorXd rightReachPhone(7);
  rightReachPhone << 5.65, -0.78, 1.45, -0.70, -0.82, -0.28, -1.43;

  Eigen::VectorXd rightHoldPhone(7);
  rightHoldPhone << 3.93, -0.80, -0.27, 2.36, 1.13, 0.92, 0.0;

  Eigen::VectorXd leftMenacing(7);
  leftMenacing << 2.60, -1.90, 0.00, 2.20, 0.00, 0.00, 0.00;

  Eigen::VectorXd rightMenacing(7);
  rightMenacing << 3.68, -1.90, 0.00, 2.20, 0.00, 0.00, 0.00;

  Eigen::VectorXd rightRetrieveUp(7);
  rightRetrieveUp << 3.68, -1.90, 0.00, 0.00, 1.20, -0.12, 0.00;

  Eigen::VectorXd rightRetrieveBack(7);
  rightRetrieveBack << 5.02, -1.67, -2.40, 1.84, -0.90, 0.98, 1.60;

  Eigen::VectorXd leftHandPhone(7);
  leftHandPhone << 0.64, -1.50, 0.26, 1.96, 1.16, 0.87, 1.43;

  Eigen::VectorXd rightHandPhone(7);
  rightHandPhone << 5.02, -0.24, -0.12, 0.87, 1.21, 0.34, 0.0;

  Eigen::VectorXd rightFarLeftReach(7);
  rightFarLeftReach << 5.30, 1.50, -0.12, 0.87, 1.21, 0.34, 0.0;

  Eigen::VectorXd rightTopShelf(7);
  rightTopShelf << 3.1, -1.0, 0.0, 1.1, -0.9, 0.0, -1.43;

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

  // Start the RViz viewer.
  ROS_INFO_STREAM("Starting viewer. Please subscribe to the '" << execTopicName << "' InteractiveMarker topic in RViz.");
  WorldInteractiveMarkerViewer viewer(env, execTopicName, baseFrameName);
  viewer.setAutoUpdate(true);

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
  // env->addSkeleton(roof);

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
      = collisionDetector->createCollisionGroup(table.get(),
                                                shelf.get()
                                                // pitcher.get()
                                                // roof.get()
                                                );

  auto rightCollisionFreeConstraint = std::make_shared<CollisionFree>(
        rightArmSpace, robot.getRightArm()->getMetaSkeleton(), collisionDetector);
  rightCollisionFreeConstraint->addPairwiseCheck(rightArmGroup, envGroup);
  rightCollisionFreeConstraint->addPairwiseCheck(rightArmGroup, leftArmGroup);
  auto fullCollisionConstraint = robot.getFullCollisionConstraint(rightArmSpace, 
    robot.getRightArm()->getMetaSkeleton(), rightCollisionFreeConstraint);

  /// Planner Setup
  ROS_INFO("The environment has been setup. Press key to start planning");

  rightArm->setPositions(rightRelaxedHome);
  leftArm->setPositions(leftRelaxedHome);
  robot.getRightHand()->executePreshape("closed").wait();

  // auto planner = OMPLConfigurationToConfigurationPlanner<LRAstar::LRAstar>(rightArmSpace, nullptr);

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
  space->setLongestValidSegmentFraction(0.01 / space->getMaximumExtent());
  space->setup();

  // Space Information
  ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
  std::function<bool(const ompl::base::State*)> isStateValid = std::bind(
        isPointValid, rightArmSpace, rightCollisionFreeConstraint, std::placeholders::_1);
  si->setStateValidityChecker(isStateValid);
  si->setup();

  // Problem Definition
  ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
  pdef->addStartState(make_state(space, rightRelaxedHome));
  pdef->setGoalState(make_state(space, rightFarLeftReach));

  // Setup planner
  gls::GLS planner(si);
  planner.setConnectionRadius(1.5);
  planner.setCollisionCheckResolution(0.1);
  planner.setRoadmap("/home/adityavk/workspaces/lab-ws/src/planning_dataset/herb_priors_hard.graphml");

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
    // Get planner data if required
    auto path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(
      pdef->getSolutionPath());
    std::size_t pathSize = path->getStateCount();
    std::cout << pathSize << std::endl;

    std::vector<ompl::base::State*> states = path->getStates();
    for (int i = 0; i < pathSize; ++i)
    {
      Eigen::VectorXd currentConfig(7);
      double *vals = states[i]->as<ompl::base::RealVectorStateSpace::StateType>()->values;
      currentConfig <<  vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6];
      std::cin.get();
      rightArm->setPositions(currentConfig);
    }

    ROS_INFO("Plan Complete. Press key to smooth.");
    std::cin.get();

    auto untimedTrajectory = std::make_shared<aikido::trajectory::Interpolated>(
         std::move(rightArmSpace), std::move(std::make_shared<aikido::statespace::GeodesicInterpolator>(rightArmSpace)));
    for (std::size_t idx = 0; idx < pathSize; ++idx)
    {
     const auto* st = static_cast<aikido::planner::ompl::GeometricStateSpace::StateType*>(path->getState(idx));
     untimedTrajectory->addWaypoint(idx, st->mState);
    }

    auto timedTrajectory = robot.retimePath(rightArm, untimedTrajectory.get());
    ROS_INFO("Press key to execute");
    std::cin.get();
    robot.executeTrajectory(std::move(timedTrajectory)).wait();
  }

  waitForUser("Press enter to exit");
  return 0;
}