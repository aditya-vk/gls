/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/GLS.hpp"

#include <cmath>     // pow, sqrt
#include <iostream>  // std::invalid_argument
#include <set>       // std::set
#include <assert.h>  // debug

#include <boost/graph/connected_components.hpp> // connected_components

using gls::datastructures::CollisionStatus;
using gls::datastructures::Edge;
using gls::datastructures::EdgeIter;
using gls::datastructures::EdgeProperties;
using gls::datastructures::EPLengthMap;
using gls::datastructures::EvaluationStatus;
using gls::datastructures::Graph;
using gls::datastructures::Path;
using gls::datastructures::NeighborIter;
using gls::datastructures::State;
using gls::datastructures::StatePtr;
using gls::datastructures::Vertex;
using gls::datastructures::VertexIter;
using gls::datastructures::VertexProperties;
using gls::datastructures::VisitStatus;
using gls::datastructures::VPStateMap;
using gls::event::ConstEventPtr;
using gls::selector::ConstSelectorPtr;
using gls::event::EventPtr;
using gls::selector::SelectorPtr;

namespace gls {

GLS::GLS(const ompl::base::SpaceInformationPtr& si, const std::string& name)
  : ompl::base::Planner(si, name)
  , mSpace(si->getStateSpace())
{
}

GLS::~GLS()
{
  // Do nothing.
}

// ============================================================================
void GLS::setup()
{
  // Check if already setup.
  if (static_cast<bool>(ompl::base::Planner::setup_))
    return;

  // Mark the planner to have been setup.
  ompl::base::Planner::setup();

  // Check if the graph has been setup.
  if (!mGraphSetup)
    std::invalid_argument("Graph has not been provided.");

  OMPL_INFORM("%s: Planner has been setup.", ompl::base::Planner::getName().c_str());
}

// ============================================================================
void GLS::setProblemDefinition(const ompl::base::ProblemDefinitionPtr& pdef)
{
  // Make sure we setup the planner first.
  if (!static_cast<bool>(ompl::base::Planner::setup_))
  {
    setup();
  }

  // Mark the planner's problem to be defined.
  ompl::base::Planner::setProblemDefinition(pdef);

  setupPreliminaries();
  OMPL_INFORM("%s: Problem Definition has been setup.", ompl::base::Planner::getName().c_str());
}

// ============================================================================
void GLS::setupPreliminaries()
{
  auto costMap = get(&VertexProperties::mCostToCome, mGraph);
  auto totalCostMap = get(&VertexProperties::mTotalCost, mGraph);

  // TODO (avk): Should I kill these pointers manually?
  StatePtr sourceState(new gls::datastructures::State(mSpace));
  mSpace->copyState(sourceState->getOMPLState(), pdef_->getStartState(0));

  StatePtr targetState(new gls::datastructures::State(mSpace));
  mSpace->copyState(
      targetState->getOMPLState(),
      pdef_->getGoal()->as<ompl::base::GoalState>()->getState());

  // Add start and goal vertices to the graph
  mSourceVertex = boost::add_vertex(mGraph);
  mGraph[mSourceVertex].updateState(sourceState);

  mTargetVertex = boost::add_vertex(mGraph);
  mGraph[mTargetVertex].updateState(targetState);

  // Assign default values.
  costMap[mSourceVertex] = 0;
  mGraph[mSourceVertex].updateHeuristic(getGraphHeuristic(mSourceVertex));
  totalCostMap[mSourceVertex] = costMap[mSourceVertex] + mGraph[mSourceVertex].getHeuristic();
  mGraph[mSourceVertex].updateVisitStatus(VisitStatus::NotVisited);
  mGraph[mSourceVertex].updateCollisionStatus(CollisionStatus::Free);
  mGraph[mSourceVertex].updateParent(mSourceVertex);

  costMap[mTargetVertex] = std::numeric_limits<double>::max();
  mGraph[mSourceVertex].updateHeuristic(0);
  totalCostMap[mTargetVertex] = costMap[mTargetVertex];
  mGraph[mTargetVertex].updateVisitStatus(VisitStatus::NotVisited);
  mGraph[mTargetVertex].updateCollisionStatus(CollisionStatus::Free);

  // TODO (AVK): Make this kNN + R-disc. Additionally join the start and goal.
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi)
  { 
    double sourceDistance = mSpace->distance(
        mGraph[*vi].getState()->getOMPLState(), sourceState->getOMPLState());
    double targetDistance = mSpace->distance(
        mGraph[*vi].getState()->getOMPLState(), targetState->getOMPLState());

    if (sourceDistance < mConnectionRadius)
    {
      if (mSourceVertex == *vi)
        continue;

      std::pair<Edge, bool> newEdge
          = boost::add_edge(mSourceVertex, *vi, mGraph);

      mGraph[newEdge.first].updateLength(sourceDistance);
      mGraph[newEdge.first].updateEvaluationStatus(EvaluationStatus::NotEvaluated);
      mGraph[newEdge.first].updateCollisionStatus(CollisionStatus::Free);
    }

    if (targetDistance < mConnectionRadius)
    {
      if (mTargetVertex == *vi)
        continue;

      std::pair<Edge, bool> newEdge
          = boost::add_edge(mTargetVertex, *vi, mGraph);
      mGraph[newEdge.first].updateLength(targetDistance);
      mGraph[newEdge.first].updateEvaluationStatus(EvaluationStatus::NotEvaluated);
      mGraph[newEdge.first].updateCollisionStatus(CollisionStatus::Free);
    }

    // Additionally, set the heuristic for all the vertices.
    mGraph[*vi].updateHeuristic(getGraphHeuristic(*vi));
  }

  // Additionally connect the source and target with a straight line to snap.
  std::pair<Edge, bool> newEdge
      = boost::add_edge(mSourceVertex, mTargetVertex, mGraph);
  mGraph[newEdge.first].updateLength(
      mSpace->distance(
          sourceState->getOMPLState(), targetState->getOMPLState()));
  mGraph[newEdge.first].updateEvaluationStatus(EvaluationStatus::NotEvaluated);
  mGraph[newEdge.first].updateCollisionStatus(CollisionStatus::Free);

  // Setup the event.
  mEvent->setup(&mGraph, mSourceVertex, mTargetVertex);

  // Setup the selector.
  mSelector->setup(&mGraph);

  // Setup the search queues.
  mExtendQueue.setup(&mGraph, true);
  mUpdateQueue.setup(&mGraph, false);
  mRewireQueue.setup(&mGraph, false);
}

// ============================================================================
void GLS::clear()
{
  auto costMap = get(&VertexProperties::mCostToCome, mGraph);
  auto totalCostMap = get(&VertexProperties::mTotalCost, mGraph);

  // Call the base clear
  ompl::base::Planner::clear();

  // Clear the queues.
  mExtendQueue.clear();
  mRewireQueue.clear();
  mUpdateQueue.clear();
  mRewireSet.clear();

  // Reset the vertices and edges.
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi)
  {
    costMap[*vi] = std::numeric_limits<double>::max();
    totalCostMap[*vi] = std::numeric_limits<double>::max();
    mGraph[*vi].updateHeuristic(std::numeric_limits<double>::max());
    mGraph[*vi].removeAllChildren();
    mGraph[*vi].updateVisitStatus(VisitStatus::NotVisited);
    mGraph[*vi].updateCollisionStatus(CollisionStatus::Free);
  }

  EdgeIter ei, ei_end;
  for (boost::tie(ei, ei_end) = edges(mGraph); ei != ei_end; ++ei)
  {
    mGraph[*ei].updateEvaluationStatus(EvaluationStatus::NotEvaluated);
    mGraph[*ei].updateCollisionStatus(CollisionStatus::Free);
  }

  // Remove edges between source, target to other vertices.
  clear_vertex(mSourceVertex, mGraph);
  clear_vertex(mTargetVertex, mGraph);

  // Remove the vertices themselves.
  remove_vertex(mSourceVertex, mGraph);
  remove_vertex(mTargetVertex, mGraph);

  setBestPathCost(0);
  mNumberOfEdgeEvaluations = 0;
  mNumberOfEdgeRewires = 0;
  mTreeValidityStatus = TreeValidityStatus::Valid;
  mPlannerStatus = PlannerStatus::NotSolved;

  // TODO(avk): Clear the selector and event.

  OMPL_INFORM("%s: Cleared Everything", ompl::base::Planner::getName().c_str());
}

// ============================================================================
ompl::base::PlannerStatus GLS::solve(
    const ompl::base::PlannerTerminationCondition& /*ptc*/)
{
  auto costMap = get(&VertexProperties::mCostToCome, mGraph);
  auto totalCostMap = get(&VertexProperties::mCostToCome, mGraph);

  // Return if source or target are in collision.
  if (evaluateVertex(mSourceVertex) == CollisionStatus::Collision)
  {
    OMPL_INFORM("%s: Start State is invalid.", ompl::base::Planner::getName().c_str());
    return ompl::base::PlannerStatus::INVALID_START;
  }

  if (evaluateVertex(mTargetVertex) == CollisionStatus::Collision)
  {
    OMPL_INFORM("%s: Goal State is invalid.", ompl::base::Planner::getName().c_str());
    return ompl::base::PlannerStatus::INVALID_GOAL;
  }

  // Add the source vertex to the search tree with zero cost-to-come.
  mGraph[mSourceVertex].updateVisitStatus(VisitStatus::Visited);
  mEvent->updateVertexProperties(mSourceVertex);

  mExtendQueue.addVertex(mSourceVertex);

  // Run in loop.
  while (!mExtendQueue.isEmpty())
  {
    // Extend the tree till the event is triggered.
    extendSearchTree();

    // Evaluate the extended search tree.
    evaluateSearchTree();

    // If the plan is successful, return.
    if (mPlannerStatus == PlannerStatus::Solved)
      break;
  }

  if (mPlannerStatus == PlannerStatus::Solved)
  {
    setBestPathCost(costMap[mTargetVertex]);
    pdef_->addSolutionPath(constructSolution(mSourceVertex, mTargetVertex));
    return ompl::base::PlannerStatus::EXACT_SOLUTION;
  }
  else
  {
    OMPL_INFORM("%s: No Solution Found.", ompl::base::Planner::getName().c_str());
    return ompl::base::PlannerStatus::TIMEOUT;
  }
}

// ============================================================================
void GLS::setEvent(EventPtr event)
{
  mEvent = event;
}

// ============================================================================
ConstEventPtr GLS::getEvent() const
{
  return mEvent;
}

// ============================================================================
void GLS::setSelector(SelectorPtr selector)
{
  mSelector = selector;
}

// ============================================================================
ConstSelectorPtr GLS::getSelector() const
{
  return mSelector;
}

// ============================================================================
Edge GLS::getEdge(Vertex u, Vertex v)
{
  Edge uv;
  bool edgeExists;
  boost::tie(uv, edgeExists) = edge(u, v, mGraph);

  return uv;
}

// ============================================================================
Path GLS::getPathToSource(Vertex u)
{
  Path pathToSource;
  while (u != mSourceVertex)
  {
    pathToSource.emplace_back(u);
    u = mGraph[u].getParent();
  }
  pathToSource.emplace_back(mSourceVertex);
  return pathToSource;
}

// ============================================================================
bool GLS::foundPathToGoal()
{
  if (mGraph[mTargetVertex].getVisitStatus() == VisitStatus::NotVisited)
    return false;

  Path pathToGoal = getPathToSource(mTargetVertex);
  for (std::size_t i = 0; i < pathToGoal.size() - 1; ++i)
  {
    Edge e = getEdge(pathToGoal[i+1], pathToGoal[i]);
    if (mGraph[e].getEvaluationStatus() == EvaluationStatus::NotEvaluated)
      return false;
  }

  return true;
}

// ============================================================================
// TODO (avk): I should be able to set the heuristic function from the demo
// script. Create a Heuristic Class and send it in. Have a default heuristic
// if nothing has been set.
double GLS::getGraphHeuristic(Vertex v)
{
  double heuristic = mSpace->distance(
      mGraph[mTargetVertex].getState()->getOMPLState(),
      mGraph[v].getState()->getOMPLState());
  return heuristic;
}

// ============================================================================
void GLS::setConnectionRadius(double radius)
{
  mConnectionRadius = radius;
}

// ============================================================================
double GLS::getConnectionRadius()
{
  return mConnectionRadius;
}

// ============================================================================
void GLS::setRoadmap(std::string filename)
{
  if (filename == "")
    std::invalid_argument("Roadmap Filename cannot be empty!");

  // Load the graph.
  mRoadmap = boost::
      shared_ptr<io::RoadmapFromFile<Graph, VPStateMap, State, EPLengthMap>>(
          new io::RoadmapFromFile<Graph, VPStateMap, State, EPLengthMap>(
              mSpace, filename));

  mRoadmap->generate(
      mGraph,
      get(&VertexProperties::mState, mGraph),
      get(&EdgeProperties::mLength, mGraph));

  // Mark the graph to have been setup.
  mGraphSetup = true;
}

// ============================================================================
void GLS::setBestPathCost(double cost)
{
  mBestPathCost = cost;
}

// ============================================================================
double GLS::getBestPathCost()
{
  return mBestPathCost;
}

// ============================================================================
void GLS::setPlannerStatus(PlannerStatus status)
{
  mPlannerStatus = status;
}

// ============================================================================
PlannerStatus GLS::getPlannerStatus()
{
  return mPlannerStatus;
}

// ============================================================================
double GLS::getNumberOfEdgeEvaluations()
{
  return mNumberOfEdgeEvaluations;
}

// ============================================================================
double GLS::getNumberOfEdgeRewires()
{
  return mNumberOfEdgeRewires;
}

// ============================================================================
CollisionStatus GLS::evaluateVertex(Vertex v)
{
  // Access the validity checker.
  auto validityChecker = si_->getStateValidityChecker();
  
  auto state = mGraph[v].getState()->getOMPLState();

  // Evaluate the state.
  if (!validityChecker->isValid(state))
    return CollisionStatus::Collision;

  return CollisionStatus::Free;
}

// ============================================================================
CollisionStatus GLS::evaluateEdge(const Edge& e)
{
  mNumberOfEdgeEvaluations++;
  std::cout << mNumberOfEdgeEvaluations << " ";

  // Access the validity checker.
  auto validityChecker = si_->getStateValidityChecker();

  // Collision check the start and goal.
  Vertex startVertex = source(e, mGraph);
  Vertex endVertex = target(e, mGraph);

  auto startState = mGraph[startVertex].getState()->getOMPLState();
  auto endState = mGraph[endVertex].getState()->getOMPLState();

  // Evaluate Start and End States.
  if (!validityChecker->isValid(startState))
  {
    mGraph[startVertex].updateCollisionStatus(CollisionStatus::Collision);
    return CollisionStatus::Collision;
  }

  if (si_->checkMotion(startState, endState))
    return CollisionStatus::Free;
  else
    return CollisionStatus::Collision;
}

// ============================================================================
void GLS::extendSearchTree()
{
  auto costMap = get(&VertexProperties::mCostToCome, mGraph);
  auto totalCostMap = get(&VertexProperties::mCostToCome, mGraph);

  while (!mExtendQueue.isEmpty())
  {
    // Check if the popping the top vertex triggers the event.
    Vertex u = mExtendQueue.getTopVertex();
    if (mEvent->isTriggered(u))
      break;

    // Pop the top vertex in the queue to extend the search tree.
    u = mExtendQueue.popTopVertex();

    // If the vertex has been marked to be in collision, do not extend.
    if (mGraph[u].getCollisionStatus() == CollisionStatus::Collision)
      continue;

    // Get the neighbors and extend.
    // TODO (avk): Have a wrapper in case the implicit vs explicit.
    NeighborIter ni, ni_end;
    for (boost::tie(ni, ni_end) = adjacent_vertices(u, mGraph); ni != ni_end;
         ++ni)
    {
      Vertex v = *ni;

      // If the successor has been previously marked to be in collision,
      // continue to the next successor.
      if (mGraph[v].getCollisionStatus() == CollisionStatus::Collision)
        continue;

      // Enforce prevention of loops.
      if (v == mGraph[u].getParent())
        continue;

      // Never come back to the source.
      if (v == mSourceVertex)
        continue;

      // Get the edge between the two vertices.
      Edge uv = getEdge(u, v);

      // If the edge has been previously marked to be in collision,
      // continue to the next successor.
      if (mGraph[uv].getCollisionStatus() == CollisionStatus::Collision)
        continue;

      double edgeLength = mGraph[uv].getLength();
      if (mGraph[v].getVisitStatus() == VisitStatus::NotVisited)
      {
        mGraph[v].updateVisitStatus(VisitStatus::Visited);
      }
      else
      {
        double oldCostToCome = costMap[v];
        double newCostToCome = costMap[u] + edgeLength;

        // Use the parent ID to break ties.
        Vertex previousParent = mGraph[v].getParent();

        // If the previous cost-to-come is lower, continue.
        if (oldCostToCome < newCostToCome)
          continue;

        // Tie-Breaking Rule
        if (oldCostToCome == newCostToCome)
        {
          if (previousParent < u)
            continue;
        }

        // The new cost is lower, make necessary updates.
        // Remove from previous siblings.
        mGraph[previousParent].removeChild(v);

        // Remove the previous version of the vertex from possible queues.
        if (mExtendQueue.hasVertex(v))
          mExtendQueue.removeVertex(v);

        // Cascade the updates to all the descendents.
        // Replace this with getDescendents() function.
        std::vector<Vertex> subtree = {v};
        while (!subtree.empty())
        {
          auto iterT = subtree.rbegin();
          std::set<Vertex>& children = mGraph[*iterT].getChildren();
          subtree.pop_back();

          for (auto iterS = children.begin(); iterS != children.end(); ++iterS)
          {
            mGraph[*iterS].updateVisitStatus(VisitStatus::NotVisited);
            subtree.emplace_back(*iterS);
            if (mExtendQueue.hasVertex(*iterS))
              mExtendQueue.removeVertex(*iterS);
          }
          children.clear();
        }
      }

      // Update the successor's properties.
      mGraph[v].updateParent(u);
      costMap[v] = costMap[u] + edgeLength;
      totalCostMap[v] = costMap[v] + mGraph[v].getHeuristic();

      // Update the vertex property associated with the event.
      mEvent->updateVertexProperties(v);

      // Add it to its new siblings
      mGraph[u].addChild(v);

      // Add the vertex for extension.
      mExtendQueue.addVertex(v);
    }
  }
}

// ============================================================================
void GLS::updateSearchTree()
{
  if (mTreeValidityStatus == TreeValidityStatus::Valid)
  {
    // Update the vertex properties of the entire search tree.
    mEvent->updateVertexProperties(mUpdateQueue);
  }
  else
  {
    // Rewire the search tree.
    rewireSearchTree();

    // With successful rewire, mark the tree to be valid again.
    mTreeValidityStatus = TreeValidityStatus::Valid;
  }
}

// ============================================================================
void GLS::rewireSearchTree()
{
  auto costMap = get(&VertexProperties::mCostToCome, mGraph);
  auto totalCostMap = get(&VertexProperties::mTotalCost, mGraph);

  // 1. Collect all the vertices that need to be rewired.
  while (!mRewireQueue.isEmpty())
  {
    Vertex v = mRewireQueue.popTopVertex();

    // Add all the children of the current node to mRewireQueue.
    auto children = mGraph[v].getChildren();
    for (auto iterS = children.begin(); iterS != children.end(); ++iterS)
      mRewireQueue.addVertex(*iterS);

    // Remove all the children from book-keeping.
    mGraph[v].removeAllChildren();

    // Add the vertex to set.
    mRewireSet.insert(v);

    // Remove from mExtendQueue.
    // TODO (avk): Can this happen?
    if (mExtendQueue.hasVertex(v))
      mExtendQueue.removeVertex(v);

    // Assign default values
    mGraph[v].updateParent(v);
    costMap[v] = std::numeric_limits<double>::max();
    totalCostMap[v] = std::numeric_limits<double>::max();

    // Mark it as not visited
    mGraph[v].updateVisitStatus(VisitStatus::NotVisited);
    mEvent->updateVertexProperties(v);
  }

  mNumberOfEdgeRewires += mRewireSet.size();

  // 2. Assign the nodes keys
  for (auto iterS = mRewireSet.begin(); iterS != mRewireSet.end(); ++iterS)
  {
    Vertex v = *iterS;

    // If the vertex has been marked to be in collision, ignore.
    if (mGraph[v].getCollisionStatus() == CollisionStatus::Collision)
      continue;

    // Look for possible parents in the rest of the graph.
    NeighborIter ni, ni_end;
    for (boost::tie(ni, ni_end) = adjacent_vertices(v, mGraph); ni != ni_end;
         ++ni)
    {
      // Get the possible parent.
      Vertex u = *ni;

      // If the parent has been marked in collision, ignore.
      if (mGraph[u].getCollisionStatus() == CollisionStatus::Collision)
        continue;

      // No point rewiring to the target vertex.
      if (u == mTargetVertex)
        continue;

      // If the neighbor is one of the vertices to be rewires, ignore now.
      if (costMap[u] == std::numeric_limits<double>::max())
        continue;

      // If the neighbor is currently not in search tree, ignore.
      if (mGraph[u].getVisitStatus() == VisitStatus::NotVisited)
        continue;

      // If the parent is gonna trigger the event, ignore.
      if (mEvent->isTriggered(u))
        continue;

      // If the parent is already in mExtendQueue, can be rewired later.
      if (mExtendQueue.hasVertex(u))
        continue;

      Edge uv = getEdge(u, v);
      double edgeLength = mGraph[uv].getLength();

      if (mGraph[uv].getCollisionStatus() == CollisionStatus::Free)
      {
        if (costMap[v] > costMap[u] + edgeLength
            || (costMap[v]
                    == costMap[u] + edgeLength
                && u < mGraph[v].getParent()))
        {
          // Update the vertex.
          costMap[v] = costMap[u] + edgeLength;
          totalCostMap[v] = costMap[v] + mGraph[v].getHeuristic();
          mGraph[v].updateParent(u);

          // Update the vertex property associated with the event.
          mEvent->updateVertexProperties(v);
        }
      }
    }
    mRewireQueue.addVertex(v);
  }

  // 3. Start Rewiring in the cost space
  while (!mRewireQueue.isEmpty())
  {
    Vertex u = mRewireQueue.popTopVertex();

    // Ignore loops.
    if (u == mGraph[u].getParent())
      continue;

    // Since valid parent is found, mark as visited.
    mGraph[u].updateVisitStatus(VisitStatus::Visited);

    // Let the parent know of its new child.
    Vertex p = mGraph[u].getParent();
    mGraph[p].addChild(u);

    mExtendQueue.addVertex(u);

    // Check if u can be a better parent to the vertices being rewired.
    NeighborIter ni, ni_end;
    for (boost::tie(ni, ni_end) = adjacent_vertices(u, mGraph); ni != ni_end;
         ++ni)
    {
      Vertex v = *ni;

      // TODO (avk): Is this necessary?
      if (mGraph[v].getCollisionStatus() == CollisionStatus::Collision)
        continue;

      // Vertex needs to be in set to update.
      if (mRewireSet.find(v) == mRewireSet.end())
        continue;

      Edge uv = getEdge(u, v);
      double edgeLength = mGraph[uv].getLength();

      if (mGraph[uv].getCollisionStatus() == CollisionStatus::Free)
      {
        if (costMap[v] > costMap[u] + edgeLength
            || (costMap[v]
                    == costMap[u] + edgeLength
                && u < mGraph[v].getParent()))
        {
          if (mRewireQueue.hasVertex(v))
            mRewireQueue.removeVertex(v);

          // Update the vertex.
          costMap[v] = costMap[u] + edgeLength;
          totalCostMap[v] = costMap[v] + mGraph[v].getHeuristic();
          mGraph[v].updateParent(u);

          // Update the vertex property associated with the event.
          mEvent->updateVertexProperties(v);
          
          mRewireQueue.addVertex(v);
        }
      }
    }
  }
  mRewireSet.clear();
}

// ============================================================================
void GLS::evaluateSearchTree()
{
  if (mExtendQueue.isEmpty())
    return;

  Vertex bestVertex = mExtendQueue.getTopVertex();
  Edge edgeToEvaluate
      = mSelector->selectEdgeToEvaluate(getPathToSource(bestVertex));

  Vertex u = source(edgeToEvaluate, mGraph);
  Vertex v = target(edgeToEvaluate, mGraph);
  Edge uv = getEdge(u, v);

  // Evaluate the edge.
  mGraph[uv].updateEvaluationStatus(EvaluationStatus::Evaluated);
  if (evaluateEdge(uv) == CollisionStatus::Free)
  {
    // Set the edge collision status.
    mGraph[uv].updateCollisionStatus(CollisionStatus::Free);

    // Populate the queue to update the search tree.
    mUpdateQueue.addVertex(v);
  }
  else
  {
    mGraph[uv].updateCollisionStatus(CollisionStatus::Collision);
    mTreeValidityStatus = TreeValidityStatus::NotValid;

    // Let the old parent know that the child has been removed.
    Vertex previousParent = mGraph[v].getParent();
    mGraph[previousParent].removeChild(v);
   
    // Add to the rewire queue.
    mRewireQueue.addVertex(v);
  }

  if (bestVertex == mTargetVertex && mTreeValidityStatus == TreeValidityStatus::Valid)
  {
    if (foundPathToGoal())
    {
      // Planning problem has been solved!
      mPlannerStatus = PlannerStatus::Solved;
    }
  }

  // Based on the evaluation, update the search tree.
  updateSearchTree();
}

// ============================================================================
ompl::base::PathPtr GLS::constructSolution(
    const Vertex& source, const Vertex& target)
{
  ompl::geometric::PathGeometric* path
      = new ompl::geometric::PathGeometric(si_);
  Vertex v = target;

  while (v != source)
  {
    path->append(mGraph[v].getState()->getOMPLState());
    v = mGraph[v].getParent();
  }

  if (v == source)
  {
    path->append(mGraph[source].getState()->getOMPLState());
  }
  path->reverse();
  return ompl::base::PathPtr(path);
}

} // namespace gls
