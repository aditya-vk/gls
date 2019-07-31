/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/datastructures/Graph.hpp"

namespace gls {
namespace datastructures {

// ============================================================================
void VertexProperties::updateState(StatePtr state)
{
  mState = state;
}

// ============================================================================
StatePtr VertexProperties::getState()
{
  return mState;
}

// ============================================================================
void VertexProperties::updateCost(double cost)
{
  mCostToCome = cost;
  mTotalCost = mCostToCome + mHeuristic;
}

// ============================================================================
double VertexProperties::getCostToCome()
{
  return mCostToCome;
}

// ============================================================================
void VertexProperties::updateHeuristic(double heuristic)
{
  mHeuristic = heuristic;
  mTotalCost = mCostToCome + mHeuristic;
}

// ============================================================================
double VertexProperties::getHeuristic()
{
  return mHeuristic;
}

// ============================================================================
void VertexProperties::updateParent(Vertex parent)
{
  mParent = parent;
}

// ============================================================================
Vertex VertexProperties::getParent()
{
  return mParent;
}

// ============================================================================
std::set<Vertex>& VertexProperties::getChildren()
{
  return mChildren;
}

// ============================================================================
void VertexProperties::updateChildren(std::set<Vertex> children)
{
  mChildren = children;
}

// ============================================================================
void VertexProperties::addChild(Vertex child)
{
  mChildren.emplace(child);
}

// ============================================================================
void VertexProperties::addChildren(std::set<Vertex> children)
{
  for (auto iterS = children.begin(); iterS != children.end(); ++iterS)
  {
    mChildren.emplace(*iterS);
  }
}

// ============================================================================
void VertexProperties::removeChild(Vertex child)
{
  auto iterS = mChildren.find(child);
  if (iterS != mChildren.end())
    mChildren.erase(iterS);
}

// ============================================================================
void VertexProperties::removeChildren(std::set<Vertex> children)
{
  for (auto iterS = children.begin(); iterS != children.end(); ++iterS)
  {
    auto iterRemove = mChildren.find(*iterS);
    if (iterRemove != mChildren.end())
      mChildren.erase(iterRemove);
  }
}

// ============================================================================
void VertexProperties::removeAllChildren()
{
  mChildren.clear();
}

// ============================================================================
bool VertexProperties::hasChild(Vertex child)
{
  auto iterS = mChildren.find(child);
  if (iterS != mChildren.end())
    return true;
  return false;
}

// ============================================================================
void VertexProperties::updateVisitStatus(VisitStatus status)
{
  mVisitStatus = status;
}

// ============================================================================
VisitStatus VertexProperties::getVisitStatus()
{
  return mVisitStatus;
}

// ============================================================================
void VertexProperties::updateCollisionStatus(CollisionStatus status)
{
  mCollisionStatus = status;
}

// ============================================================================
CollisionStatus VertexProperties::getCollisionStatus()
{
  return mCollisionStatus;
}

// ============================================================================
void EdgeProperties::updateLength(double length)
{
  mLength = length;
}

// ============================================================================
double EdgeProperties::getLength()
{
  return mLength;
}

// ============================================================================
void EdgeProperties::updateEvaluationStatus(EvaluationStatus evaluationStatus)
{
  mEvaluationStatus = evaluationStatus;
}

// ============================================================================
EvaluationStatus EdgeProperties::getEvaluationStatus()
{
  return mEvaluationStatus;
}

// ============================================================================
void EdgeProperties::updateCollisionStatus(CollisionStatus status)
{
  mCollisionStatus = status;
}

// ============================================================================
CollisionStatus EdgeProperties::getCollisionStatus()
{
  return mCollisionStatus;
}

} // datastructures
} // gls
