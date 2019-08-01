/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/datastructures/SearchQueue.hpp"

#include <ompl/util/Console.h> // OMPL_INFORM

namespace gls {
namespace datastructures {

using gls::datastructures::Vertex;
using gls::datastructures::VPCostMap;

SearchQueue::SearchQueue()
  : mVertexQueue(
        [this](
            const gls::datastructures::Vertex lhs,
            const gls::datastructures::Vertex rhs) {
          return queueComparison(lhs, rhs);
        })
{
  // Do Nothing.
}

// ============================================================================
void SearchQueue::setup(gls::datastructures::Graph* graph, bool useTotalCost)
{
  mGraph = graph;
  mUseTotalCost = useTotalCost;
}

// ============================================================================
void SearchQueue::clear()
{
  mVertexQueue.clear();
}

// ============================================================================
void SearchQueue::addVertex(Vertex vertex)
{
  mVertexQueue.insert(vertex);
}

// ============================================================================
Vertex SearchQueue::popTopVertex()
{
  Vertex topVertex = *mVertexQueue.begin();
  mVertexQueue.erase(mVertexQueue.begin());

  return topVertex;
}

// ============================================================================
Vertex SearchQueue::getTopVertex()
{
  return *mVertexQueue.begin();
}

// ============================================================================
double SearchQueue::getTopVertexValue()
{
  auto costMap = get(&VertexProperties::mCostToCome, *mGraph);
  if (mUseTotalCost)
    costMap = get(&VertexProperties::mTotalCost, *mGraph);

  return costMap[*mVertexQueue.begin()];
}

// ============================================================================
void SearchQueue::removeVertex(Vertex vertex)
{
  auto iterQ = mVertexQueue.find(vertex);
  if (iterQ != mVertexQueue.end())
    mVertexQueue.erase(iterQ);
}

// ============================================================================
bool SearchQueue::isEmpty()
{
  if (mVertexQueue.empty())
    return true;

  return false;
}

// ============================================================================
std::size_t SearchQueue::getSize() const
{
  return mVertexQueue.size();
}

// ============================================================================
bool SearchQueue::hasVertex(const Vertex vertex)
{
  auto iterQ = mVertexQueue.find(vertex);
  if (iterQ != mVertexQueue.end())
    return true;

  return false;
}

// ============================================================================
bool SearchQueue::queueComparison(
    const gls::datastructures::Vertex left,
    const gls::datastructures::Vertex right) const
{
  auto costMap = get(&VertexProperties::mCostToCome, *mGraph);
  if (mUseTotalCost)
    costMap = get(&VertexProperties::mTotalCost, *mGraph);

  if (costMap[left] < costMap[right])
    return true;
  else if (costMap[left] > costMap[right])
    return false;
  else
  {
    return left < right;
  }
}

// ============================================================================
void SearchQueue::printQueue() const
{
  auto costMap = get(&VertexProperties::mCostToCome, *mGraph);
  if (mUseTotalCost)
    costMap = get(&VertexProperties::mTotalCost, *mGraph);

  std::cout << "--------------------" << std::endl;
  std::cout << "Queue Size: " << mVertexQueue.size() << std::endl;
  std::cout << "--------------------" << std::endl;
  for (auto iterQ = mVertexQueue.begin(); iterQ != mVertexQueue.end(); ++iterQ)
  {
    auto vertex = *iterQ;
    std::cout << "Vertex: " << vertex << " " << "Cost: " << costMap[vertex] << std::endl;
  }
  std::cout << "--------------------" << std::endl;
}

} // datastructures
} // gls
