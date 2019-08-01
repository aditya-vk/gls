/* Author: Aditya Vamsikrishna Mandalika */

#ifndef GLS_DATASTRUCTURES_SEARCHQUEUE_HPP_
#define GLS_DATASTRUCTURES_SEARCHQUEUE_HPP_

// STL headers
#include <functional> // std::function
#include <set>        // std::set

// OMPL headers
#include <ompl/base/Cost.h>
#include <ompl/datastructures/BinaryHeap.h>

// GLS headers
#include "gls/datastructures/Types.hpp"
#include "gls/datastructures/Graph.hpp"

namespace gls {
namespace datastructures {

class SearchQueue
{
public:
  /// The function signature of the sorting function for the vertex queue.
  typedef std::function<bool(
      const gls::datastructures::Vertex,
      const gls::datastructures::Vertex)>
      VertexSortingFunction;

  /// The underlying vertex queue.
  typedef std::set<gls::datastructures::Vertex, VertexSortingFunction> VertexQueue;

  /// Constructor.
  SearchQueue();

  /// Destructor.
  virtual ~SearchQueue() = default;

  /// Setup the search queue with the graph information.
  void setup(gls::datastructures::Graph* graph, bool useTotalCost);

  /// Clear the search queue.
  void clear();

  /// Adds vertex to search queue.
  /// \param[in] vertex Vertex to add to the queue.
  void addVertex(gls::datastructures::Vertex vertex);

  /// Pop top vertex.
  gls::datastructures::Vertex popTopVertex();

  /// Get top vertex. Does not remove from the queue.
  gls::datastructures::Vertex getTopVertex();

  /// Get top vertex value.
  double getTopVertexValue();

  /// Remove vertex from search queue.
  /// \param[in] vertex Vertex to remove from the queue.
  void removeVertex(
      const gls::datastructures::Vertex vertex);

  /// Returns true if queue is empty.
  bool isEmpty();

  /// Returns the size of the queue.
  std::size_t getSize() const;

  /// Returns true if queue has vertex.
  /// \param[in] vertex Vertex to search for in the queue.
  bool hasVertex(
      const gls::datastructures::Vertex vertex);

  void printQueue() const;

private:
  /// Custom comparator used to order vertices.
  bool queueComparison(
      const gls::datastructures::Vertex,
      const gls::datastructures::Vertex) const;

  /// The underlying queue of vertices sorted by VertexQueueSortingFunction.
  VertexQueue mVertexQueue;

  /// Pointer to the graph.
  gls::datastructures::Graph* mGraph;

  /// Boolean to use total cost.
  bool mUseTotalCost;
}; // SearchQueue

} // datastructures
} // gls

#endif // GLS_DATASTRUCTURES_SEARCHQUEUE_HPP_
