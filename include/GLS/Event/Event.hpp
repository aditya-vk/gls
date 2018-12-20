/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_EVENT_EVENT_HPP_
#define GLS_EVENT_EVENT_HPP_

#include <string>   // std::string
#include <utility>  // std::pair
#include <vector>   // std::vector

#include "GLS/Datastructures/Graph.hpp"
#include "GLS/Datastructures/Types.hpp"

namespace gls {
namespace event {

enum vertexUpdateOption
{
  SingleUpdate,
  CascadeUpdate
};

/// Event is a base class to define the trigger to pause search.
/// The rule for switching between serach and edge evaluation is
/// specified by the concrete classes.
class Event
{
public:
  /// Constructor.
  Event();

  /// Destructor.
  virtual ~Event() = default;

  /// Return true if the event is triggered.
  /// \param[in] vertex Vertex that might cause the trigger.
  bool isTriggered(const gls::datastructures::Vertex vertex) const;

  /// Update vertex properties
  /// Concrete classes specify the appropriate update rules.
  /// \param[in] vertex Vertex whose properties need to be updated.
  /// \param[in] cascade Set to true if the update needs to be cascaded downstream.
  virtual void updateVertexProperties(gls::datastructures::Vertex vertex, vertexUpdateOption cascade = vertexUpdateOption::CascadeUpdate) = 0;

protected:
  /// Pointer to the graph.
  gls::datastructures::GraphPtr mGraph;

  /// Source vertex of the graph.
  gls::datastructures::Vertex mSourceVertex;

  /// Target vertex of the graph.
  gls::datastructures::Vertex mTargetVertex;

}; // Event

typedef std::shared_ptr<Event> EventPtr;
typedef std::shared_ptr<const Event> ConstEventPtr;

} // event
} // gls

#endif // GLS_EVENT_EVENT_HPP_
