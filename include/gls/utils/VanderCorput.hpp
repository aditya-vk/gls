/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_UTILS_VANDERCORPUT_HPP_
#define GLS_UTILS_VANDERCORPUT_HPP_

#include <vector>
#include <map>

namespace gls {
namespace utils {

/// Generates a Van Der Corput sequence ordering for states to check along an edge.
class VanderCorput
{
public:
  /// Constructor
  VanderCorput();

  /// Destructor
  ~VanderCorput(void);

  /// Returns a map from integer index to fractional position along edge.
  /// For an edge that has n states, we generate a sequence of n fractional
  /// positions along the edge to check for collision, based on Van Der Corput Sequences
  /// We start at 1/2, then 1/4, and 3/4 and so on upto n states.
  /// \param[in] n The number of states along the edge
  const std::vector<std::pair<int,int>>& get(int n);

private:
  std::map<int, const std::vector< std::pair<int,int>>> mCache;
};

} // namespace utils
} // namespace gls

#endif // GLS_UTILS_VANDERCORPUT_HPP_
