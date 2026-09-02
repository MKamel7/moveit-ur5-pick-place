// The decisions behind the pick graph, with no ROS and no MTC in them.
//
// The same split the Python side of this repository already uses: the
// classification is the interesting part and it belongs where a test can reach
// it without standing up a simulator. `detection_outcome.py`, `plan_outcome.py`
// and `pick_stages.py` are the same idea on the other side of the language
// boundary.
#ifndef UR5_MTC__PICK_GRAPH_HPP_
#define UR5_MTC__PICK_GRAPH_HPP_

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace ur5_mtc
{

/// Candidate approach yaws, evenly spread over HALF a turn.
///
/// Half rather than a whole one because a parallel gripper's grasp at yaw and
/// at yaw + pi are the same grasp with the jaws swapped. Generating both would
/// double the planning for a set of duplicates, and a report that counts
/// solutions would then claim twice as many as exist.
inline std::vector<double> graspYaws(std::size_t count)
{
  std::vector<double> yaws;
  yaws.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    yaws.push_back(-M_PI / 2.0 + M_PI * static_cast<double>(i) / static_cast<double>(count));
  }
  return yaws;
}

/// How one stage of the task fared.
struct StageReport
{
  std::string name;
  std::size_t solutions{0};
  std::size_t failures{0};

  /// True when this stage received work and let nothing through.
  bool prunedEverything() const {return solutions == 0 && failures > 0;}
};

/// The earliest stage that rejected everything it was given.
///
/// This is the answer the imperative pick cannot give. A task that produced no
/// solution failed somewhere specific, and the useful place to look is the
/// FIRST stage that let nothing through: everything after it was starved rather
/// than broken, so reporting the last empty stage sends a reader to the wrong
/// end of the graph.
inline const StageReport * firstBlockingStage(const std::vector<StageReport> & reports)
{
  for (const auto & report : reports) {
    if (report.prunedEverything()) {
      return &report;
    }
  }
  return nullptr;
}

}  // namespace ur5_mtc

#endif  // UR5_MTC__PICK_GRAPH_HPP_
