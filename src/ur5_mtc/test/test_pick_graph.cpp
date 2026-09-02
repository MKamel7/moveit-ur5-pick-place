// Grasp candidates, and reading a task graph's failure back out of it.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "ur5_mtc/pick_graph.hpp"

TEST(GraspYaws, cover_half_a_turn_without_duplicating_a_grasp)
{
  // A parallel gripper's grasp at yaw and yaw + pi is the same grasp.
  const auto yaws = ur5_mtc::graspYaws(6);
  ASSERT_EQ(yaws.size(), 6u);
  for (std::size_t i = 0; i < yaws.size(); ++i) {
    for (std::size_t j = i + 1; j < yaws.size(); ++j) {
      EXPECT_GT(std::abs(std::abs(yaws[i] - yaws[j]) - M_PI), 1e-9);
    }
  }
}

TEST(GraspYaws, a_single_candidate_is_allowed)
{
  EXPECT_EQ(ur5_mtc::graspYaws(1).size(), 1u);
}

TEST(FirstBlockingStage, reports_the_first_stage_that_pruned_everything)
{
  // Later empty stages were starved, not broken, and point the wrong way.
  const std::vector<ur5_mtc::StageReport> reports{
    {"current state", 1, 0},
    {"connect to pre-grasp", 8, 2},
    {"compute IK", 0, 8},
    {"descend", 0, 0},
    {"lift", 0, 0}};

  const auto * blocking = ur5_mtc::firstBlockingStage(reports);
  ASSERT_NE(blocking, nullptr);
  EXPECT_EQ(blocking->name, "compute IK");
}

TEST(FirstBlockingStage, a_stage_that_received_nothing_is_not_the_culprit)
{
  // It never received work. Blaming it hides the stage that starved it.
  const ur5_mtc::StageReport starved{"lift", 0, 0};
  EXPECT_FALSE(starved.prunedEverything());
}

TEST(FirstBlockingStage, a_task_that_solved_has_no_blocking_stage)
{
  const std::vector<ur5_mtc::StageReport> reports{{"compute IK", 4, 12}, {"lift", 4, 0}};
  EXPECT_EQ(ur5_mtc::firstBlockingStage(reports), nullptr);
}
