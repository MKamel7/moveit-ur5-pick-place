// The pick as a MoveIt Task Constructor graph, planned and reported per stage.
//
//   ros2 launch ur5_pick_place demo_bringup.launch.py gazebo_gui:=false
//   ros2 launch ur5_mtc mtc_pick.launch.py target_x:=0.55 target_y:=0.17
//
// WHAT THIS ANSWERS THAT THE IMPERATIVE PICK DOES NOT
//
// `pick_one` is a sequence of calls, each returning true or false. It works and
// is measured at 92 of 100 in the placement campaign, and it has two limits
// that are properties of its shape rather than its code: it commits to the
// first grasp it computes, and a failure says which CALL failed rather than
// which CONSTRAINT pruned it. The campaign's eight failures were all `lift` or
// `retreat` giving up after three attempts, and finding out why meant reading
// MoveIt's log.
//
// MTC plans the whole sequence as a graph. Several grasp yaws are offered and
// explored rather than one being chosen up front, and every stage reports how
// many solutions reached it and how many were rejected, which turns "motion
// failed" into "eight grasp candidates reached the descent and none survived
// the lift".
//
// WHY C++ AND NOT PYTHON, since the rest of this package is Python
//
// Not preference. MTC's Python bindings cannot construct a PipelinePlanner:
// it takes an rclcpp::Node and neither moveit_py nor the MTC bindings expose
// one. Falling back to interpolation-only planners does not help either, since
// Task::plan then fails with "context argument is null" because MTC needs the
// C++ context for its own introspection. Every upstream MTC demo is C++ for
// this reason.

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ur5_mtc/pick_graph.hpp"

namespace mtc = moveit::task_constructor;

namespace
{

constexpr const char * kGroup = "ur_manipulator";
constexpr const char * kIkFrame = "tool0";
constexpr const char * kObject = "target_object";

geometry_msgs::msg::PoseStamped topDownGrasp(
  double x, double y, double z, double yaw)
{
  // Straight down, then rotated about the approach axis. The same pose the
  // Python `grasp.top_down_grasp` builds, kept in the same convention so a
  // solution here and a solution there are the same grasp.
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  tf2::Quaternion q;
  q.setRPY(M_PI, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(q);
  return pose;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mtc_pick");
  const double target_x = node->declare_parameter("target_x", 0.55);
  const double target_y = node->declare_parameter("target_y", 0.17);
  const double target_z = node->declare_parameter("target_z", 0.25);
  const double place_x = node->declare_parameter("place_x", 0.50);
  const double place_y = node->declare_parameter("place_y", -0.38);
  const double place_z = node->declare_parameter("place_z", 0.20);
  const double approach = node->declare_parameter("approach_height", 0.12);
  // Six candidates cost 438 s of planning in the first run, nearly all of it in
  // the OMPL connect to each one. Three is enough to show whether exploring
  // alternatives helps, and keeps a run inside a couple of minutes.
  const int yaw_count = node->declare_parameter("yaw_count", 2);
  const bool execute = node->declare_parameter("execute", false);

  std::thread spinner([node] {rclcpp::spin(node);});

  mtc::Task task;
  task.stages()->setName("pick");
  task.loadRobotModel(node);
  task.setProperty("group", std::string(kGroup));
  task.setProperty("ik_frame", std::string(kIkFrame));

  auto sampling = std::make_shared<mtc::solvers::PipelinePlanner>(node, "ompl");
  auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();

  auto current = std::make_unique<mtc::stages::CurrentState>("current state");
  task.add(std::move(current));

  // THE PART HAS TO BE IN THE SCENE, and it was not in the first run: the
  // descent rejected all 13 grasp candidates and `attach the part` would have
  // failed on an object that did not exist. The placement benchmark adds this
  // box before every pick; a task graph that skips it is planning a different
  // problem from the one the cell solves.
  auto spawn = std::make_unique<mtc::stages::ModifyPlanningScene>("add the part");
  moveit_msgs::msg::CollisionObject part;
  part.id = kObject;
  part.header.frame_id = "base_link";
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {0.05, 0.05, 0.05};
  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = target_x;
  box_pose.position.y = target_y;
  box_pose.position.z = target_z - 0.025;   // the top was given; the centre is half a cube down
  box_pose.orientation.w = 1.0;
  part.primitives.push_back(box);
  part.primitive_poses.push_back(box_pose);
  part.operation = moveit_msgs::msg::CollisionObject::ADD;
  spawn->addObject(part);
  auto * current_ptr = spawn.get();
  task.add(std::move(spawn));

  task.add(std::make_unique<mtc::stages::Connect>(
      "connect to pre-grasp",
      mtc::stages::Connect::GroupPlannerVector{{kGroup, sampling}}));

  // Several candidate grasps, all offered to IK. This is the difference from
  // pick_one, which computes one grasp and fails on it.
  auto grasps = std::make_unique<mtc::Alternatives>("grasp candidates");
  int index = 0;
  for (double yaw : ur5_mtc::graspYaws(static_cast<std::size_t>(yaw_count))) {
    auto pose = std::make_unique<mtc::stages::GeneratePose>(
      "pose yaw " + std::to_string(index));
    pose->setPose(topDownGrasp(target_x, target_y, target_z + approach, yaw));
    pose->setMonitoredStage(current_ptr);

    auto ik = std::make_unique<mtc::stages::ComputeIK>(
      "IK yaw " + std::to_string(index), std::move(pose));
    ik->setGroup(kGroup);
    geometry_msgs::msg::PoseStamped frame;
    frame.header.frame_id = kIkFrame;
    frame.pose.orientation.w = 1.0;
    ik->setIKFrame(frame);
    ik->setMaxIKSolutions(4);
    // The generator publishes its pose into the interface state; without this
    // the IK stage declares target_pose and never receives one, and MTC throws
    // "Property 'target_pose': undefined" at plan time rather than at init.
    ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
    grasps->insert(std::move(ik));
    ++index;
  }
  task.add(std::move(grasps));

  // The descent ends with the tool at the part, so the two are touching by
  // construction. Permitted before the move rather than after it, or every
  // candidate is rejected for arriving where it was sent.
  auto touch = std::make_unique<mtc::stages::ModifyPlanningScene>("allow touching the part");
  touch->allowCollisions(
    kObject, task.getRobotModel()->getJointModelGroup(kGroup)->getLinkModelNamesWithCollisionGeometry(),
    true);
  task.add(std::move(touch));

  auto descend = std::make_unique<mtc::stages::MoveRelative>("descend", cartesian);
  descend->setGroup(kGroup);
  // The frame that is moved. Without it MoveRelative has no IK frame to drive
  // along the direction and returns nothing at all, which reads as "the
  // descent is impossible" rather than "the stage was not told what to move".
  geometry_msgs::msg::PoseStamped tool;
  tool.header.frame_id = kIkFrame;
  tool.pose.orientation.w = 1.0;
  descend->setIKFrame(tool);
  geometry_msgs::msg::Vector3Stamped down;
  down.header.frame_id = "base_link";
  down.vector.z = -1.0;
  descend->setDirection(down);
  // A minimum of 60 % of the approach rejected every candidate. Cartesian
  // descents fail progressively rather than all at once, so the floor is what
  // decides whether a partial descent counts, and the imperative pick has no
  // equivalent floor at all: it plans to the grasp pose and takes what it gets.
  descend->setMinMaxDistance(
    node->declare_parameter("descend_min", 0.02), approach);
  task.add(std::move(descend));

  auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach the part");
  attach->attachObject(kObject, kIkFrame);
  attach->allowCollisions(
    kObject, task.getRobotModel()->getJointModelGroup(kGroup)->getLinkModelNamesWithCollisionGeometry(),
    true);
  // Kept, because the place pose has to be generated against a scene where the
  // part is already in the gripper. Without a monitored stage MTC refuses to
  // initialise, which is the right failure: a place pose computed against the
  // scene as it was before the grasp is a pose for a part that is still on the
  // table.
  auto * attach_ptr = attach.get();
  task.add(std::move(attach));

  auto lift = std::make_unique<mtc::stages::MoveRelative>("lift", cartesian);
  lift->setGroup(kGroup);
  lift->setIKFrame(tool);
  geometry_msgs::msg::Vector3Stamped up;
  up.header.frame_id = "base_link";
  up.vector.z = 1.0;
  lift->setDirection(up);
  lift->setMinMaxDistance(approach * 0.6, approach);
  task.add(std::move(lift));

  task.add(std::make_unique<mtc::stages::Connect>(
      "transfer", mtc::stages::Connect::GroupPlannerVector{{kGroup, sampling}}));

  auto place_pose = std::make_unique<mtc::stages::GeneratePose>("place pose");
  place_pose->setPose(topDownGrasp(place_x, place_y, place_z + approach, 0.0));
  place_pose->setMonitoredStage(attach_ptr);
  auto place_ik = std::make_unique<mtc::stages::ComputeIK>(
    "IK at place", std::move(place_pose));
  place_ik->setGroup(kGroup);
  geometry_msgs::msg::PoseStamped place_frame;
  place_frame.header.frame_id = kIkFrame;
  place_frame.pose.orientation.w = 1.0;
  place_ik->setIKFrame(place_frame);
  place_ik->setMaxIKSolutions(4);
  place_ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
  task.add(std::move(place_ik));

  auto release = std::make_unique<mtc::stages::ModifyPlanningScene>("release the part");
  release->detachObject(kObject, kIkFrame);
  task.add(std::move(release));

  auto retreat = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian);
  retreat->setGroup(kGroup);
  retreat->setIKFrame(tool);
  retreat->setDirection(up);
  retreat->setMinMaxDistance(approach * 0.6, approach);
  task.add(std::move(retreat));

  int status = 0;
  try {
    task.init();
    const auto started = node->now();
    const bool solved = static_cast<bool>(task.plan(1));
    const double seconds = (node->now() - started).seconds();

    std::vector<ur5_mtc::StageReport> reports;
    for (std::size_t i = 0; i < task.stages()->numChildren(); ++i) {
      const auto * stage = (*task.stages())[i];
      reports.push_back(
        {stage->name(), stage->solutions().size(), stage->failures().size()});
    }

    RCLCPP_INFO(
      node->get_logger(), "planned in %.2f s, solved=%s, solutions=%zu",
      seconds, solved ? "true" : "false", task.solutions().size());
    for (const auto & report : reports) {
      RCLCPP_INFO(
        node->get_logger(), "  %-28s %4zu solutions %6zu rejected",
        report.name.c_str(), report.solutions, report.failures);
    }
    const auto * blocking = ur5_mtc::firstBlockingStage(reports);
    if (blocking != nullptr) {
      RCLCPP_WARN(
        node->get_logger(), "first stage to prune everything: %s",
        blocking->name.c_str());
      status = 1;
    } else if (!solved) {
      RCLCPP_WARN(node->get_logger(), "no solution, and no stage pruned everything");
      status = 2;
    }

    if (solved && execute) {
      RCLCPP_INFO(node->get_logger(), "executing the best solution");
      task.execute(*task.solutions().front());
    }
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "the task graph is malformed: " << e);
    status = 3;
  }

  rclcpp::shutdown();
  spinner.join();
  return status;
}
