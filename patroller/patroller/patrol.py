import math
from time import sleep
from geometry_msgs.msg import PoseStamped, Pose
import rclpy
import json
from rclpy.duration import Duration

from nav2_simple_commander.robot_navigator import (
    BasicNavigator,
    TaskResult,
    NavigateToPose,
)
from threading import Thread

# from nav2_simple_commander.nav2_simple_commander.robot_navigator import (
#     BasicNavigator,
#     TaskResult,
#     NavigateToPose,
# )

# Eventually I want to turn this into a behavior tree where patrolling is just a subtree action.
######################################################################
# !Patrol
# Should be obtained from an external json like "Patrol sheet" or something.
# {
#   Dock: [x y z orientationz orientationw]
#   PatrolPoints:
#   [
#       [x y z orientationz orientationw],
#       [x y z orientationz orientationw],
#                   . . .
#   ]
# }
#######################################################################
# !Patrol behavior
# Navigate to location, rotate in place +90, -180deg, +270, -270
# Between each rotation wait 4-8 seconds
# 4-8 seconds after final rotate, move to next patrol spot.
# Note: This doesn't actually do anything, just looks cute.
#######################################################################


def pose_from_entry(entry):
    pose = Pose()
    pose.position.x = float(entry[0])
    pose.position.y = float(entry[1])
    pose.orientation.z = float(entry[3])
    pose.orientation.w = float(entry[4])
    return pose


class Patroller:
    def __init__(self, dock_pose, patrol_poses):
        self.dock_pose = dock_pose
        self.patrol_poses = patrol_poses
        self.navigator = BasicNavigator()
        self.currently_navigating = False
        self.initalize_navigation()

    def initalize_navigation(self):
        inital_pose = self.pose_stamped_from_pose(self.dock_pose)
        self.navigator.setInitialPose(inital_pose)
        print("Waiting until nav2 is active.")
        self.navigator.waitUntilNav2Active()
        print("Nav2 successfully broughtup.")

    def patrol(self):
        print("Starting patrol")
        print(f"{len(self.patrol_poses)} patrols obtained.")
        # Go through patrols
        for i, patrol_pose in enumerate(self.patrol_poses):
            print(f"Heading to patrol location {i+1}")
            self.attempt_navigate_to_pose(patrol_pose)
            sleep(3)
            # # Patrol behavior (turn in place/examine surroundings?)
            # # Should make this a function that checks task completion.
            # self.navigator.spin(1.57)
            # sleep(5)
            # self.navigator.spin(-1.57 * 2)
            # sleep(5)
            # self.navigator.spin(1.57 * 2.5)
            # sleep(6)  # Theres a chance this messes up next nav goal :C
        # Finished, go to dock
        self.return_to_dock()

    def attempt_navigate_to_pose(self, pose, max_attempts=3, timeout_seconds=30):
        # Retries are required due to https://github.com/ros-planning/navigation2/issues/2272
        # the fix is in galactic + but foxy still has issues. and right now we on foxy.
        fail_count = 0
        destination = self.pose_stamped_from_pose(pose)
        self.currently_navigating = True
        result = None
        while result != TaskResult.SUCCEEDED:
            self.navigator.goToPose(destination)
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                # just in case feedback isn't ready or we accidentally access
                # feedback from a previous command that isn't gotopose (spin)
                if feedback == None or type(feedback) != type(
                    NavigateToPose.Feedback()
                ):
                    continue
                # instead of making this straight seconds, maybe
                # a more robust method is to see how long its stood still
                # by getting goal_distance and navigation_time and only incrementing
                # a separate variable while distance isn't covered.
                if feedback.navigation_time.sec > timeout_seconds:
                    self.navigator.cancelTask()
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print("Navigation task successful.")
                return True
            # elif result == TaskResult.CANCELED:
            #     print("Navigation cancelled. (Timeout)")
            #     fail_count += 1
            else:
                fail_count += 1
                print(
                    f"Navigation unsuccessful, Retrying.. {fail_count}/{max_attempts}"
                )
                if fail_count == max_attempts:
                    print("Navigation unsuccessful.")
                    return False

    def pose_stamped_from_pose(self, pose: Pose):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = pose
        pose_stamped.header.stamp = self.navigator.get_clock().now().to_msg()
        return pose_stamped

    def modify_pose_orientation(self, pose: Pose, theta: float):
        pose.orientation.z = math.sin(theta / 2)
        pose.orientation.w = math.cos(theta / 2)
        return pose

    def return_to_dock(self):
        print("Returning to dock")
        self.attempt_navigate_to_pose(self.dock_pose)


def main(args=None):
    rclpy.init()
    with open(
        "/home/cherry/mk0_ws/src/mk0_ros/mk0_bringup/config/autonomy_config.json", "r"
    ) as f:
        autonomy_config = json.load(f)
    dock_pose = pose_from_entry(autonomy_config["dock"])
    patrol_entries = autonomy_config["patrol_locations"]
    patrol_poses = [pose_from_entry(entry) for entry in patrol_entries]

    patroller = Patroller(dock_pose, patrol_poses)
    patroller.patrol()


if __name__ == "__main__":
    main()
