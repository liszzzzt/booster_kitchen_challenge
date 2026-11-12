from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


class T2MoveItNode(Node):
    def __init__(self, t2):
        super().__init__("t2_moveit")
        self.t2 = t2
        self.callback_group = ReentrantCallbackGroup()

        self.left_arm_server = self.create_arm_server("left")
        self.right_arm_server = self.create_arm_server("right")

    def create_arm_server(self, arm_side):
        return ActionServer(
            self,
            FollowJointTrajectory,
            f"/{arm_side}_arm_controller/follow_joint_trajectory",
            execute_callback=lambda goal_handle: self.execute(arm_side, goal_handle),
            goal_callback=self.accept_goal,
            cancel_callback=self.accept_cancel,
            callback_group=self.callback_group,
        )

    def accept_goal(self, goal_request):
        return GoalResponse.ACCEPT

    def accept_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute(self, arm_side, goal_handle):
        command = {
            "type": "moveit2",
            "arm_side": arm_side,
            "goal_handle": goal_handle,
        }
        try:
            result = self.t2.manipulation(command)
            if result is not False:
                return result

            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "Robot is busy"
            goal_handle.abort()
            return result
        except Exception as error:
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(error)
            goal_handle.abort()
            return result

    def destroy_node(self):
        self.left_arm_server.destroy()
        self.right_arm_server.destroy()
        super().destroy_node()
