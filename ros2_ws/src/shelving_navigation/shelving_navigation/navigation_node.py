"""Expose the shelving navigation action to the task manager.

The node owns the ROS action server. Navigation execution is delegated to
``NavigationController`` so ROS request handling stays separate from control
flow.
"""

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from shelving_interfaces.action import NavigateToTarget

from .navigation_controller import NavigationController


class NavigationNode(Node):
    """Validate shelving navigation requests and delegate their execution."""

    def __init__(self) -> None:
        """Create the shelving action server and navigation controller."""
        super().__init__('navigation_node')

        self.declare_parameter('action_name', '/navigate_to_target')
        self.declare_parameter('nav2_action_name', '/navigate_to_pose')
        self.declare_parameter(
            'nav2_route_action_name',
            '/navigate_through_poses',
        )
        self.declare_parameter('nav2_server_timeout_sec', 10.0)
        self.declare_parameter('approach_radius', 0.25)
        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('yaw_tolerance', 0.08)
        self.declare_parameter('fine_alignment_linear_speed', 0.08)
        self.declare_parameter('fine_alignment_angular_speed', 0.15)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')

        self._action_name = str(
            self.get_parameter('action_name').value
        )
        self._callback_group = ReentrantCallbackGroup()

        self._controller = NavigationController(
            node=self,
            callback_group=self._callback_group,
            nav2_action_name=str(
                self.get_parameter('nav2_action_name').value
            ),
            nav2_route_action_name=str(
                self.get_parameter('nav2_route_action_name').value
            ),
            nav2_server_timeout_sec=float(
                self.get_parameter('nav2_server_timeout_sec').value
            ),
            approach_radius=float(
                self.get_parameter('approach_radius').value
            ),
            position_tolerance=float(
                self.get_parameter('position_tolerance').value
            ),
            yaw_tolerance=float(
                self.get_parameter('yaw_tolerance').value
            ),
            fine_alignment_linear_speed=float(
                self.get_parameter(
                    'fine_alignment_linear_speed'
                ).value
            ),
            fine_alignment_angular_speed=float(
                self.get_parameter(
                    'fine_alignment_angular_speed'
                ).value
            ),
            cmd_vel_topic=str(
                self.get_parameter('cmd_vel_topic').value
            ),
            global_frame=str(
                self.get_parameter('global_frame').value
            ),
            robot_base_frame=str(
                self.get_parameter('robot_base_frame').value
            ),
        )

        self._action_server = ActionServer(
            self,
            NavigateToTarget,
            self._action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            'Navigation node is ready. '
            f'action={self._action_name}, '
            'navigate_to_pose='
            f'{self.get_parameter("nav2_action_name").value}, '
            'navigate_through_poses='
            f'{self.get_parameter("nav2_route_action_name").value}'
        )

    def _goal_callback(
        self,
        goal_request: NavigateToTarget.Goal,
    ) -> GoalResponse:
        """Validate an incoming shelving navigation request."""
        if self._controller.is_busy:
            self.get_logger().warning(
                'Rejected navigation goal: another goal is already active.'
            )
            return GoalResponse.REJECT

        if not goal_request.job_id.strip():
            self.get_logger().warning(
                'Rejected navigation goal: empty job_id.'
            )
            return GoalResponse.REJECT

        if not goal_request.target_id.strip():
            self.get_logger().warning(
                'Rejected navigation goal: empty target_id.'
            )
            return GoalResponse.REJECT

        target_frame = goal_request.target_pose.header.frame_id
        if not target_frame:
            self.get_logger().warning(
                'Rejected navigation goal: '
                'target_pose.header.frame_id is empty.'
            )
            return GoalResponse.REJECT

        for index, waypoint in enumerate(goal_request.waypoints):
            waypoint_frame = waypoint.header.frame_id
            if not waypoint_frame:
                self.get_logger().warning(
                    'Rejected navigation goal: '
                    f'waypoints[{index}].header.frame_id is empty.'
                )
                return GoalResponse.REJECT

            if waypoint_frame != target_frame:
                self.get_logger().warning(
                    'Rejected navigation goal: '
                    f'waypoints[{index}] uses frame '
                    f"'{waypoint_frame}', but target_pose uses "
                    f"'{target_frame}'."
                )
                return GoalResponse.REJECT

        route_type = 'route' if goal_request.waypoints else 'single target'
        self.get_logger().info(
            'Accepted navigation goal: '
            f'job_id={goal_request.job_id}, '
            f'target_type={goal_request.target_type}, '
            f'target_id={goal_request.target_id}, '
            f'mode={route_type}, '
            f'waypoints={len(goal_request.waypoints)}, '
            f'fine_alignment={goal_request.enable_fine_alignment}'
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        """Accept cancellation of the active navigation request."""
        del goal_handle
        self.get_logger().info('Navigation cancellation requested.')
        return CancelResponse.ACCEPT

    async def _execute_callback(
        self,
        goal_handle,
    ) -> NavigateToTarget.Result:
        """Delegate an accepted request to the controller."""
        return await self._controller.execute(goal_handle)

    def destroy_node(self) -> None:
        """Destroy the action server and controller resources."""
        self._action_server.destroy()
        self._controller.destroy()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run the navigation node."""
    rclpy.init(args=args)

    node: NavigationNode | None = None
    executor: MultiThreadedExecutor | None = None

    try:
        node = NavigationNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None and node is not None:
            executor.remove_node(node)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
