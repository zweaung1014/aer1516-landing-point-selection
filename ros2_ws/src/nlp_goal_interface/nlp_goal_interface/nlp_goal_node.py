#!/usr/bin/env python3
"""
NLP Goal Interface Node

Provides an interactive terminal where users type natural language navigation
commands. Uses OpenAI gpt-4o-mini with Structured Outputs to extract (x, y)
coordinates, then publishes a PoseStamped on /goal_pose to trigger the
RRT* → local planner → hopcopter pipeline.
"""

import os
import sys
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from openai import OpenAI
from pydantic import BaseModel


class GoalCoordinate(BaseModel):
    """Structured output schema for extracted goal coordinates."""
    x: float
    y: float


SYSTEM_PROMPT = """\
You are a coordinate extraction assistant for a hopping robot navigation system.
Extract the x and y goal coordinates from the user's natural language command.

The robot operates in a 2D plane (world frame). Valid coordinate ranges:
- x: -2.0 to 8.0 meters
- y: -4.0 to 4.0 meters

Examples of valid commands and expected outputs:
- "go to 2, 0.5" → x=2.0, y=0.5
- "move to position x=3 y=-1" → x=3.0, y=-1.0
- "navigate to (4.5, 2)" → x=4.5, y=2.0
- "fly to 6 meters forward and 1 meter left" → x=6.0, y=1.0
- "head to the point at 1.5, -2.5" → x=1.5, y=-2.5

If the user provides coordinates outside the valid range, clamp them to the
nearest valid boundary. If coordinates truly cannot be determined from the
input, use x=0.0, y=0.0.
"""


class NlpGoalNode(Node):
    def __init__(self):
        super().__init__('nlp_goal_node')

        # Validate API key
        api_key = os.environ.get('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().fatal(
                'OPENAI_API_KEY environment variable is not set. '
                'Export it before running: export OPENAI_API_KEY=sk-...'
            )
            sys.exit(1)

        self.client = OpenAI(api_key=api_key)
        self.model = os.environ.get('NLP_GOAL_MODEL', 'gpt-4o-mini')

        # Publisher
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.get_logger().info(
            f'NLP Goal Interface ready (model={self.model}). '
            f'Type navigation commands or "quit" to exit.'
        )

        # Start input loop in a daemon thread
        self._input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self._input_thread.start()

    def _input_loop(self):
        """Interactive loop reading commands from stdin."""
        try:
            while rclpy.ok():
                try:
                    user_input = input('\n[NLP Goal] Enter command: ')
                except EOFError:
                    break

                user_input = user_input.strip()
                if not user_input:
                    continue
                if user_input.lower() in ('quit', 'exit', 'q'):
                    self.get_logger().info('Shutting down NLP Goal Interface.')
                    rclpy.shutdown()
                    break

                self._process_command(user_input)
        except KeyboardInterrupt:
            pass

    def _process_command(self, command: str):
        """Send command to OpenAI, parse coordinates, publish goal."""
        self.get_logger().info(f'Processing: "{command}"')

        try:
            completion = self.client.chat.completions.parse(
                model=self.model,
                messages=[
                    {'role': 'system', 'content': SYSTEM_PROMPT},
                    {'role': 'user', 'content': command},
                ],
                response_format=GoalCoordinate,
            )

            result = completion.choices[0].message

            if result.refusal:
                self.get_logger().warn(f'Model refused: {result.refusal}')
                return

            goal = result.parsed
            if goal is None:
                self.get_logger().warn('Failed to parse coordinates from response.')
                return

            # Clamp to valid ranges
            goal.x = max(-2.0, min(8.0, goal.x))
            goal.y = max(-4.0, min(4.0, goal.y))

            self._publish_goal(goal.x, goal.y)

        except Exception as e:
            self.get_logger().error(f'OpenAI API error: {e}')

    def _publish_goal(self, x: float, y: float):
        """Publish PoseStamped to /goal_pose."""
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.3
        msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)
        self.get_logger().info(
            f'Published goal: x={x:.2f}, y={y:.2f}, z=0.30 on /goal_pose'
        )


def main(args=None):
    rclpy.init(args=args)
    node = NlpGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
