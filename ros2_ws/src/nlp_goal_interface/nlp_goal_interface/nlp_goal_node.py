#!/usr/bin/env python3
"""
NLP Goal Interface Node

Provides an interactive terminal where users type natural language navigation
commands. Uses Anthropic Claude with tool use to extract (x, y) coordinates,
then publishes a PoseStamped on /goal_pose to trigger the
RRT* → local planner → hopcopter pipeline.
"""

import math
import os
import sys
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import anthropic


# ---------------------------------------------------------------------------
# Scene registry — positions sourced from crazysim_default.sdf
# ---------------------------------------------------------------------------
SCENE_OBJECTS = {
    "chair": {"position": (4.0, 2.5),  "display_name": "Chair"},
    "car":   {"position": (4.0, -2.5), "display_name": "Prius Hybrid"},
    "prius": {"position": (4.0, -2.5), "display_name": "Prius Hybrid"},
}

# How far (in metres) to stop from the object's centre
APPROACH_DISTANCE = 1.5


SYSTEM_PROMPT = """\
You are a coordinate extraction assistant for a hopping robot navigation system.
Extract the x and y goal coordinates from the user's natural language command.
Always use the extract_coordinates tool to return the result.

The robot operates in a 2D plane (world frame). Valid coordinate ranges:
- x: -2.0 to 8.0 meters
- y: -4.0 to 4.0 meters

Known scene objects (use approach_object for these):
- "chair" — located at approximately (4.0, 2.5)
- "car" / "prius" — Prius Hybrid, located at approximately (4.0, -2.5)

Rules:
1. If the user asks to APPROACH, GO TO, or FLY TO a named scene object
   (chair, car, prius, etc.), call the approach_object tool with the object name.
2. If the user gives explicit numeric coordinates, call extract_coordinates.
3. Always call one of the two tools — never respond with plain text.

Examples:
- "go to 2, 0.5" → extract_coordinates(x=2.0, y=0.5)
- "approach the car" → approach_object(object_name="car")
- "fly near the chair" → approach_object(object_name="chair")
- "navigate to (4.5, 2)" → extract_coordinates(x=4.5, y=2.0)

If coordinates truly cannot be determined from the input, use x=0.0, y=0.0.
"""

EXTRACT_TOOL = {
    "name": "extract_coordinates",
    "description": (
        "Extract navigation goal coordinates from a natural language command. "
        "Returns the x and y coordinates in meters for the robot to navigate to."
    ),
    "input_schema": {
        "type": "object",
        "properties": {
            "x": {
                "type": "number",
                "description": "Goal x coordinate in meters (valid range: -2.0 to 8.0)",
            },
            "y": {
                "type": "number",
                "description": "Goal y coordinate in meters (valid range: -4.0 to 4.0)",
            },
        },
        "required": ["x", "y"],
    },
}

APPROACH_TOOL = {
    "name": "approach_object",
    "description": (
        "Navigate the robot to a position near a named scene object. "
        "Use this when the user mentions approaching, going to, or flying near "
        "a known object in the scene (e.g. chair, car, prius)."
    ),
    "input_schema": {
        "type": "object",
        "properties": {
            "object_name": {
                "type": "string",
                "enum": ["chair", "car", "prius"],
                "description": "The name of the scene object to approach.",
            },
        },
        "required": ["object_name"],
    },
}


class NlpGoalNode(Node):
    def __init__(self):
        super().__init__('nlp_goal_node')

        # Validate API key
        api_key = os.environ.get('ANTHROPIC_API_KEY')
        if not api_key:
            self.get_logger().fatal(
                'ANTHROPIC_API_KEY environment variable is not set. '
                'Export it before running: export ANTHROPIC_API_KEY=sk-ant-...'
            )
            sys.exit(1)

        self.client = anthropic.Anthropic(api_key=api_key)
        self.model = os.environ.get('NLP_GOAL_MODEL', 'claude-haiku-4-5-20251001')

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
        """Send command to Claude, parse coordinates via tool use, publish goal."""
        self.get_logger().info(f'Processing: "{command}"')

        try:
            response = self.client.messages.create(
                model=self.model,
                max_tokens=256,
                system=SYSTEM_PROMPT,
                messages=[
                    {'role': 'user', 'content': command},
                ],
                tools=[EXTRACT_TOOL, APPROACH_TOOL],
                tool_choice={'type': 'any'},
            )

            # Find the tool_use block in the response
            tool_block = None
            for block in response.content:
                if block.type == 'tool_use':
                    tool_block = block
                    break

            if tool_block is None:
                self.get_logger().warn('No tool_use block in response.')
                return

            if tool_block.name == 'extract_coordinates':
                x = float(tool_block.input['x'])
                y = float(tool_block.input['y'])
                # Clamp to valid ranges
                x = max(-2.0, min(8.0, x))
                y = max(-4.0, min(4.0, y))
                self._publish_goal(x, y)

            elif tool_block.name == 'approach_object':
                object_name = tool_block.input['object_name'].lower()
                result = self._compute_approach_position(object_name)
                if result is None:
                    return
                self._publish_goal(*result)

            else:
                self.get_logger().warn(f'Unknown tool called: {tool_block.name}')

        except Exception as e:
            self.get_logger().error(f'Anthropic API error: {e}')

    def _compute_approach_position(self, object_name: str):
        """Return (x, y) goal APPROACH_DISTANCE metres from the object toward the origin."""
        entry = SCENE_OBJECTS.get(object_name)
        if entry is None:
            self.get_logger().warn(f'Unknown scene object: "{object_name}"')
            return None

        ox, oy = entry['position']
        display = entry['display_name']

        # Unit vector from object toward world origin (0, 0)
        dx, dy = -ox, -oy
        magnitude = math.sqrt(dx * dx + dy * dy)
        if magnitude < 1e-6:
            # Object is at origin; approach from +x
            dx, dy = 1.0, 0.0
        else:
            dx, dy = dx / magnitude, dy / magnitude

        x = ox + APPROACH_DISTANCE * dx
        y = oy + APPROACH_DISTANCE * dy

        # Clamp to valid ranges
        x = max(-2.0, min(8.0, x))
        y = max(-4.0, min(4.0, y))

        self.get_logger().info(
            f'Approaching {display} at ({ox}, {oy}) — goal set to ({x:.2f}, {y:.2f})'
        )
        return x, y

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
