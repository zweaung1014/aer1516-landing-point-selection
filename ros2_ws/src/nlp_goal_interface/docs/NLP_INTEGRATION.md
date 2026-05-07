# NLP Goal Interface — Integration Documentation

## Overview

The `nlp_goal_interface` package adds a natural language layer on top of the existing autonomous navigation stack. Instead of manually publishing goal coordinates via `ros2 topic pub`, the user types plain-English navigation commands in a terminal, and an LLM extracts the target coordinates.

**Pipeline (end-to-end):**

```
Terminal (user types NL command)
        │
        ▼
┌────────────────────────────┐
│  nlp_goal_node             │
│  • Anthropic Claude Haiku  │
│  • Tool Use (structured)   │
│  • Publishes PoseStamped   │
└──────────┬─────────────────┘
           │  /goal_pose
           ▼
┌────────────────────────────┐
│  rrt_star_planner          │  Plans collision-free path
│  → /ompl_rrt_star_trajectory│
└──────────┬─────────────────┘
           ▼
┌────────────────────────────┐
│  local_planner             │  Adjusts waypoints for terrain
│  → /local_planner/         │
│    adjusted_waypoint       │
└──────────┬─────────────────┘
           ▼
┌────────────────────────────┐
│  hopcopter                 │  Follows trajectory (hopping)
└────────────────────────────┘
```

## Prerequisites

```bash
pip install anthropic
```

You need an Anthropic API key. Get one at: https://console.anthropic.com/settings/keys

## Setup

1. **Set your API key** (one of these methods):

   ```bash
   # Option A: export in shell
   export ANTHROPIC_API_KEY=sk-ant-...

   # Option B: source a .env file (create from template)
   cp ros2_ws/src/nlp_goal_interface/.env.example ros2_ws/src/nlp_goal_interface/.env
   # Edit .env and paste your key, then:
   set -a && source ros2_ws/src/nlp_goal_interface/.env && set +a
   ```

2. **Build the package:**

   ```bash
   cd ~/CrazySim/ros2_ws
   colcon build --packages-select nlp_goal_interface
   source install/setup.bash
   ```

## Usage

```bash
# Terminal 1: Start the full sim stack (CrazySim, server, bridges, etc.)

# Terminal 2: Run the NLP goal interface
ros2 run nlp_goal_interface nlp_goal_node
```

You'll see an interactive prompt:

```
[NLP Goal] Enter command: go to 3 meters forward and 1 meter to the left
[INFO] Published goal: x=3.00, y=1.00, z=0.30 on /goal_pose
[NLP Goal] Enter command: navigate to position 5, -2
[INFO] Published goal: x=5.00, y=-2.00, z=0.30 on /goal_pose
[NLP Goal] Enter command: quit
```

Type `quit`, `exit`, or `q` to stop. Ctrl+C also works.

## Configuration

| Environment Variable | Default | Description |
|---------------------|---------|-------------|
| `ANTHROPIC_API_KEY` | *(required)* | Your Anthropic API key |
| `NLP_GOAL_MODEL` | `claude-haiku-3-5-20241022` | Model to use for coordinate extraction |

## How It Works

1. **Input**: User types a natural language command in the terminal.

2. **LLM Call**: The node sends the command to Anthropic's Messages API with:
   - A system prompt that instructs the model to extract (x, y) coordinates
   - **Tool use** with `tool_choice={"type": "tool", "name": "extract_coordinates"}` forces Claude to call a tool with a JSON Schema defining `x` and `y` fields — guaranteeing structured output with no regex parsing needed.

3. **Validation**: Coordinates are clamped to the valid map bounds (x: [-2, 8], y: [-4, 4]).

4. **Publish**: A `geometry_msgs/PoseStamped` message is published on `/goal_pose` with:
   - `header.frame_id = "world"`
   - `pose.position.z = 0.3` (matches RRT* path height convention)
   - `pose.orientation.w = 1.0` (identity quaternion)

5. **Downstream**: The existing `rrt_star_planner` node receives the goal via its `/goal_pose` subscription and triggers path planning. No modifications to downstream nodes are required.

## Supported Command Formats

The LLM handles varied phrasings. Examples:

- `"go to 2, 0.5"`
- `"move to position x=3 y=-1"`
- `"navigate to (4.5, 2)"`
- `"fly 6 meters forward and 1 meter left"`
- `"head to the point at 1.5, -2.5"`
- `"go to coordinates 7 and negative 3"`

## Security Notes

- The `.env` file containing your API key is **gitignored** and will not be committed.
- The `.env.example` file (with placeholder values) IS committed as a template.
- API keys are loaded from environment variables, never hardcoded.

## Future Extensions

1. **Object Grounding**: Integrate with a vision/perception pipeline to resolve commands like "go to the red box" or "navigate to the obstacle on the left" into world-frame coordinates.

2. **Multi-turn Context**: Maintain conversation history so the user can say "go a bit further" or "turn left from there".

3. **Voice Input**: Replace terminal input with speech-to-text for hands-free operation.

4. **Semantic Map**: Build a labeled map of the environment so the LLM can resolve references to known locations ("go to the charging station").
