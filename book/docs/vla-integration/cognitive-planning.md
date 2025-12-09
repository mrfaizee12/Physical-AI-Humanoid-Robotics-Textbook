---
sidebar_position: 3
title: "Cognitive Planning with LLMs"
---

# Cognitive Planning with LLMs (Natural Language → ROS 2 Action Graph)

This chapter covers the implementation of cognitive planning using Large Language Models (LLMs) to convert natural language commands into ROS 2 action graphs for humanoid robot execution. You'll learn how to connect LLM outputs with robotic control systems, creating cognitive planning that translates high-level goals into executable action sequences.

import Diagram from '@site/src/components/Diagram/Diagram';
import ReproducibleExample from '@site/src/components/ReproducibleExample/ReproducibleExample';
import Citation from '@site/src/components/Citation/Citation';

<Diagram
  src="/static/diagrams/llm-cognitive-planning-pipeline.svg"
  alt="LLM Cognitive Planning Pipeline"
  caption="Architecture showing LLM-based cognitive planning pipeline from natural language to ROS 2 action graph generation for humanoid robots"
/>

## Introduction to LLM-Based Cognitive Planning

Cognitive planning with LLMs represents the intelligence layer of the Vision-Language-Action pipeline. It takes natural language commands and converts them into structured action plans that humanoid robots can execute. This approach enables:

- **Natural language understanding**: Interpreting human commands in everyday language
- **Task decomposition**: Breaking complex tasks into executable robot behaviors
- **Context awareness**: Adapting plans based on environmental context
- **Flexibility**: Handling novel commands without pre-programmed behaviors

### The Role of LLMs in Humanoid Robotics

Large Language Models provide cognitive capabilities that bridge the gap between human intentions and robotic execution:

1. **Semantic understanding**: Converting natural language to structured intent
2. **Knowledge integration**: Leveraging world knowledge for planning
3. **Reasoning**: Making inferences about appropriate actions
4. **Adaptation**: Adjusting plans based on context and constraints

## LLM Integration for Natural Language Processing

### Setting Up LLM Access

To implement cognitive planning, you need access to an LLM with strong reasoning capabilities:

1. **API access**: Obtain credentials for OpenAI GPT, Anthropic Claude, or similar service
2. **Prompt engineering**: Design prompts that effectively guide the LLM toward action generation
3. **Response parsing**: Extract structured action plans from LLM responses
4. **Validation**: Ensure generated plans are safe and executable

### Humanoid-Specific Planning Considerations

When planning for humanoid robots, consider:

- **Locomotion constraints**: Bipedal movement limitations and stability requirements
- **Manipulation capabilities**: Humanoid-specific manipulation skills and limitations
- **Environmental interaction**: How humanoid robots interact with the world differently
- **Safety constraints**: Humanoid-specific safety requirements and limitations

<ReproducibleExample
  title="LLM-Based Cognitive Planning for Humanoid Robots"
  description="Create a system that converts natural language commands to ROS 2 action graphs for humanoid robot execution."
  prerequisites={`Access to LLM API (OpenAI GPT, Anthropic Claude, or open-source alternative)
ROS 2 (Humble Hawksbill or later) environment
Python 3.8+ with ROS 2 integration
Humanoid robot model with available ROS 2 actions`}
  steps={`1. Configure LLM API access with proper authentication
2. Design a prompt template for action graph generation
3. Process natural language commands through the LLM
4. Parse the LLM response to extract action sequences
5. Validate the action graph for humanoid execution
6. Execute the generated action graph on the humanoid robot`}
  expectedResult="Natural language commands (e.g., 'Navigate to the kitchen and bring me the coffee mug') are converted into valid ROS 2 action graphs that break down the high-level task into executable humanoid robot behaviors."
  code={`import openai
import json
import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatus

# Configure LLM client
client = openai.OpenAI(api_key="your-api-key")

def generate_action_graph(natural_language_command, robot_capabilities):
    """
    Generate a ROS 2 action graph from natural language command
    """
    prompt_template = """
    Convert the following natural language command to a sequence of ROS 2 actions for a humanoid robot.

    Command: "{command}"

    Robot capabilities: {capabilities}

    Return the action sequence as a JSON array with the following format:
    [
        {{
            "action_type": "navigation",  # or "manipulation", "perception", etc.
            "action_name": "move_base",   # ROS 2 action name
            "parameters": {{              # ROS 2 action parameters
                "target_pose": [x, y, theta]
            }},
            "dependencies": []            # Other actions this action depends on
        }}
    ]

    Only return the JSON array, no other text.
    """
    prompt = prompt_template.format(command=natural_language_command, capabilities=robot_capabilities)

    response = client.chat.completions.create(
        model="gpt-4",
        messages=[{"role": "user", "content": prompt}],
        temperature=0.1
    )

    try:
        # Extract JSON from response
        action_json_str = response.choices[0].message.content.strip()

        # Remove any markdown code block markers
        # Create triple backtick string without using quotes to avoid MDX parsing issues
        # Using chr() function to get backtick character (ASCII 96)
        backtick = chr(96)
        marker_start = backtick + backtick + backtick
        if action_json_str.startswith(marker_start):
            newline_pos = action_json_str.find("\\n")
            if newline_pos != -1:
                action_json_str = action_json_str[newline_pos + 2:]
            marker_end = backtick + backtick + backtick
            action_json_str = action_json_str.rsplit(marker_end, 1)[0].strip()

        action_graph = json.loads(action_json_str)
        return action_graph
    except json.JSONDecodeError:
        print("Failed to parse LLM response: " + str(response.choices[0].message.content))
        return None

def validate_action_graph(action_graph, robot_capabilities):
    """
    Validate that the action graph is executable by the humanoid robot
    """
    for action in action_graph:
        action_type = action.get("action_type")
        action_name = action.get("action_name")

        # Check if the robot has the required capability
        if action_name not in robot_capabilities.get(action_type, []):
            print("Invalid action: " + str(action_name) + " not available for " + str(action_type))
            return False

    return True

# Example usage
if __name__ == "__main__":
    robot_caps = {
        "navigation": ["move_base", "nav2_follow_waypoints"],
        "manipulation": ["pick_place", "grasp_object"],
        "perception": ["detect_objects", "recognize_faces"]
    }

    command = "Navigate to the kitchen and bring me the coffee mug"
    action_graph = generate_action_graph(command, robot_caps)

    if action_graph:
        print("Generated action graph:")
        print(json.dumps(action_graph, indent=2))

        if validate_action_graph(action_graph, robot_caps):
            print("Action graph is valid for humanoid execution")
            # Execute the action graph here
        else:
            print("Action graph validation failed")
    else:
        print("Failed to generate action graph")`}
/>

## ROS 2 Action Graph Generation

### Understanding ROS 2 Action Graphs

ROS 2 action graphs represent executable sequences of robot behaviors with proper feedback mechanisms:

- **Actions**: Long-running tasks with feedback and result reporting
- **Goals**: Specific targets for action execution
- **Feedback**: Continuous status updates during execution
- **Results**: Final outcomes upon completion

### Action Graph Structure for Humanoid Robots

Humanoid-specific action graphs should include:

- **Locomotion actions**: Navigation, walking, balancing
- **Manipulation actions**: Grasping, lifting, placing objects
- **Perception actions**: Object detection, localization, mapping
- **Interaction actions**: Human-robot interaction, communication

### Generating Action Sequences

The LLM-based planning process involves:

1. **Command analysis**: Understanding the high-level task from natural language
2. **Task decomposition**: Breaking the task into executable subtasks
3. **Action selection**: Choosing appropriate ROS 2 actions for each subtask
4. **Parameter generation**: Creating proper parameters for each action
5. **Dependency mapping**: Establishing relationships between actions

## Cognitive Planning Guide for Humanoid Tasks

### Designing Effective Prompts

For cognitive planning to work effectively, design prompts that:

- **Specify the robot type**: Clearly indicate this is for humanoid robots
- **Define the action space**: List available ROS 2 actions and their parameters
- **Include constraints**: Specify humanoid-specific limitations
- **Provide examples**: Show examples of proper action graph generation

### Humanoid-Specific Considerations

When planning for humanoid robots, account for:

- **Stability requirements**: Actions that maintain bipedal balance
- **Manipulation constraints**: Reachable workspace and dexterity limitations
- **Locomotion patterns**: Bipedal walking vs. wheeled navigation
- **Safety margins**: Additional safety considerations for humanoid environments

## Acceptance Scenario: Natural Language to Action Graph Generation

The primary acceptance scenario tests natural language processing:

**Given**: A natural language command describing a complex humanoid task
**When**: Student submits it to the LLM-based planning system
**Then**: The system generates a valid ROS 2 action graph with proper task decomposition and execution sequence

This can be verified by:
1. Providing a complex natural language command (e.g., "Go to the kitchen, pick up the red cup, and bring it to the living room")
2. Processing through the LLM planning system
3. Validating that the resulting action graph is executable
4. Confirming that the task decomposition is logical and complete
5. Ensuring the action sequence respects humanoid constraints

## Integration with Voice Recognition and Action Execution

Cognitive planning sits between voice recognition and action execution in the VLA pipeline:

- **Input**: Processed voice commands with recognized intent (from Whisper)
- **Processing**: Translation of intent to executable action graphs (via LLM)
- **Output**: ROS 2 action graphs ready for humanoid robot execution

## Summary

LLM-based cognitive planning provides the intelligent decision-making layer that translates human intentions into robotic actions. By properly implementing LLM integration and action graph generation, you can create sophisticated planning systems that enable humanoid robots to understand and execute complex natural language commands.

## Verification

To verify that your cognitive planning system is properly implemented:

1. Ensure LLM API access is properly configured and authenticated
2. Verify prompt engineering produces consistent action graph outputs
3. Test with various natural language commands for humanoid tasks
4. Validate that generated action graphs are executable by the robot
5. Confirm that humanoid-specific constraints are respected
6. Check that the system handles ambiguous commands gracefully
7. Ensure the planning integrates properly with upstream voice recognition and downstream action execution

The reproducible example provided earlier can be used to validate that all cognitive planning components work together correctly.

## References

<Citation
  id="1"
  authors="J. Wei et al."
  title="Emergent Abilities of Large Language Models"
  journal="Transactions on Machine Learning Research"
  year="2022"
  pages="1-28"
/>

<Citation
  id="2"
  authors="A. Brohan et al."
  title="RT-1: Robotics Transformer for Real-World Control at Scale"
  journal="arXiv preprint arXiv:2202.01197"
  year="2022"
/>

<Citation
  id="3"
  authors="M. Ahn et al."
  title="Do As I Can, Not As I Say: Grounding Language in Robotic Affordances"
  journal="Conference on Robot Learning"
  year="2022"
  pages="957-971"
/>

<Citation
  id="4"
  authors="T. Chen et al."
  title="Language Models as Zero-Shot Planners: Extracting Actionable Knowledge for Embodied Tasks"
  journal="International Conference on Machine Learning"
  year="2022"
  pages="3315-3335"
/>