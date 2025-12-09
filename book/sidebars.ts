import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: Robotic Nervous System (ROS 2)',
      items: [
        'ros2-basics/communication',
        'ros2-basics/rclpy-control',
        'ros2-basics/urdf-basics',
      ],
    },
    {
      type: 'category',
      label: 'Module 2 — Digital Twin (Gazebo & Unity)',
      items: [
        'digital-twin/quickstart',
        'digital-twin/gazebo-physics-simulation',
        'digital-twin/sensor-simulation-pipeline',
        'digital-twin/unity-interaction-rendering'
      ],
    },
    {
      type: 'category',
      label: 'Module 3 — Isaac AI Brain (NVIDIA Isaac™)',
      items: [
        'isaac-ai-brain/index',
        'isaac-ai-brain/isaac-sim-synthetic-data',
        'isaac-ai-brain/isaac-ros-vslam',
        'isaac-ai-brain/nav2-path-planning',
        'isaac-ai-brain/quickstart'
      ],
    },
    {
      type: 'category',
      label: 'Module 4 — Vision-Language-Action (VLA) Integration',
      items: [
        'vla-integration/index',
        'vla-integration/voice-to-action',
        'vla-integration/cognitive-planning',
        'vla-integration/capstone-vla-pipeline',
        'vla-integration/quickstart'
      ],
    },
    // {
    //   type: 'category',
    //   label: 'Module 5',
    //   items: [
    //     // Add Module 5 chapters here
    //   ],
    // },
  ],
};

export default sidebars;