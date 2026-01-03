// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    {
      type: 'category',
      label: 'Module 1: Foundations of Physical AI',
      items: [
        'foundations/index',
        'foundations/what-is-physical-ai',
        'foundations/embodied-intelligence',
        'foundations/humanoid-form-factor',
        'foundations/system-overview',
        'foundations/advanced-topics',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: ROS 2 - The Robotic Nervous System',
      items: [
        'ros2/index',
        'ros2/ros2-architecture',
        'ros2/communication-patterns',
        'ros2/distributed-control',
        'ros2/middleware-concepts',
        'ros2/advanced-topics',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Digital Twin - Gazebo & Unity Simulation',
      items: [
        'digital-twin/index',
        'digital-twin/simulation-purpose',
        'digital-twin/gazebo-physics',
        'digital-twin/unity-visualization',
        'digital-twin/sensor-modeling',
        'digital-twin/sim-to-real',
        'digital-twin/advanced-topics',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: NVIDIA Isaac - AI Robot Brain',
      items: [
        'isaac/index',
        'isaac/isaac-overview',
        'isaac/isaac-sim',
        'isaac/isaac-ros',
        'isaac/ai-inference',
        'isaac/ros2-integration',
        'isaac/advanced-topics',
      ],
    },
    {
      type: 'doc',
      id: 'vla/placeholder',
      label: 'Module 5: VLA (Coming in Iteration 3)',
    },
    {
      type: 'doc',
      id: 'capstone/placeholder',
      label: 'Module 6: Capstone (Coming in Iteration 3)',
    },
  ],
};

export default sidebars;
