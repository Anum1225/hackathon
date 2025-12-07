
import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Foundations',
      collapsible: false,
      items: [
        'intro',
        'week-01-02-intro',
        'week-03-05-ros2'
      ],
    },
    {
      type: 'category',
      label: 'Simulation & Dev',
      collapsible: false,
      items: [
        'week-06-07-gazebo',
        'week-08-10-isaac',
        'hardware-lab',
      ],
    },
    {
      type: 'category',
      label: 'Advanced Robotics',
      collapsible: false,
      items: [
        'week-11-12-humanoid',
        'week-13-conversational',
      ],
    },
  ],
};

export default sidebars;
