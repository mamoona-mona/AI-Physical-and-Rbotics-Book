import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/',
    component: ComponentCreator('/', '225'),
    routes: [
      {
        path: '/',
        component: ComponentCreator('/', '3b6'),
        routes: [
          {
            path: '/',
            component: ComponentCreator('/', '690'),
            routes: [
              {
                path: '/isaac-sim/chapter-09-isaac-sim-basics',
                component: ComponentCreator('/isaac-sim/chapter-09-isaac-sim-basics', '62f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/isaac-sim/chapter-10-synthetic-data',
                component: ComponentCreator('/isaac-sim/chapter-10-synthetic-data', '1f4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/isaac-sim/chapter-11-vslam',
                component: ComponentCreator('/isaac-sim/chapter-11-vslam', '838'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/isaac-sim/intro',
                component: ComponentCreator('/isaac-sim/intro', 'd3b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physics-simulation/chapter-05-rigid-body-dynamics',
                component: ComponentCreator('/physics-simulation/chapter-05-rigid-body-dynamics', '0e1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physics-simulation/chapter-06-sensor-simulation',
                component: ComponentCreator('/physics-simulation/chapter-06-sensor-simulation', 'ec6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physics-simulation/chapter-07-sim-to-real',
                component: ComponentCreator('/physics-simulation/chapter-07-sim-to-real', '540'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physics-simulation/chapter-08-digital-twin',
                component: ComponentCreator('/physics-simulation/chapter-08-digital-twin', '911'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physics-simulation/intro',
                component: ComponentCreator('/physics-simulation/intro', '1d0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ros2-fundamentals/chapter-01-ros2-architecture',
                component: ComponentCreator('/ros2-fundamentals/chapter-01-ros2-architecture', '899'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ros2-fundamentals/chapter-02-robot-modeling',
                component: ComponentCreator('/ros2-fundamentals/chapter-02-robot-modeling', '295'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ros2-fundamentals/chapter-03-actions-behaviors',
                component: ComponentCreator('/ros2-fundamentals/chapter-03-actions-behaviors', '26f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ros2-fundamentals/chapter-04-gazebo-simulation',
                component: ComponentCreator('/ros2-fundamentals/chapter-04-gazebo-simulation', '9dd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ros2-fundamentals/intro',
                component: ComponentCreator('/ros2-fundamentals/intro', 'ace'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/vla-autonomy/chapter-12-llm-planning',
                component: ComponentCreator('/vla-autonomy/chapter-12-llm-planning', '09e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/vla-autonomy/chapter-13-speech',
                component: ComponentCreator('/vla-autonomy/chapter-13-speech', '821'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/vla-autonomy/chapter-14-vla-models',
                component: ComponentCreator('/vla-autonomy/chapter-14-vla-models', '8d1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/vla-autonomy/chapter-15-capstone',
                component: ComponentCreator('/vla-autonomy/chapter-15-capstone', 'c65'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/vla-autonomy/intro',
                component: ComponentCreator('/vla-autonomy/intro', 'bbb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/',
                component: ComponentCreator('/', 'fc9'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
