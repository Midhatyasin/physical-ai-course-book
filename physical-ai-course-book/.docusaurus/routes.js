import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/physical-ai-course-book/__docusaurus/debug',
    component: ComponentCreator('/physical-ai-course-book/__docusaurus/debug', 'fcf'),
    exact: true
  },
  {
    path: '/physical-ai-course-book/__docusaurus/debug/config',
    component: ComponentCreator('/physical-ai-course-book/__docusaurus/debug/config', '218'),
    exact: true
  },
  {
    path: '/physical-ai-course-book/__docusaurus/debug/content',
    component: ComponentCreator('/physical-ai-course-book/__docusaurus/debug/content', 'aa8'),
    exact: true
  },
  {
    path: '/physical-ai-course-book/__docusaurus/debug/globalData',
    component: ComponentCreator('/physical-ai-course-book/__docusaurus/debug/globalData', '80a'),
    exact: true
  },
  {
    path: '/physical-ai-course-book/__docusaurus/debug/metadata',
    component: ComponentCreator('/physical-ai-course-book/__docusaurus/debug/metadata', 'f42'),
    exact: true
  },
  {
    path: '/physical-ai-course-book/__docusaurus/debug/registry',
    component: ComponentCreator('/physical-ai-course-book/__docusaurus/debug/registry', '4b2'),
    exact: true
  },
  {
    path: '/physical-ai-course-book/__docusaurus/debug/routes',
    component: ComponentCreator('/physical-ai-course-book/__docusaurus/debug/routes', '74e'),
    exact: true
  },
  {
    path: '/physical-ai-course-book/profile',
    component: ComponentCreator('/physical-ai-course-book/profile', 'cde'),
    exact: true
  },
  {
    path: '/physical-ai-course-book/',
    component: ComponentCreator('/physical-ai-course-book/', 'b37'),
    routes: [
      {
        path: '/physical-ai-course-book/',
        component: ComponentCreator('/physical-ai-course-book/', '751'),
        routes: [
          {
            path: '/physical-ai-course-book/',
            component: ComponentCreator('/physical-ai-course-book/', '177'),
            routes: [
              {
                path: '/physical-ai-course-book/capstone',
                component: ComponentCreator('/physical-ai-course-book/capstone', 'd14'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course-book/deployment',
                component: ComponentCreator('/physical-ai-course-book/deployment', 'a58'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course-book/intro',
                component: ComponentCreator('/physical-ai-course-book/intro', 'a1b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course-book/module1-ros2',
                component: ComponentCreator('/physical-ai-course-book/module1-ros2', 'af1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course-book/module2-simulation',
                component: ComponentCreator('/physical-ai-course-book/module2-simulation', 'a62'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course-book/module3-isaac',
                component: ComponentCreator('/physical-ai-course-book/module3-isaac', '9ab'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-course-book/module4-vla',
                component: ComponentCreator('/physical-ai-course-book/module4-vla', '5ca'),
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
