import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Part I — Foundations of Physical AI',
      items: [
        'chapter-1',
        'chapter-2'
      ],
    },
    {
      type: 'category',
      label: 'Part II — The Robotic Nervous System (ROS 2)',
      items: [
        'chapter-3',
        'chapter-4',
        'chapter-5'
      ],
    },
    {
      type: 'category',
      label: 'Part III — Digital Twins & Simulation',
      items: [
        'chapter-6',
        'chapter-7',
        'chapter-8'
      ],
    },
    {
      type: 'category',
      label: 'Part IV — The AI Robot Brain (NVIDIA Isaac)',
      items: [
        'chapter-9',
        'chapter-10',
        'chapter-11'
      ],
    },
    {
      type: 'category',
      label: 'Part V — Vision-Language-Action (VLA)',
      items: [
        'chapter-12',
        'chapter-13',
        'chapter-14'
      ],
    },
    {
      type: 'category',
      label: 'Part VI — Humanoid Systems',
      items: [
        'chapter-15',
        'chapter-16',
        'chapter-17'
      ],
    },
    {
      type: 'category',
      label: 'Part VII — Capstone Project',
      items: [
        'chapter-18',
        'chapter-19'
      ],
    }
  ],
};

export default sidebars;
