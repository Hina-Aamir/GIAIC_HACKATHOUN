# Documentation

This directory contains the source files for the Physical AI & Humanoid Robotics book, built with Docusaurus.

## Structure

- `intro.md` - Introduction to the book
- `modules/` - Contains all book modules
  - `module-1-ros2/` - Module 1: The Robotic Nervous System (ROS 2)

## Adding New Content

To add new modules or content:

1. Create the appropriate directory structure in `docs/modules/`
2. Add your markdown files
3. Update `sidebars.js` to include navigation for your new content
4. Add links to your content in appropriate places

## Building the Documentation

To build and serve the documentation locally:

```bash
npm install
npm start
```

This will start a local development server with hot reloading.