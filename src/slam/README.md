# Slam Code
The SLAM code will hold three main components:
- `SLAM`. This will create a map of the local environment 
- `NAV2` - This creates the paths and moves the bot based on a goal.
- `EXPLORE_LITE` - This finds frontiers to explore and sends it back to nav2.

## Compile the package
```bash
colcon build --packages-select tutorial_pkg
```