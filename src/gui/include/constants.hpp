#ifndef _CONSTANTS_HPP
#define _CONSTANTS_HPP

#include <iostream>

// Namespace for defining the dimensions of the map that is spawned
// for the actual turtlebot
namespace GmapWindow
{
    constexpr int MAP_WIDTH = 1200;
    constexpr int MAP_HEIGHT = 600;
    const std::string MAP_NAME = "Supermarket Map";
}


namespace MenuWindow
{
    constexpr int MENU_OPTIONS = 3;
    constexpr int MENU_WIDTH = 1200;
    constexpr int MENU_HEIGHT = 600;
    const std::string MENU_WINDOW_NAME = "Supermarket Menu";

    constexpr int CHARACTER_SIZE = 70;
    constexpr float TROLLEY_SPEED = 100.0f;
    constexpr int TROLLEY_START = 700;
}

#endif