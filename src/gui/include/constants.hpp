#ifndef _CONSTANTS_HPP
#define _CONSTANTS_HPP

#include <iostream>

// Namespace for defining the dimensions of the map that is spawned
// for the actual turtlebot

constexpr int DEG_PER_ROT = 360;

enum MeunItems
{
    SHOPPING_TIME = 0,
    MEET_THE_TEAM,
    EXIT
};

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
    constexpr float VERTICAL_SPEED = 0.2;
    constexpr float RNG_SCALING = 0.1;
    constexpr float TROLLEY_RELOAD_TIME = 100;

    constexpr int TROLLEY_START_Y = 503;
    constexpr int TROLLEY_START_X = 825;
    constexpr float ROAD_Y_LOCATION = 560;
}

namespace CreditsWindow
{
    constexpr int MAP_WIDTH = 1200;
    constexpr int MAP_HEIGHT = 600;
    const std::string MAP_NAME = "Meet The Team";
    constexpr int NUM_GROUP_MEMBERS = 6;
    constexpr int ROTATION_SPEED = 8;
    
    // face separation variables
    constexpr int X_SEP = 300;
    constexpr int Y_SEP = 300;
    constexpr int BASE_X = 150;
    constexpr int BASE_Y = 50;
    constexpr int ROT_LIMIT = 2;
}

#endif