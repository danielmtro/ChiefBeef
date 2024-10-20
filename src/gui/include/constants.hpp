#ifndef _CONSTANTS_HPP
#define _CONSTANTS_HPP

#include <iostream>

// Namespace for defining the dimensions of the map that is spawned
// for the actual turtlebot

constexpr int DEG_PER_ROT = 360;
constexpr int WHITE_UINT = 255;

enum MeunItems
{
    SHOPPING_TIME = 0,
    MEET_THE_TEAM,
    EXIT
};

namespace GmapWindow
{
    constexpr int MAP_WIDTH = 1200;
    constexpr int MAP_HEIGHT = 800;
    const std::string MAP_NAME = "Supermarket Map";

    // the slam button constants
    constexpr int SBUTTON_X = 50;
    constexpr int SBUTTON_Y = MAP_HEIGHT - 150;
    constexpr int SBUTTON_W = 150;
    constexpr int SBUTTON_H = 100;
    constexpr int SBUTTON_CHAR = 30;

    // the home button constants
    constexpr int HBUTTON_X = MAP_WIDTH - 200;
    constexpr int HBUTTON_Y = MAP_HEIGHT - 150;;
    constexpr int HBUTTON_W = 150;
    constexpr int HBUTTON_H = 100;
    constexpr int HBUTTON_CHAR = 30;

    const std::string SBUTTON_WORD = "Stocktake";
    const std::string HBUTTON_WORD = "Home";

    // constants to control the icons in the gamewindow
    constexpr int ICON_ROTATION_SPEED = 40;
    constexpr int ROT_LIMIT = 5;
    constexpr float TIME_BETWEEN_JIGGLES = 2;
    constexpr float TIME_JIGGLING = 1.5;
    constexpr int NUM_ITEMS = 2;
    constexpr int ICON_Y = 100;
    constexpr int ICON_X = HBUTTON_X + 40;
    constexpr int ICON_SEP = 65;
    constexpr int NUM_ICON_CHARSIZE = 30;
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

namespace Buttons
{
    constexpr int CHARSIZE = 30;
    constexpr int SCALED_CHARSIZE = 32;
    constexpr int HOVER_SCALING = 10;
}

#endif