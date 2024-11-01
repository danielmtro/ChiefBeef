#ifndef _CONSTANTS_HPP
#define _CONSTANTS_HPP

#include <iostream>
#include <cmath>
#include <unordered_map>


// math based variables
constexpr float RAD_TO_DEG = 180/M_PI;
constexpr int DEG_PER_ROT = 360;

// integer corresponding to all white i.e. (WHITE_UINT, WHITE_UINT, WHITE_UINT)
constexpr int WHITE_UINT = 255;

// global problem variable
constexpr int ITEMS_IN_STORE = 11;

// this hashmap maps April Tag codes to the corresponding item (represents a catalogue)
const std::unordered_map<int, std::string> ID_CODES_TO_ITEM = 
{    
    {0, "Chocolate"},
    {1, "Chocolate"},
    {2, "Chocolate"},
    {3, "Chips"},
    {4, "Chips"},
    {5, "Chips"},
    {6, "Chips"},
    {7, "Chips"},
    {8, "Chicken"},
    {9, "Beef"},
    {10, "Chicken"},
    {11, "Beef"},
    {12, "Cheese"},
    {13, "Milk"},
    {14, "Milk"},
    {15, "Milk"},
    {16, "Eggplant"},
    {17, "Carrot"},
    {18, "Eggplant"},
    {19, "Carrot"},
    {20, "Eggplant"},
    {21, "Carrot"},
    {22, "Carrot"},
    // 23 specifically not supported to demonstrate unknown item functionality when scanned
    {24, "Apple"},
    {25, "Apple"},
    {26, "Orange"},
    {27, "Apple"},
    {28, "Orange"},
    {29, "Apple"},
    {30, "Apple"}, 
    {31, "Orange"}
};


// The gui stores menu items in a std::vector. This hashmap maps the index of the list
// to the item in the store
const std::unordered_map<int, std::string> MENU_INDEX_TO_ITEM = {
    {0, "Apple"},
    {1, "Orange"},
    {2, "Eggplant"},
    {3, "Carrot"},
    {4, "Chocolate"},
    {5, "Chips"},
    {6, "Beef"},
    {7, "Chicken"},
    {8, "Cheese"},
    {9, "Milk"},
    {10, "Unknown"},
    {11, "Unknown"} // have two sets of unknowns to fill up every page
};

// Enum for the pixel values returned from a ROS2 occupancy grid
enum PixelValues
{
    UNKNOWN = -1,
    EMPTY,
    FULL
};

// Different options for the menu items in the GUI
enum MeunItems
{
    SHOPPING_TIME = 0,
    MEET_THE_TEAM,
    EXIT
};

// Namespace for variables pertaining to the main window that is running 
// during operation of the program.
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

    // the next page button constants
    constexpr int NBUTTON_X = MAP_WIDTH - 180;
    constexpr int NBUTTON_Y = 214;
    constexpr int NBUTTON_W = 110;
    constexpr int NBUTTON_H = 60;
    constexpr int NBUTTON_CHAR = 20;

    // text for each of the buttons
    const std::string SBUTTON_WORD = "Stocktake";
    const std::string HBUTTON_WORD = "Home";
    const std::string NBUTTON_WORD = "Next";

    // constants to control the icons in the gamewindow
    constexpr int ICON_ROTATION_SPEED = 40;
    constexpr int ROT_LIMIT = 5;
    constexpr float TIME_BETWEEN_JIGGLES = 2;
    constexpr float TIME_JIGGLING = 1.5;
    constexpr int ICON_Y = 100;
    constexpr int ICON_X = HBUTTON_X + 40;
    constexpr int ICON_SEP = 65;
    constexpr int NUM_ICON_CHARSIZE = 30;

    // items numeric control
    // add an extra item to have the unknown symbol
    constexpr int NUM_ITEMS = ITEMS_IN_STORE + 1;
    constexpr int ITEMS_PER_PAGE = 2;
    constexpr int NUM_PAGES = NUM_ITEMS/ITEMS_PER_PAGE;
}

// Namespace for variables pertaining to the menu that is initially brought up
// when the gui is ran
namespace MenuWindow
{
    // window dimensions
    constexpr int MENU_OPTIONS = 3;
    constexpr int MENU_WIDTH = 1200;
    constexpr int MENU_HEIGHT = 600;
    const std::string MENU_WINDOW_NAME = "Supermarket Menu";

    // variables to control the trolley animation that goes across the screen in the main menu
    constexpr int CHARACTER_SIZE = 70;
    constexpr float TROLLEY_SPEED = 100.0f;
    constexpr float VERTICAL_SPEED = 0.2;
    constexpr float RNG_SCALING = 0.1;
    constexpr float TROLLEY_RELOAD_TIME = 100;
    constexpr int TROLLEY_START_Y = 503;
    constexpr int TROLLEY_START_X = 825;
    
    // remember at what pixel the road in the image ends at
    constexpr float ROAD_Y_LOCATION = 560;
}

// Namespace for variables pertaining to the meet the team window
namespace CreditsWindow
{
    // window geometry
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

// namespace for general button properties
namespace Buttons
{
    constexpr int CHARSIZE = 30;
    constexpr int SCALED_CHARSIZE = 32;
    constexpr int HOVER_SCALING = 10;
}

#endif