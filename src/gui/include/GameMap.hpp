/*
GameMap.hpp INTERFACE

THis is the interface for the GameMap
Written: Daniel Monteiro
Date: 12/10/2024
*/

#ifndef _GAME_MAP_HPP
#define _GAME_MAP_HPP

#include <memory>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <SFML/Graphics.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "Map.hpp"
#include "Window.hpp"
#include "Button.hpp"
#include "Animation.hpp"

/*
GameMap is the main class that is in operation during the programs function
*/
class GameMap : public Window {

    public:

        /**
        * @brief Constructor for the gamemap object this is the main window
        *  
        * @param name name of window to be shows
        * @param width width of window in pixels
        * @param height height of the window in pixels
        * @param MapMtr a shared pointer to a Map object (This is the ROS Node)
        */
        GameMap(const std::string& name, int width, int height, std::shared_ptr<Map> MapPtr);
        ~GameMap();

        /**
        * @brief this function runs the complete window functionality
        * used to visualise the map
        *
        */
        void RunMap();

        /**
        * @brief this function draws a map on the window
        *
        */
        void DrawMapData(sf::RenderWindow& window);

    private: 

        /**
        * @brief plays a sound byte
        * 
        * @param button_num an integer that selects which sound byte to play depending on
        * what button is being interacted with
        */
        void play_button_sound(int button_num);

        // The ROS2 node that will read from /map and store relevant map data
        std::shared_ptr<Map> ros_map_node_;

        // variables for the buttons on the screen
        Button* slam_request_button_;
        Button* home_button_;
        Button* next_page_button_;
        Button* log_to_csv_button_;

        //containers to store the menu items
        std::vector<std::shared_ptr<Icon>> items_in_store_;
        std::vector<std::shared_ptr<sf::Text>> number_of_items_;

        // what sets of icons and text are we showing?
        int page_num_;

        // updates what the battery should display
        void update_battery_state();
        void initialise_battery_textures();

        // create a sprite for the battery location
        sf::Sprite battery_;
        std::vector<std::shared_ptr<sf::Texture>> battery_textures_;
        sf::Text battery_text_;
        float battery_percentage_;

        // wndow variables
        float border_width_;
        
        /**
        * @brief this function initialises the positions of each
        * item in the store, the numbers and the bounding box 
        *
        */
        void initialise(sf::RenderWindow& window);

        /**
        * @brief this function draws everything that should appear on a frame.
        * Also updates positions of sprites before drawing
        * 
        * @param deltaTime time between frames
        */
        void draw_frame(sf::RenderWindow& window, sf::Time deltaTime);

        // box that surrounds the menu items
        sf::RectangleShape bounding_box_;

        // icon for the robot's position
        std::shared_ptr<CharacterIcon> trolley_;

        // map read variables
        uint32_t map_width_;
        uint32_t map_height_;

        // scaling factor for map and characters
        float scaling_factor_;
        int x_offset_;
        int y_offset_;
        
        // button sound variables
        sf::SoundBuffer buffer_;
        sf::Sound click_sound_;
};

#endif