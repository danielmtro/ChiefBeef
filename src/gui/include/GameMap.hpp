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
#include <SFML/Graphics.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "Map.hpp"
#include "Window.hpp"
#include "Button.hpp"

class GameMap : public Window {

    public:

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

        // The ROS2 node that will read from /map and store relevant map data
        std::shared_ptr<Map> map_;
        Button* slam_request_button_;
        

};

#endif