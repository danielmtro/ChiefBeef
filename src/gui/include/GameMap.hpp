/*
GameMap.hpp INTERFACE

THis is the interface for the GameMap
Written: Daniel Monteiro
*/

#ifndef _GAME_MAP_HPP
#define _GAME_MAP_HPP

#include <memory>
#include <iostream>
#include <SFML/Graphics.hpp>
#include "Map.hpp"
#include "Window.hpp"

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


    private: 

        // The ROS2 node that will read from /map and store relevant map data
        std::shared_ptr<Map> map_;
        

};

#endif