/*
Window.hpp INTERFACE

THis is the parent class interface for 
all windows.

Written: Daniel Monteiro
Date: 12/10/2024
*/

#ifndef _WINDOW_HPP
#define _WINDOW_HPP

#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>
#include <iostream>


/*
This is the parent class to all windows. Contains all the 
common functionality that windows possess.
*/
class Window 
{

    public: 

        Window(const std::string& name, int width, int height);
        ~Window();
        
        /**
        * @brief returns the current status of the window.
        */
        bool is_window_active() const;

        /**
        * @brief this function marks a window open
        *
        */
        void activate_window();

        /**
        * @brief this function marks a window closed
        *
        */
        void close_window();

    protected: 

        // window geometry and naming properties
        std::string window_name_;
        int window_width_;
        int window_height_;
        bool window_active_;

        // general member variables of windows
        sf::Font font;
        sf::RectangleShape background;
        sf::Texture texture;

        // variables to play the background music
        sf::Music background_music;
};

#endif