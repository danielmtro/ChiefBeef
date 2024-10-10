/*
Window.hpp INTERFACE

THis is the parent class interface for 
all windows.

Written: Daniel Monteiro
*/

#ifndef _WINDOW_HPP
#define _WINDOW_HPP

#include <SFML/Graphics.hpp>
#include <iostream>

class Window 
{

    public: 

        Window(const std::string& name, int width, int height);
        ~Window();

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

        std::string window_name_;
        
        int window_width_;
        int window_height_;
        bool window_active_;


};

#endif