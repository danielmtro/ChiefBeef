/*
MainMenu.hpp INTERFACE

THis is the interface for the MainMenu
Written: Daniel Monteiro
*/

#ifndef _MAINMENU_HPP
#define _MAINMENU_HPP

#include <SFML/Graphics.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

#include <iostream>
#include "Window.hpp"
#include "constants.hpp"

class MainMenu : private Window{
    
    public: 

        // Main Menu
        MainMenu(const std::string& name, int width, int height);
        ~MainMenu();

        // Runs the current menu
        void RunMenu();

        // functions to control selecting options inside of the menu
        void up_command();
        void down_command();

        // gets the current selection for the menu
        int get_menu_selection();

    private: 

        int selection_;
        int current_selection_;

        sf::Font font;
        sf::Text menu[MenuWindow::MENU_OPTIONS];


};

#endif