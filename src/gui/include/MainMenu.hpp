/*
MainMenu.hpp INTERFACE

THis is the interface for the MainMenu
Written: Daniel Monteiro
Date: 12/10/2024
*/

#ifndef _MAINMENU_HPP
#define _MAINMENU_HPP

#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

#include <iostream>
#include "Window.hpp"
#include "Animation.hpp"
#include "constants.hpp"

class MainMenu : public Window{
    
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

        // the text based variables
        int selection_;
        int current_selection_;
        sf::Text menu[MenuWindow::MENU_OPTIONS];
        sf::Color FontColour;
        sf::Color SelectedFontColour;

        // member variables to move around animated trolley
        Trolley trolley_animation;
};

#endif