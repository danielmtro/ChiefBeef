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

/*
Class for the main menu window that first appears when the user runs the 
program.
*/
class MainMenu : public Window{
    
    public: 

        /**
        * @brief Constructor when the menu is first called
        * @param name the name of the window
        * @param width the width of the window
        * @param height the height of the window
        */
        MainMenu(const std::string& name, int width, int height);
        ~MainMenu();

        /**
        * @brief Contains the main loop where the menu frame is updated
        */
        void RunMenu();

        /**
        * @brief method called when user presses an up arrow on the keyboard.
        * Changes the highlighted text selection.
        */
        void up_command();

        /**
        * @brief method called when user presses a down arrow on the keyboard.
        * Changes the highlighted text selection.
        */
        void down_command();

        /**
        * @brief Returns the option that the user has selected i.e. 
        * the shoppingtime, credits or an exit request.
        */
        int get_menu_selection();

    private: 

        /**
        * @brief Draws the current frame
        * 
        * @param window the window that eveyrthing will be displayed on
        */
        void draw_frame(sf::RenderWindow& window);

        /**
        * @brief Plays the current msuic
        */
        void play_sound_selection();

        /**
        * @brief method increases the size of the text based on the current selectino
        */
        void increase_text_size();
        
        // variables to store sounds when menu options are selected
        sf::Sound click_sound_;
        sf::SoundBuffer buffer_;        

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