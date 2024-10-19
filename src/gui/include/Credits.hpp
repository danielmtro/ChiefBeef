/*
Credits.hpp INTERFACE

This is the interface for the Credits
Written: Daniel Monteiro
Date: 12/10/2024
*/

#ifndef _CREDITS_HPP
#define _CREDITS_HPP

#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <iostream>
#include <vector>
#include <memory>

#include "Window.hpp"
#include "Animation.hpp"
#include "constants.hpp"


class Credits : public Window {
    
    public:

        Credits(const std::string& name, int width, int height);
        ~Credits();


        void RunCredits();

    private:

        /**
        * @brief this function rotates the faces in the credits scene
        * 
        * @param deltaTime is the time difference between each frame  on the screen
        */
        void RotateSprites(sf::Time deltaTime);
        
        std::vector<sf::Sprite> faces;
        std::vector<sf::Texture> face_textures;
        int rotation_direction_;
};

#endif