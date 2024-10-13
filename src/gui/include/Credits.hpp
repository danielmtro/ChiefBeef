/*
Credits.hpp INTERFACE

This is the interface for the Credits
Written: Daniel Monteiro
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

        std::vector<sf::Sprite> faces;
        std::vector<sf::Texture> face_textures;
};

#endif