/*
Animation.hpp Interface

This is the Interface for the Animation
parent class and all children of the animation class
Written: Daniel Monteiro
*/

#include <SFML/Graphics.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <iostream>

#include "constants.hpp"

class Animation
{

    public:

        Animation();
        ~Animation();

        /**
        * @brief this function gets a sprite that's associated 
        * with the current animation object
        *
        */
        sf::Sprite& get_sprite();

        /**
        * @brief this function should initialise the texture associated
        * with the sprite. Must be implemented by all derived classes. This
        * function should do the following:
        * 
        * 1. Load a texture from a file
        * 2. Set the texture to the sprite
        * 3. Set the sprites initial position
        * 
        * @param width the width of the screen that the animation will be on
        * @param height the height of the screen that the animation will be on
        */
        virtual void initialise(int width, int height) = 0;

        /**
        * @brief updates the position of a sprite on the screen
        * 
        * @param deltaTime is the time difference between each frame  on the screen
        */
        virtual void update_position(sf::Time deltaTime) = 0;

    protected:

        sf::Sprite sprite;
        sf::Texture sprite_texture;
        float animation_speed;

        int window_width_;
        int window_height_;

};

class Trolley : public Animation
{

    public:
        Trolley();
        ~Trolley();

        /**
        * @brief updates the position of a sprite on the screen
        * 
        * @param deltaTime is the time difference between each frame  on the screen
        */
        void update_position(sf::Time deltaTime);

    public:
        void initialise(int width, int height);
};