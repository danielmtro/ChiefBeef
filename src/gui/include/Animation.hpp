/*
Animation.hpp Interface

This is the Interface for the Animation
parent class and all children of the animation class
Written: Daniel Monteiro
Date: 18/10/2024
*/

#ifndef _ANIMATION_HPP
#define _ANIMATION_HPP

#include <SFML/Graphics.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <iostream>
#include <random>

#include "constants.hpp"
#include "Map.hpp"

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
        sf::Sprite* get_sprite();

        /**
        * @brief this function should initialise the texture associated
        * with the sprite. This function should do the following:
        * 
        * 1. Load a texture from a file
        * 2. Set the texture to the sprite
        * 
        * @param window the window that the sprite will be displayed on
        */
        virtual void initialise(sf::RenderWindow& window) = 0;

        /**
        * @brief updates the position of a sprite on the screen
        * 
        * @param deltaTime is the time difference between each frame  on the screen
        */
        virtual void update_position(sf::Time deltaTime) = 0;

        /**
        * @brief sets the position of a sprite on the screen
        * 
        * @param x horiontal position on screen
        * @param y vertical position on screen
        */
        void set_position(float x, float y);

    protected:

        // rng generation variables
        std::mt19937 gen;
        std::uniform_real_distribution<> distr;

        // animation variables
        sf::Sprite sprite;
        sf::Texture sprite_texture;
        float animation_speed_;

        // window the animation is placed on
        int window_width_;
        int window_height_;

        /**
        * @brief intialises the texture for a given sprite
        * 
        * @param sref reference to the sprite to load for
        * @param tref reference to the texture to load for
        * @param fname the filename of the texture to load
        */
        void initialise_texture(sf::Sprite& sref, sf::Texture& tref, std::string& fname);

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

        /**
        * @brief initialises the trolley's xand y position on screen
        * also loads in the texture associated with a trolley
        * 
        * @param window The window that the trolley will be displayed on
        */
        void initialise(sf::RenderWindow& window);
        
};


/*
This is the class for little animated sprites that will act as the loggers
for the items that are recorded. Items will jiggle every few seconds
*/
class Icon : public Animation
{
    public:

        /**
        * @brief Initialises a menu icon
        * 
        * @param texture_path is the filename of the texture to  be 
        * loaded for this animation (can be a jpg, png, jpeg)
        */
        Icon();
        ~Icon();

        // initialises icon with default image of an apple
        void initialise(sf::RenderWindow& window);
    
        /**
        * @brief this function should initialise the texture associated
        * with the sprite. This function should do the following:
        * 
        * 1. Load a texture from a file
        * 2. Set the texture to the sprite
        * 
        * @param window the window that the sprite will be displayed on
        * @param fname specifies the filename of the sprite texture
        */
        void initialise(sf::RenderWindow& window, std::string fname);

        /**
        * @brief updates the position of a sprite on the screen
        * 
        * @param deltaTime is the time difference between each frame  on the screen
        */
        void update_position(sf::Time deltaTime);

        /**
        * @brief deactivates the icon, setting an indication variable that 
        * this icon should not be displayed on a window
        */
        void deactivate();

        /**
        * @brief activates the icon, setting an indication variable that 
        * this icon should be displayed on a window
        */
        void activate();

        /**
        * @brief gets the variables determining if the icon should be displayed on
        * the window
        */
        bool get_active();

    protected:

        // booleans to control the jiggling motion of the icons
        bool do_i_jiggle_;
        float time_since_last_jiggle_;
        float time_jiggled_;

        // rotation direction 1 for forwards -1 for reverse
        int rotation_direction_;

        bool active_;

};

/*
Class for the character icon that will move around the map.
This wil be displayed as a top view trolley that traverses the map.
*/
class CharacterIcon : public Icon
{

    public:
        
        /**
        * @brief updates the position of a sprite on the screen
        * reads from 
        * 
        * @param pose is the current pose of the turtlebot that should be passed in to this
        * @param scaling_factor the size scaling based on relative size of window to map
        * @param x_offset offset to allow for buttons on the side
        * @param y_offset offset to allow for buttons on top and bottom
        * @param map_meta_data information struct of the map itself
        * 
        */
        void update_position(Map::Pose pose,
                             float scaling_factor, 
                             int x_offset, 
                             int y_offset, 
                             Map::MapMetaData map_meta_data);

};

#endif