/*
Button.hpp INTERFACE

This is the interface for the Button
Written: Daniel Monteiro
Date: 12/10/2024
*/

#include <SFML/Graphics.hpp>
#include <iostream>
#include "constants.hpp"

class Button {

    public:

        Button(float x,\
               float y,\
               float width,\
               float height,\
               const sf::Color& color,\
               const std::string& textString,\
               sf::Font& font);

        ~Button();

        /**
        * @brief Draws the whole button object on the window
        *
        */
        void draw(sf::RenderWindow& window);

        /**
        * @brief function to check if the mouse is hovering over
        * the button. Will change button properties if it is.
        *
        */
        bool buttonHover(sf::Vector2i mousePos);


        /**
        * @brief Functions to control button activation deactiavtion
        * and current state
        *
        */
        void deactivate_button();
        void activate_button();
        bool is_active();


    private:

        /**
        * @brief function scales the dimension of a button to a given size
        *
        */
        void scale_button(float x, float y, float width, float height, int charsize);

        sf::RectangleShape buttonShape;
        sf::Text buttonText;
        sf::Color text_colour_;
        sf::Color button_colour_;
        sf::Color clicked_colour_;


        // determines if the button remains active
        bool is_active_;

        float width_;
        float height_;
        float x_;
        float y_;

};