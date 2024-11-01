/*
Button.hpp INTERFACE

This is the interface for the Button
Written: Daniel Monteiro
Date: 17/10/2024
*/

#include <SFML/Graphics.hpp>
#include <iostream>
#include "constants.hpp"

class Button {

    public:
        /**
        * @brief constructor for the button class.
        * 
        * @param x the x position of the button on the window (in pixels)
        * @param y the y position of the button on the window (in pixels)
        * @param width horizontal width of the button (x direction)
        * @param height vertical height of the button (y direction)
        * @param color the background colour of thebutton
        * @param text_string the word that will be displayed on the button
        * @param font the font that should be used to display the text on the button
        */
        Button(float x,\
               float y,\
               float width,\
               float height,\
               const sf::Color& color,\
               const std::string& text_string,\
               sf::Font& font);

        ~Button();

        /**
        * @brief Draws the whole button object on the window
        *
        * @param window the window that the button will be displayed on
        */
        void draw(sf::RenderWindow& window);

        /**
        * @brief function to check if the mouse is hovering over
        * the button. Will change button properties if it is.
        *
        * @param mousePos a 2D vector representing the current position of the mouse
        */
        bool buttonHover(sf::Vector2i mousePos);


        /**
        * @brief Deactivates the button. This will stop it from interacting when the mouse
        * hovers over it and change the background colour to gray.
        */
        void deactivate_button();

        /**
        * @brief Activates a button again to be interacted with based on mouse clicks and hovers
        */
        void activate_button();

        /**
        * @brief Gets a boolean on whether or not the button is active or not
        */
        bool is_active();


    private:

        /**
        * @brief function scales the dimension of a button to a given size
        *
        * @param x x position of where the button should be placed
        * @param y y position of where the button should be placed
        * @param width horizontal width of the button
        * @param height vertical height of the button
        * @param charsize the character size of the text to be used on the button class
        */
        void scale_button(float x, float y, float width, float height, int charsize);

        // member variables to store properties of the button
        sf::RectangleShape button_shape_;
        sf::Text button_text_;
        sf::Color text_colour_;
        sf::Color button_colour_;
        sf::Color clicked_colour_;

        // determines if the button remains active
        bool is_active_;

        // member variables to store the geometry of the button
        float width_;
        float height_;
        float x_;
        float y_;

};