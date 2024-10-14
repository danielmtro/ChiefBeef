/*
Button.hpp INTERFACE

This is the interface for the Button
Written: Daniel Monteiro
*/

#include <SFML/Graphics.hpp>

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

        // Function to draw the button on the window
        void draw(sf::RenderWindow& window);

        // Function to check if the button is clicked
        bool buttonClicked(sf::Vector2i mousePos);


    private:
        sf::RectangleShape buttonShape;
        sf::Text buttonText;
        sf::Color text_colour_;
};