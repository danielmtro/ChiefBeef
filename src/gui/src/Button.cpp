/*
Button.hpp Implementation

This is the Implementation for the Button
Written: Daniel Monteiro
Date: 12/10/2024
*/


#include "Button.hpp"

Button::Button(float x,\

               float y,\
               float width,\
               float height,\
               const sf::Color& colour,\
               const std::string& textString,\
               sf::Font& font)
{

    // save variables to the button class
    text_colour_ = sf::Color::White;

    // Set up the button's rectangle shape
    buttonShape.setPosition(x, y);
    buttonShape.setSize(sf::Vector2f(width, height));
    buttonShape.setFillColor(colour);

    // Set up the button's text - defaults to white
    buttonText.setFont(font);
    buttonText.setString(textString);
    buttonText.setCharacterSize(30);
    buttonText.setFillColor(text_colour_);

    // Center the text inside the button
    sf::FloatRect textRect = buttonText.getLocalBounds();
    buttonText.setOrigin(textRect.left + textRect.width / 2.0f, textRect.top + textRect.height / 2.0f);
    buttonText.setPosition(x + width / 2.0f, y + height / 2.0f);
}

Button::~Button()
{
    return;
}

void Button::draw(sf::RenderWindow& window)
{
    window.draw(buttonShape);
    window.draw(buttonText);
}

bool Button::buttonClicked(sf::Vector2i mousePos)
{
    return buttonShape.getGlobalBounds().contains(static_cast<sf::Vector2f>(mousePos));
}

