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
    button_colour_ = colour;
    clicked_colour_ = sf::Color::Green;
    is_active_ = true;

    // dimensions
    width_ = width;
    height_ = height;
    x_ = x;
    y_ = y;

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

void Button::deactivate_button()
{
    is_active_ = false;
}

void Button::activate_button()
{
    is_active_ = true;
}

bool Button::is_active()
{
    return is_active_;
}

void Button::draw(sf::RenderWindow& window)
{
    window.draw(buttonShape);
    window.draw(buttonText);
}

bool Button::buttonHover(sf::Vector2i mousePos)
{
    bool inside = buttonShape.getGlobalBounds().contains(static_cast<sf::Vector2f>(mousePos));

    // early exit if the button isn't active
    if(!is_active_)
    {
        scale_button(x_, y_, width_, height_, Buttons::CHARSIZE);
        buttonShape.setFillColor(sf::Color(160, 160, 160)); // grey colour
        return inside;
    }

    // scale the button
    if(inside)
    {
        // scale button size
        float new_x = x_ - Buttons::HOVER_SCALING/2;
        float new_y = y_ - Buttons::HOVER_SCALING/2;
        float new_width = width_ + Buttons::HOVER_SCALING;
        float new_height = height_ + Buttons::HOVER_SCALING;

        scale_button(new_x, new_y, new_width, new_height, Buttons::SCALED_CHARSIZE);
    }
    else
    {
        // scale button back to normal
        scale_button(x_, y_, width_, height_, Buttons::CHARSIZE);
    }

    // update the colour if its being clicked
    if(inside && sf::Mouse::isButtonPressed(sf::Mouse::Left))
        buttonShape.setFillColor(clicked_colour_);
    else
        buttonShape.setFillColor(button_colour_);

    return inside;
}

void Button::scale_button(float x, float y, float width, float height, int charsize)
{
    // sets the button position size
    buttonShape.setPosition(x, y);
    buttonShape.setSize(sf::Vector2f(width, height));

    // rescale text back to normal
    buttonText.setCharacterSize(charsize);
    sf::FloatRect textRect = buttonText.getLocalBounds();
    buttonText.setOrigin(textRect.left + textRect.width / 2.0f, textRect.top + textRect.height / 2.0f);
    buttonText.setPosition(x + width / 2.0f, y + height / 2.0f);
}