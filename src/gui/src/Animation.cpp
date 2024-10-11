/*
Animation.hpp Implementation

This is the Implementation for the Animation
parent class and all children of the animation class
Written: Daniel Monteiro
*/

#include "Animation.hpp"

Animation::Animation()
{
    std::cout << "Creating an animation" << std::endl;
    return;
}

Animation::~Animation()
{
    return;
}

sf::Sprite& Animation::get_sprite()
{
    return sprite;
}

Trolley::Trolley()
{
    animation_speed = MenuWindow::TROLLEY_SPEED;
}

Trolley::~Trolley()
{
    std::cout << "No more trolley :(" << std::endl;
}

void Trolley::initialise(int width, int height)
{

    // set dimensions
    window_width_ = width;
    window_height_ = height;

    // Load the trolley texture from the file
    std::string trolley_fname = "/Textures/shopping_cart_small.png";
    std::string trolley_texture_path = ament_index_cpp::get_package_share_directory("gui") + trolley_fname;
    if (!sprite_texture.loadFromFile(trolley_texture_path)) {
        std::cerr << "Failed to load trolley texture from " << trolley_texture_path << std::endl;
    }
    std::cout << "Trolley Texture path at " << trolley_texture_path << std::endl;

    sprite.setTextureRect(sf::IntRect(0, 0, sprite_texture.getSize().x, sprite_texture.getSize().y));  // Show full texture

    // Set the texture to the sprite
    sprite.setTexture(sprite_texture);

    // Set the initial position (starting off-screen on the left)
    // -sprite.getGlobalBounds().width
    sprite.setPosition(300, MenuWindow::TROLLEY_START);
}

void Trolley::update_position(sf::Time deltaTime)
{
    // set the trolley position on the frame
    float trolleyX = sprite.getPosition().x + animation_speed * deltaTime.asSeconds();
    // std::cout << "Trolley pos " << trolleyX << std::endl;

    // If the trolley has moved off-screen to the right, reset its position to the left
    if (trolleyX > window_width_) {
        trolleyX = -sprite.getGlobalBounds().width;  // Start off-screen on the left
    }

    sprite.setPosition(trolleyX, sprite.getPosition().y);  // Update position
}