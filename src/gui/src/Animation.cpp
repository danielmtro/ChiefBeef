/*
Animation.hpp Implementation

This is the Implementation for the Animation
parent class and all children of the animation class
Written: Daniel Monteiro
*/

#include "Animation.hpp"

Animation::Animation()
    : gen(std::random_device()()), distr(-1, 1)
{
    std::cout << "Creating an animation" << std::endl;
    return;
}

Animation::~Animation()
{
    return;
}

sf::Sprite* Animation::get_sprite()
{
    return &sprite;
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
    sprite.setPosition(-sprite.getGlobalBounds().width, MenuWindow::TROLLEY_START_Y);
}

void Trolley::set_position(float x, float y)
{
    sprite.setPosition(x, y);
}

void Trolley::update_position(sf::Time deltaTime)
{
    // set the trolley position on the frame
    float trolleyX;
    float trolleyY;

    // make it always move towards the left
    trolleyX = sprite.getPosition().x -1* animation_speed * deltaTime.asSeconds()\
    + distr(gen) * MenuWindow::RNG_SCALING;

    // control the trolley moving downwards
    float current_y = sprite.getPosition().y;
    trolleyY = current_y+MenuWindow::VERTICAL_SPEED* animation_speed * deltaTime.asSeconds()\
    + distr(gen) * MenuWindow::RNG_SCALING;

    // If the trolley has moved off-screen to the left, reset its position to the start
    if (trolleyX < 0 || trolleyY > window_height_ + abs(distr(gen)) * MenuWindow::TROLLEY_RELOAD_TIME) {
        trolleyX = MenuWindow::TROLLEY_START_X;
        trolleyY = MenuWindow::TROLLEY_START_Y;
    }

    sprite.setPosition(trolleyX, trolleyY);  // Update position
}