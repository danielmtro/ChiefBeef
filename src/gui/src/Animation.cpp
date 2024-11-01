/*
Animation.hpp Implementation

This is the Implementation for the Animation
parent class and all children of the animation class
Written: Daniel Monteiro
Date: 18/10/2024
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

void Animation::set_position(float x, float y)
{
    sprite.setPosition(x, y);
}

void Animation::initialise_texture(sf::Sprite& sref, sf::Texture& tref, std::string& fname)
{
    // Load the trolley texture from the file
    std::string tpath = ament_index_cpp::get_package_share_directory("gui") + "/Textures" + fname;
    if (!tref.loadFromFile(tpath)) {
        std::cerr << "Failed to load texture from " << tpath << std::endl;
    }
    std::cout << "Texture loaded from " << tpath << std::endl;

    // set the size of the sprite
    sref.setTextureRect(sf::IntRect(0, 0, tref.getSize().x, tref.getSize().y));  // Show full texture

    // Set the texture to the sprite
    sref.setTexture(tref);
}

Trolley::Trolley()
{
    animation_speed_ = MenuWindow::TROLLEY_SPEED;
}

Trolley::~Trolley()
{
    std::cout << "No more trolley :(" << std::endl;
}

void Trolley::initialise(sf::RenderWindow& window)
{
    // set dimensions
    window_width_ = window.getSize().x;
    window_height_ = window.getSize().y;

    std::string trolley_fname = "/shopping_cart_small.png";
    initialise_texture(sprite, sprite_texture, trolley_fname);

}

void Trolley::update_position(sf::Time deltaTime)
{
    // set the trolley position on the frame
    float trolleyX;
    float trolleyY;

    // make it always move towards the left
    trolleyX = sprite.getPosition().x -1* animation_speed_ * deltaTime.asSeconds()\
    + distr(gen) * MenuWindow::RNG_SCALING;

    // control the trolley moving downwards
    float current_y = sprite.getPosition().y;
    trolleyY = current_y+MenuWindow::VERTICAL_SPEED* animation_speed_ * deltaTime.asSeconds()\
    + distr(gen) * MenuWindow::RNG_SCALING;

    // If the trolley has moved off-screen to the left, reset its position to the start
    if (trolleyX < 0 || trolleyY > window_height_ + abs(distr(gen)) * MenuWindow::TROLLEY_RELOAD_TIME) {
        trolleyX = MenuWindow::TROLLEY_START_X;
        trolleyY = MenuWindow::TROLLEY_START_Y;
    }

    // Update position
    sprite.setPosition(trolleyX, trolleyY);  
}

Icon::Icon()
{
    active_ = true;
    do_i_jiggle_= false;
    time_since_last_jiggle_ = 0;
    rotation_direction_ = 1;
    animation_speed_ = GmapWindow::ICON_ROTATION_SPEED;
}

Icon::~Icon()
{
    std::cout << "Icon no longer active." << std::endl;
}

void Icon::deactivate()
{
    active_ = false;
}

void Icon::activate()
{
    active_ = true;
}

bool Icon::get_active()
{
    return active_;
}

void Icon::update_position(sf::Time deltaTime)
{   

    // control if we initiate a jiggle
    if(!do_i_jiggle_ && time_since_last_jiggle_ > GmapWindow::TIME_BETWEEN_JIGGLES)
    {
        do_i_jiggle_ = true;
        time_since_last_jiggle_ = 0;
        return;
    }
    // stop jiggling if we've reached the allocated jiggle time
    else if(do_i_jiggle_ && time_jiggled_ > GmapWindow::TIME_JIGGLING)
    {
        do_i_jiggle_ = false;
        time_jiggled_ = 0;
        return;
    }

    // reset the rotation if we aren't jiggling
    if(!do_i_jiggle_)
    {
        sprite.setRotation(0);
        time_since_last_jiggle_ = time_since_last_jiggle_ + deltaTime.asSeconds();
        time_jiggled_ = 0;
        return;
    }
    
    time_since_last_jiggle_  = 0;
    time_jiggled_ = time_jiggled_ + deltaTime.asSeconds();

    // decide if we change rotation direction
    float current_rotation = sprite.getRotation();
    if(rotation_direction_ == 1)
    {
        if(current_rotation > GmapWindow::ROT_LIMIT && current_rotation < DEG_PER_ROT/2)
            rotation_direction_ = -1;
    }
    else
    {
        if(current_rotation > DEG_PER_ROT/2 && current_rotation < DEG_PER_ROT - GmapWindow::ROT_LIMIT)
            rotation_direction_ = 1;
    }

    // intiate a rotation
    float rotation_amount = animation_speed_ * deltaTime.asSeconds() * rotation_direction_;
    sprite.rotate(rotation_amount);
}

void Icon::initialise(sf::RenderWindow& window)
{   
    std::string fname = "/apple.png";

    // set dimensions
    window_width_ = window.getSize().x;
    window_height_ = window.getSize().y;
    initialise_texture(sprite, sprite_texture, fname);

    // Set origin to the center of the sprite
    sf::Vector2f center(sprite_texture.getSize().x / 2.0f, sprite_texture.getSize().y / 2.0f);
    sprite.setOrigin(center);
}

void Icon::initialise(sf::RenderWindow& window, std::string fname)
{
    // set dimensions
    window_width_ = window.getSize().x;
    window_height_ = window.getSize().y;
    initialise_texture(sprite, sprite_texture, fname);

    // Set origin to the center of the sprite
    sf::Vector2f center(sprite_texture.getSize().x / 2.0f, sprite_texture.getSize().y / 2.0f);
    sprite.setOrigin(center);
}

void CharacterIcon::update_position(Map::Pose pose,
                                    float scaling_factor,
                                    int x_offset,
                                    int y_offset,
                                    Map::MapMetaData map_meta_data)
{

    // start positions
    float start_x = map_meta_data.width/2.0;
    float start_y = map_meta_data.height/2.0;

    // positions of the trolley on the unaltered map
    float x = pose.x/map_meta_data.resolution + start_x; 
    float y = -pose.y/map_meta_data.resolution + start_y; // reverse y because pixels go pos down
    float yaw = 2*M_PI - pose.yaw;  // correct again

    // default variables if we don't have a valid pose
    if(x == 0 && y == 0)
        return;

    // offsets including the origin in FOR
    float x_off, y_off;

    // add in the offset for the original FOR
    x_off = x_offset + map_meta_data.o_x/map_meta_data.resolution;
    y_off = y_offset + map_meta_data.o_y/map_meta_data.resolution;

    // scale the size of the sprite and set the position and orientation
    sprite.setScale(scaling_factor/6, scaling_factor/6);
    sprite.setPosition(x*scaling_factor + x_off, y*scaling_factor + y_off);
    sprite.setRotation(yaw * RAD_TO_DEG);
}