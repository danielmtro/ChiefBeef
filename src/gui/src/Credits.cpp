/*
Credits.hpp Implementation

This is the implementation for the Credits Page
Written: Daniel Monteiro
Date: 12/10/2024
*/

#include "Credits.hpp"


Credits::Credits(const std::string& name, int width, int height)
    : Window(name, width, height)
{
    std::cout << "Credits opened" << std::endl;

    // set the background for the main menu
    std::string background_location = "/Credits_Background.png";
    std::string texture_path = ament_index_cpp::get_package_share_directory("gui") + "/Textures";
    std::cout << "Texture path " << texture_path << std::endl;

    // load the texture  
    background.setSize(sf::Vector2f(window_width_, window_height_));
    if(!texture.loadFromFile(texture_path + background_location))
        std::cout << "Failed to load background at " << texture_path << std::endl;
    background.setTexture(&texture);

    // background music variables
    std::string music_filename = "/Music/CreditsMusic.ogg";
    std::string music_path = ament_index_cpp::get_package_share_directory("gui") + music_filename;
    if (!background_music.openFromFile(music_path)) {
        std::cerr << "Error loading music file!" << std::endl;
        return;
    }

    // initialise each sprite object for the faces
    for(int i = 0; i < CreditsWindow::NUM_GROUP_MEMBERS; ++i)
    {
        std::shared_ptr<sf::Sprite> sprite = std::make_shared<sf::Sprite>();
        faces_.push_back(*sprite);
    }

    for(int i = 0; i < CreditsWindow::NUM_GROUP_MEMBERS; ++i)
    {
        std::shared_ptr<sf::Texture> face_texture = std::make_shared<sf::Texture>();
        face_textures_.push_back(*face_texture);
    }

    // set the filenames for each face
    std::string f0 = texture_path + "/Daniel_.jpg";
    std::string f1 = texture_path + "/Tumali_.jpg";
    std::string f2 = texture_path + "/Adam_.jpg";
    std::string f3 = texture_path + "/Ridley_.jpg";
    std::string f4 = texture_path + "/Viswada_.jpg";
    std::string f5 = texture_path + "/James_.jpg";

    // load in each face to the relevant texture
    face_textures_[0].loadFromFile(f0);
    face_textures_[1].loadFromFile(f1);
    face_textures_[2].loadFromFile(f2);
    face_textures_[3].loadFromFile(f3);
    face_textures_[4].loadFromFile(f4);
    face_textures_[5].loadFromFile(f5);

    // set each sprite with the corresponding texture
    for(int i = 0; i < CreditsWindow::NUM_GROUP_MEMBERS; ++i)
    {
        faces_[i].setTexture(face_textures_[i]);
    }

    rotation_direction_ = 1;

    // set the positions

    faces_[0].setPosition(CreditsWindow::BASE_X, CreditsWindow::BASE_Y);
    faces_[1].setPosition(CreditsWindow::BASE_X + CreditsWindow::X_SEP, CreditsWindow::BASE_Y);
    faces_[2].setPosition(CreditsWindow::BASE_X + 2 * CreditsWindow::X_SEP, CreditsWindow::BASE_Y);

    faces_[3].setPosition(CreditsWindow::BASE_X, CreditsWindow::BASE_Y + CreditsWindow::Y_SEP);
    faces_[4].setPosition(CreditsWindow::BASE_X + CreditsWindow::X_SEP, CreditsWindow::BASE_Y + CreditsWindow::Y_SEP);
    faces_[5].setPosition(CreditsWindow::BASE_X + 2 * CreditsWindow::X_SEP, CreditsWindow::BASE_Y + CreditsWindow::Y_SEP);

}

Credits::~Credits()
{
    std::cout << "Credits Closed" << std::endl;
}

void Credits::RotateSprites(sf::Time deltaTime)
{
    float rot_limit = 0.3;

    float current_rotation = faces_[0].getRotation();
    // std::cout << "RD" << rotation_direction_ << " rot: " << current_rotation << std::endl;

    // check if its rotating forward
    if(rotation_direction_ == 1)
    {
        if(current_rotation > CreditsWindow::ROT_LIMIT && current_rotation < DEG_PER_ROT/2)
        {
            rotation_direction_ = -1;
        }
    }
    else{
        if(current_rotation > DEG_PER_ROT/2 && current_rotation < DEG_PER_ROT - CreditsWindow::ROT_LIMIT)
        {
            rotation_direction_ = 1;
        }
    }
    

    float rot_amount = rotation_direction_ * CreditsWindow::ROTATION_SPEED * deltaTime.asSeconds();

    for(int i = 0; i < CreditsWindow::NUM_GROUP_MEMBERS; ++i)
    {
        faces_[i].rotate(rot_amount);
    }
}

void Credits::RunCredits()
{
    // Create an SFML window
    std::cout << "Starting to run the Credits Window" << std::endl;
    sf::RenderWindow window(sf::VideoMode(window_width_, window_height_), window_name_);
    activate_window();

    // Track time for movement
    sf::Clock clock;

    // play the music
    background_music.setLoop(true);
    background_music.play();

    // keep polling as long as we want
    while (window.isOpen())
    {
        
        // Restart the clock every frame
        sf::Time deltaTime = clock.restart();

        // Process SFML events (e.g., close the window)
        sf::Event event;
        while (window.pollEvent(event))
        {

            // if we close the window then close everything
            if (event.type == sf::Event::Closed)
            {
                window.close();
                close_window();
            }

            if (event.type == sf::Event::KeyReleased)
            {
                // escape key used to exit
                if (event.key.code == sf::Keyboard::Escape)
                {
                    window.close();
                    close_window();
                }
            }

            // Keep if we want them to click on the person
            if (sf::Mouse::isButtonPressed(sf::Mouse::Left))
            {
                sf::Vector2i mouse_pos = sf::Mouse::getPosition(window);
                std::cout << "Mouse Click Deteced." << std::endl;
                std::cout << "Clicked on " << mouse_pos.x << ", " << mouse_pos.y << std::endl;
            }
        }
        
        // clear the current window
        window.clear();
        window.draw(background);

        // rotate the sprites
        RotateSprites(deltaTime);

        for(int i = 0; i < CreditsWindow::NUM_GROUP_MEMBERS; ++i)
        {
            window.draw(faces_[i]);
        }

        // Display the updated frame
        window.display();
    }
}