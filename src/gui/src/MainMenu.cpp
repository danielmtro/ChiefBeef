/*
MainMenu.cpp Implementation

This is the implementation for the GameMap
Written: Daniel Monteiro
Date: 12/10/2024
*/

#include "MainMenu.hpp"

MainMenu::MainMenu(const std::string& name, int width, int height)
    : Window(name, width, height)
{

    FontColour = sf::Color::White;
    SelectedFontColour = sf::Color::Magenta;

    std::string font_path = ament_index_cpp::get_package_share_directory("gui") + "/Fonts/comic_sans_1.ttf";
    std::cout << "Font path " << font_path << std::endl;
    if (!font.loadFromFile(font_path))
        std::cout << "Font file not found: " << font_path << std::endl;


    // set the shopping time button
    menu[0].setFont(font);
    menu[0].setString("Shopping Time");
    menu[0].setCharacterSize(MenuWindow::CHARACTER_SIZE);
    menu[0].setPosition(400, 250);
    menu[0].setFillColor(FontColour);

    // set the meet the team button
    menu[1].setFont(font);
    menu[1].setString("Meet The Team");
    menu[1].setCharacterSize(MenuWindow::CHARACTER_SIZE);
    menu[1].setPosition(400, 350);
    menu[1].setFillColor(FontColour);

    // set the Exit button
    menu[2].setFont(font);
    menu[2].setString("Exit");
    menu[2].setCharacterSize(MenuWindow::CHARACTER_SIZE);
    menu[2].setPosition(400, 450);
    menu[2].setFillColor(FontColour);

    // set the background for the main menu
    std::string background_location = "/Textures/Menu_Background.jpeg";
    std::string texture_path = ament_index_cpp::get_package_share_directory("gui") + background_location;
    std::cout << "Texture path " << texture_path << std::endl;

    // load the texture  
    background.setSize(sf::Vector2f(window_width_, window_height_));
    if(!texture.loadFromFile(texture_path))
        std::cout << "Failed to load background at " << texture_path << std::endl;
    background.setTexture(&texture);

    // background music variables
    std::string music_filename = "/Music/MainMenuMusic.ogg";
    std::string music_path = ament_index_cpp::get_package_share_directory("gui") + music_filename;
    if (!background_music.openFromFile(music_path)) {
        std::cerr << "Error loading music file!" << std::endl;
        return;
    }

    // set variables to output the final and temporary menu choices
    selection_ = 0; 
    current_selection_ = -1;
    return;
}

MainMenu::~MainMenu()
{
    std::cout << "Menu has been closed." << std::endl;
}


void MainMenu::up_command()
{   
    // set the all values back to normal
    for(int selection = 0; selection < MenuWindow::MENU_OPTIONS; ++selection)
        menu[selection].setFillColor(FontColour);


    if(current_selection_ == 0 || current_selection_ == -1)
        current_selection_ = MenuWindow::MENU_OPTIONS - 1;
    else
        current_selection_--;

    // set the current selection to be as such
    menu[current_selection_].setFillColor(SelectedFontColour);

    return;
}

void MainMenu::down_command()
{
    
    // set the all values back to normal
    for(int selection = 0; selection < MenuWindow::MENU_OPTIONS; ++selection)
        menu[selection].setFillColor(FontColour);


    if(current_selection_ == MenuWindow::MENU_OPTIONS - 1 || current_selection_ == -1)
        current_selection_ = 0;
    else
        current_selection_++;

    // set the current selection to be as such
    menu[current_selection_].setFillColor(SelectedFontColour);

    return;
}


int MainMenu::get_menu_selection()
{
    return selection_;
}

void MainMenu::RunMenu()
{
    // reset selection variables
    selection_ = 0; 
    current_selection_ = -1;

    // set the all colour text values back to normal if reopening
    for(int selection = 0; selection < MenuWindow::MENU_OPTIONS; ++selection)
        menu[selection].setFillColor(FontColour);

    // Create an SFML window
    std::cout << "Starting to run the menu" << std::endl;
    sf::RenderWindow window(sf::VideoMode(window_width_, window_height_), window_name_);
    activate_window();

    // initialise the trolley animation
    trolley_animation.initialise(window);
    trolley_animation.set_position(MenuWindow::TROLLEY_START_X, MenuWindow::TROLLEY_START_Y);


    // Track time for movement
    sf::Clock clock;

    // start playing the background music
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
            // process arrow commands
            if (event.type == sf::Event::KeyReleased)
            {
                if (event.key.code == sf::Keyboard::Up)
                {
                    up_command();
                    break;
                }

                if (event.key.code == sf::Keyboard::Down)
                {
                    down_command();
                    break;
                }

                if (event.key.code == sf::Keyboard::Escape)
                {
                    // get out of here
                    selection_ = EXIT;
                    window.close();
                    close_window();
                    break;
                }

                // return key used to exit the menu as well once a selection is made
                if (event.key.code == sf::Keyboard::Return)
                {
                    // set the current selection before closing the menu
                    selection_ = current_selection_;
                    window.close();
                    close_window();
                }
            }

            if (sf::Mouse::isButtonPressed(sf::Mouse::Left))
            {
                sf::Vector2i mouse_pos = sf::Mouse::getPosition(window);
                std::cout << "Mouse Click Deteced." << std::endl;
                std::cout << "Clicked on " << mouse_pos.x << ", " << mouse_pos.y << std::endl;
            }
        }

        trolley_animation.update_position(deltaTime);
        
        // clear the current window
        window.clear();
        window.draw(background);

        // Add all the menu options of text to the window
        for (int i = 0; i < MenuWindow::MENU_OPTIONS; ++i)
        {
            window.draw(menu[i]);
        }

        // sf::Sprite& sprite = trolley_animation.get_sprite();
        window.draw(*trolley_animation.get_sprite());

        // Display the updated frame
        window.display();
    }
}