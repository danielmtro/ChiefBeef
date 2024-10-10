/*
MainMenu.cpp Implementation

This is the implementation for the GameMap
Written: Daniel Monteiro
*/

#include "MainMenu.hpp"

MainMenu::MainMenu(const std::string& name, int width, int height)
    : Window(name, width, height)
{

    std::string font_path = ament_index_cpp::get_package_share_directory("gui") + "/Fonts/comic_sans_1.ttf";
    std::cout << "Font path " << font_path << std::endl;
    if (!font.loadFromFile(font_path))
        std::cout << "Font file not found: " << font_path << std::endl;


    // set the play button
    menu[0].setFont(font);
    menu[0].setString("Shopping Time");
    menu[0].setCharacterSize(MenuWindow::CHARACTER_SIZE);
    menu[0].setPosition(400, 200);
    menu[0].setFillColor(sf::Color::White);

    // set the play button
    menu[1].setFont(font);
    menu[1].setString("Meet The Team");
    menu[1].setCharacterSize(MenuWindow::CHARACTER_SIZE);
    menu[1].setPosition(400, 300);

    // set the Exit button
    menu[2].setFont(font);
    menu[2].setString("Exit");
    menu[2].setCharacterSize(MenuWindow::CHARACTER_SIZE);
    menu[2].setPosition(400, 400);


    selection_ = 0; // this means that no selection has been made
    current_selection_ = -1;
    return;
}

MainMenu::~MainMenu()
{
    std::cout << "Menu has been closed." << std::endl;
}


void MainMenu::up_command()
{   
    // set the current value back to normal
    menu[current_selection_].setFillColor(sf::Color::White);


    if(current_selection_ == 0 || current_selection_ == -1)
        current_selection_ = MenuWindow::MENU_OPTIONS - 1;
    else
        current_selection_--;

    // set the current selection to be as such
    menu[current_selection_].setFillColor(sf::Color::Magenta);

    return;
}

void MainMenu::down_command()
{
    
    // set the current value back to normal
    menu[current_selection_].setFillColor(sf::Color::White);


    if(current_selection_ == MenuWindow::MENU_OPTIONS - 1 || current_selection_ == -1)
        current_selection_ = 0;
    else
        current_selection_++;

    // set the current selection to be as such
    menu[current_selection_].setFillColor(sf::Color::Magenta);

    return;
}


int MainMenu::get_menu_selection()
{
    return selection_;
}


void MainMenu::RunMenu()
{

    // Create an SFML window
    std::cout << "Starting to run the menu" << std::endl;
    sf::RenderWindow window(sf::VideoMode(window_width_, window_height_), window_name_);
    activate_window();

    // keep polling as long as we want
    while (window.isOpen())
    {

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


                // return key used to exit the menu as well once a selection is made
                if (event.key.code == sf::Keyboard::Return)
                {
                    // set the current selection before closing the menu
                    selection_ = current_selection_;
                    window.close();
                    close_window();
                }
            }
        }

        // clear the current window
        window.clear();

        // Add all the menu options of text to the window
        for (int i = 0; i < MenuWindow::MENU_OPTIONS; ++i)
        {
            window.draw(menu[i]);
        }

        // Display the updated frame
        window.display();
    }
}