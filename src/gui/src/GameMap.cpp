/*
GameMap.cpp Implementation

This is the implementation for the GameMap
Written: Daniel Monteiro
*/

#include "GameMap.hpp"

GameMap::GameMap(const std::string& name, int width, int height, std::shared_ptr<Map> MapPtr)
    : Window(name, width, height), map_(MapPtr)
{   

    // load in the font for the button
    std::string font_path = ament_index_cpp::get_package_share_directory("gui") + "/Fonts/comic_sans_1.ttf";
    font.loadFromFile(font_path);

    slam_request_button_ = new Button(
        GmapWindow::SBUTTON_X, GmapWindow::SBUTTON_Y,
        GmapWindow::SBUTTON_W, GmapWindow::SBUTTON_H,
        sf::Color(255, 158, 102),   // fun colour
        GmapWindow::SBUTTON_WORD,
        font
    );

    std::cout << "Game Map Created" << std::endl;
}

GameMap::~GameMap()
{
    delete slam_request_button_;
    std::cout << "Game Map no longer running" << std::endl;
}

void GameMap::DrawMapData(sf::RenderWindow& window)
{
    // Update window using ROS2 data
    // For example, let's draw a simple grid based on the occupancy grid data
        
    std::vector<int8_t> map_data = map_->get_map();

    uint32_t width = map_->get_width();
    uint32_t height = map_->get_height();
    if (width > 0 && height > 0)
        {
            sf::RectangleShape cell(sf::Vector2f(10.0f, 10.0f)); // Each cell is a 5x5 pixel square
            for (uint32_t y = 0; y < height; ++y)
            {
                for (uint32_t x = 0; x < width; ++x)
                {
                    int8_t occupancy_value = map_data[y * width + x];
                    if (occupancy_value == 0)  // Free space
                        cell.setFillColor(sf::Color::White);
                    else if (occupancy_value == 100)  // Occupied space
                        cell.setFillColor(sf::Color::Red);
                    else  // Unknown
                        cell.setFillColor(sf::Color::Black);

                    // Set position and draw the cell
                    cell.setPosition(x * 5.0f, y * 5.0f);
                    window.draw(cell);
                }
            }
        }

}


void GameMap::RunMap()
{
    // Create an SFML window
    sf::RenderWindow window(sf::VideoMode(window_width_, window_height_), window_name_);
    activate_window();
    
    // draw on the buttons to start off
    window.clear();
    slam_request_button_->draw(window);
    window.display();

    while (window.isOpen())
    {
        // Process SFML events (e.g., close the window)
        sf::Event event;
        while (window.pollEvent(event))
        {   

            // close the window if we manually scape the whole thing
            if (event.type == sf::Event::Closed)
            {
                window.close();
                close_window();
            }

            // Close the window when we press escape
            if (event.type == sf::Event::KeyReleased)
            {
                if(event.key.code == sf::Keyboard::Escape)
                {
                    window.close();
                    close_window();
                }
            }

            if (sf::Mouse::isButtonPressed(sf::Mouse::Left))
            {
                sf::Vector2i mouse_pos = sf::Mouse::getPosition(window);
                std::cout << "Mouse Click Deteced." << std::endl;
                std::cout << "Clicked on " << mouse_pos.x << ", " << mouse_pos.y << std::endl;
                if(slam_request_button_->buttonClicked(mouse_pos))
                {
                    map_->publish_slam_request();
                }
            }
        }

        // only rebuild screen if a new map has been processed
        if(!(map_->get_map_available()))
            continue;

        // Clear the window with a black color
        window.clear(sf::Color::Black);

        // once we read the map, let the map know 
        map_->read_map_data();

        // draw on the map
        DrawMapData(window);

        slam_request_button_->draw(window);
        // Display the window content
        window.display();
    }

}