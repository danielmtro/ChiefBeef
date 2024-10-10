/*
GameMap.cpp Implementation

This is the implementation for the GameMap
Written: Daniel Monteiro
*/

#include "GameMap.hpp"

GameMap::GameMap(const std::string& name, int width, int height, std::shared_ptr<Map> MapPtr)
    : Window(name, width, height), map_(MapPtr)
{
    std::cout << "Game Map Created" << std::endl;
}

GameMap::~GameMap()
{
    std::cout << "Game Map no longer running" << std::endl;
}


void GameMap::RunMap()
{
    // Create an SFML window
    sf::RenderWindow window(sf::VideoMode(window_width_, window_height_), window_name_);
    activate_window();

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
        }

        // only rebuild screen if a new map has been processed
        if(!(map_->get_map_available()))
            continue;

        // Clear the window with a black color
        window.clear(sf::Color::Black);

        // Update window using ROS2 data
        // For example, let's draw a simple grid based on the occupancy grid data
        uint32_t width = map_->get_width();
        uint32_t height = map_->get_height();
        std::vector<int8_t> map_data = map_->get_map();

        // once we read the map, let the map know 
        map_->read_map_data();

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

        // Display the window content
        window.display();
    }

}