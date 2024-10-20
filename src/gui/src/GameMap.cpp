/*
GameMap.cpp Implementation

This is the implementation for the GameMap
Written: Daniel Monteiro
Date: 18/10/2024
*/

#include "GameMap.hpp"

GameMap::GameMap(const std::string& name, int width, int height, std::shared_ptr<Map> MapPtr)
    : Window(name, width, height), map_(MapPtr)
{   

    // load in the font for the button
    std::string font_path = ament_index_cpp::get_package_share_directory("gui") + "/Fonts/comic_sans_1.ttf";
    font.loadFromFile(font_path);

    // create the request button
    slam_request_button_ = new Button(
        GmapWindow::SBUTTON_X, GmapWindow::SBUTTON_Y,
        GmapWindow::SBUTTON_W, GmapWindow::SBUTTON_H,
        sf::Color(255, 158, 102),   // fun colour
        GmapWindow::SBUTTON_WORD,
        font
    );

    // create icon objects to store 
    for(int i = 0; i < GmapWindow::NUM_ITEMS; ++i)
    {
        std::shared_ptr<Icon> item = std::make_shared<Icon>();
        std::shared_ptr<sf::Text> item_num = std::make_shared<sf::Text>();

        items_in_store_.push_back(item);

        item_num->setFont(font);
        item_num->setString("0");
        item_num->setCharacterSize(GmapWindow::NUM_ICON_CHARSIZE);
        item_num->setFillColor(sf::Color::Cyan);

        number_of_items_.push_back(item_num);
    }
    

    std::cout << "Game Map Created" << std::endl;
}

GameMap::~GameMap()
{
    delete slam_request_button_;
    std::cout << "Game Map no longer running" << std::endl;
}

void GameMap::DrawMapData(sf::RenderWindow& window)
{

    std::vector<int8_t> map_data = map_->get_map();

    uint32_t width = map_->get_width();
    uint32_t height = map_->get_height();

    if( (!width) || (!height))
        return;

    // convert to opencv matrix
    cv::Mat image(height, width, CV_8UC1);

    // Populate the matrix with pixel values (convert to 0 or 255)
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            int value = map_data[i * width + j];
            image.at<uchar>(i, j) = (value == 100) ? 255 : 0;
        }
    }

    // Apply morphological operations (Erosion and Dilation)
    cv::Mat cleanedImage;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(image, cleanedImage, cv::MORPH_OPEN, element);

    // Apply a Median Filter to remove noise
    cv::Mat finalImage;
    cv::medianBlur(image, finalImage, 3);


    sf::RectangleShape cell(sf::Vector2f(10.0f, 10.0f)); // Each cell is a 5x5 pixel square
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {

            int cell_val = finalImage.at<uchar>(y, x);
            int occupancy_value = (cell_val == -1) ? 0 : cell_val;
            // std::cout << occupancy_value << " ";

            cell.setFillColor(sf::Color(occupancy_value, occupancy_value, occupancy_value));
    
            // Set position and draw the cell
            cell.setPosition(x * 5.0f, y * 5.0f);
            window.draw(cell);
        }

        // std::cout << "\n";
    }
    

}


void GameMap::initialise_item_menu(sf::RenderWindow& window)
{

    // create a rectangle around the objects
    bounding_box_.setSize(sf::Vector2f(160, 155)); // Size matches sprite
    bounding_box_.setPosition(GmapWindow::ICON_X - 40, GmapWindow::ICON_Y - 40);  
    bounding_box_.setOutlineColor(sf::Color::Blue);  // Set outline color
    bounding_box_.setOutlineThickness(4);  // Set thickness of the outline
    bounding_box_.setFillColor(sf::Color::Transparent);  // Make the inside transparent

    // initialise the icon sprites
    items_in_store_[0]->initialise(window, "/apple.png");
    items_in_store_[0]->set_position(GmapWindow::ICON_X, GmapWindow::ICON_Y);
    items_in_store_[1]->initialise(window, "/orange.png");
    items_in_store_[1]->set_position(GmapWindow::ICON_X, GmapWindow::ICON_Y + GmapWindow::ICON_SEP);
    
    number_of_items_[0]->setPosition(GmapWindow::ICON_X + 60, GmapWindow::ICON_Y - 15);
    number_of_items_[1]->setPosition(GmapWindow::ICON_X + 60, GmapWindow::ICON_Y + GmapWindow::ICON_SEP - 15);
}

void GameMap::RunMap()
{
    // Create an SFML window
    sf::RenderWindow window(sf::VideoMode(window_width_, window_height_), window_name_);
    activate_window();

    // initalise the upper menu of items in the store
    initialise_item_menu(window);
    
    // draw on the buttons to start off
    window.clear();
    slam_request_button_->draw(window);
    window.display();

    sf::Vector2i mouse_pos;

    // Track time for movement
    sf::Clock clock;

    while (window.isOpen())
    {

        // Restart the clock every frame
        sf::Time deltaTime = clock.restart();

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

            // check if we are clicking on something
            if (sf::Mouse::isButtonPressed(sf::Mouse::Left))
            {
                mouse_pos = sf::Mouse::getPosition(window);
                std::cout << "Mouse Click Deteced." << std::endl;
                std::cout << "Clicked on " << mouse_pos.x << ", " << mouse_pos.y << std::endl;
                
                // test if we are hovering over the right button and the button is active
                if(slam_request_button_->buttonHover(mouse_pos) && slam_request_button_->is_active())
                {
                    map_->publish_slam_request();
                    slam_request_button_->deactivate_button();
                }
            }
        }

        // update the state of the button
        mouse_pos = sf::Mouse::getPosition(window);
        slam_request_button_->buttonHover(mouse_pos);

        // Clear the window with a black color
        window.clear(sf::Color::Black);

        // once we read the map, let the map know 
        map_->read_map_data();

        // draw on the map
        DrawMapData(window);

        slam_request_button_->draw(window);

        // draw on the store items
        for(int i = 0; i < GmapWindow::NUM_ITEMS; ++i)
        {
            // make the item icons jiggle
            items_in_store_[i]->update_position(deltaTime);

            // update the number of items
            number_of_items_[i]->setString(std::to_string(map_->get_item_logger()->get_num_items(i)));
    
            window.draw(*items_in_store_[i]->get_sprite());
            window.draw(*number_of_items_[i]);
        }

        window.draw(bounding_box_);
        // Display the window content
        window.display();
    }

}