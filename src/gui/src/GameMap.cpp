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

    // set the width of the border to match the buttons that are present
    border_width_ = GmapWindow::SBUTTON_X + GmapWindow::SBUTTON_W;
    page_num_ = 0;

    // set the background for the main menu
    std::string background_location = "/Textures/concrete_background.png";
    std::string texture_path = ament_index_cpp::get_package_share_directory("gui") + background_location;
    std::cout << "Background path for Game Map " << texture_path << std::endl;

    // load the texture  
    background.setSize(sf::Vector2f(window_width_, window_height_));
    if(!texture.loadFromFile(texture_path))
        std::cout << "Failed to load background at " << texture_path << std::endl;
    background.setTexture(&texture);

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

    // create the request button
    home_button_ = new Button(
        GmapWindow::HBUTTON_X, GmapWindow::HBUTTON_Y,
        GmapWindow::HBUTTON_W, GmapWindow::HBUTTON_H,
        sf::Color(0, 128, 255),   // fun colour
        GmapWindow::HBUTTON_WORD,
        font
    );

    // next page button
    next_page_button_ = new Button(
        GmapWindow::NBUTTON_X, GmapWindow::NBUTTON_Y,
        GmapWindow::NBUTTON_W, GmapWindow::NBUTTON_H,
        sf::Color(153, 51, 255),   // fun colour
        GmapWindow::NBUTTON_WORD,
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
        item_num->setFillColor(sf::Color::Blue);

        number_of_items_.push_back(item_num);
    }

    // position of the actual trolley
    trolley_ = std::make_shared<CharacterIcon>();


    // background music variables
    std::string music_filename = "/Music/MapWindowMusic.ogg";
    std::string music_path = ament_index_cpp::get_package_share_directory("gui") + music_filename;
    if (!background_music.openFromFile(music_path)) {
        std::cerr << "Error loading music file!" << std::endl;
        return;
    }

    std::cout << "Game Map Created" << std::endl;
}

GameMap::~GameMap()
{
    delete slam_request_button_;
    delete home_button_;
    delete next_page_button_;
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

    // Apply a Median Filter to remove noise
    cv::Mat finalImage;
    cv::medianBlur(image, finalImage, 3);

    // code to center the map in the middle of the window
    int window_width = window.getSize().x;
    int window_height = window.getSize().y;

    // we presume that the window is larger than the map size
    if(window_width < width || window_height < height)
        std::cerr << "Small window or Large Map not supported" << std::endl;

    // define scaling factors and offsets for placing window
    // adjust x scaling to not interfere with button positions
    int effective_x_width = window_width - 2 * (border_width_);
    int sf_x = effective_x_width/width;
    int sf_y = window_height/height;

    // take the minimum scaling and set it to both
    sf_x = (sf_x < sf_y) ? sf_x : sf_y;
    sf_y = sf_x;
    scaling_factor_ = sf_x;

    // find offsets to center the map in the middle of the screen
    x_offset_ = (effective_x_width - width * sf_x)/2 + border_width_;
    y_offset_ = (window_height - height * sf_y)/2;
    
    // create cells and display them on the window
    sf::RectangleShape cell(sf::Vector2f(sf_x, sf_y));
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            int cval = image.at<uchar>(y, x);
            // if there is a certain square at this position then plot it
            if(cval == WHITE_UINT)
            {
                // brownish colour
                cell.setFillColor(sf::Color(102, 51, 0));
    
                // Set position and draw the cell
                cell.setPosition((x * sf_x + x_offset_), (y * sf_y + y_offset_));
                window.draw(cell);
            }
        }
    }
    
    map_width_ = width;
    map_height_ = height;
}

void GameMap::initialise(sf::RenderWindow& window)
{

    // create a rectangle around the objects
    bounding_box_.setSize(sf::Vector2f(155, 230)); // Size matches sprite
    bounding_box_.setPosition(GmapWindow::ICON_X - 40, GmapWindow::ICON_Y - 40);  
    bounding_box_.setOutlineColor(sf::Color::Blue);  // Set outline color
    bounding_box_.setOutlineThickness(4);  // Set thickness of the outline
    bounding_box_.setFillColor(sf::Color::Transparent);  // Make the inside transparent

    // slide 1
    items_in_store_[0]->initialise(window, "/apple.png");
    items_in_store_[0]->set_position(GmapWindow::ICON_X, GmapWindow::ICON_Y);

    items_in_store_[1]->initialise(window, "/orange.png");
    items_in_store_[1]->set_position(GmapWindow::ICON_X, GmapWindow::ICON_Y + GmapWindow::ICON_SEP);

    // slide 2
    items_in_store_[2]->initialise(window, "/peach.png");
    items_in_store_[2]->set_position(GmapWindow::ICON_X, GmapWindow::ICON_Y);

    items_in_store_[3]->initialise(window, "/eggplant.png");
    items_in_store_[3]->set_position(GmapWindow::ICON_X, GmapWindow::ICON_Y + GmapWindow::ICON_SEP);
    

    // slide 3
    items_in_store_[4]->initialise(window, "/question_mark.png");
    items_in_store_[4]->set_position(GmapWindow::ICON_X, GmapWindow::ICON_Y);


    // placeholder on the final slide
    items_in_store_[5]->initialise(window, "/question_mark.png");
    items_in_store_[5]->set_position(GmapWindow::ICON_X, GmapWindow::ICON_Y + GmapWindow::ICON_SEP);
    items_in_store_[5]->deactivate();


    // set the positins of the item numbers
    number_of_items_[0]->setPosition(GmapWindow::ICON_X + 60, GmapWindow::ICON_Y - 15);
    number_of_items_[1]->setPosition(GmapWindow::ICON_X + 60, GmapWindow::ICON_Y + GmapWindow::ICON_SEP - 15);

    number_of_items_[2]->setPosition(GmapWindow::ICON_X + 60, GmapWindow::ICON_Y - 15);
    number_of_items_[3]->setPosition(GmapWindow::ICON_X + 60, GmapWindow::ICON_Y + GmapWindow::ICON_SEP - 15);

    number_of_items_[4]->setPosition(GmapWindow::ICON_X + 60, GmapWindow::ICON_Y - 15);
    number_of_items_[5]->setPosition(GmapWindow::ICON_X + 60, GmapWindow::ICON_Y + GmapWindow::ICON_SEP - 15);
    

    // initialise the actual character
    trolley_->initialise(window, "/trolley_top_view.png");

}

void GameMap::draw_frame(sf::RenderWindow& window, sf::Time deltaTime)
{
    window.draw(background);

    // draw on the map
    DrawMapData(window);

    slam_request_button_->draw(window);
    home_button_->draw(window);
    next_page_button_->draw(window);

    // draw on the store items
    int page_num = page_num_%GmapWindow::NUM_PAGES; // reduce the possible pages
    for(int i = page_num * GmapWindow::ITEMS_PER_PAGE; i < (page_num + 1)* GmapWindow::ITEMS_PER_PAGE; ++i)
    {
        // make the item icons jiggle
        items_in_store_[i]->update_position(deltaTime);

        // update the number of items based on the encoded index
        number_of_items_[i]->setString(std::to_string(map_->get_item_logger()->get_num_items(i)));

        // don't draw inactive icons
        if(!items_in_store_[i]->get_active())
            continue;

        window.draw(*items_in_store_[i]->get_sprite());
        window.draw(*number_of_items_[i]);
    }

    // update the position of the character based on the current odom
    Map::Pose pose = map_->get_current_pose();
    trolley_->update_position(map_->get_current_pose(),
                              scaling_factor_,
                              x_offset_, 
                              y_offset_,
                              map_->get_map_meta_data());

    // only draw the trolley on if there is a map to draw it on
    if(map_width_ > 0 && map_height_ > 0)
        window.draw(*trolley_->get_sprite());

    window.draw(bounding_box_);
}

void GameMap::RunMap()
{
    // Create an SFML window
    sf::RenderWindow window(sf::VideoMode(window_width_, window_height_), window_name_);
    activate_window();

    // initalise everything in the store
    initialise(window);
    
    // draw on the buttons to start off
    window.clear();
    slam_request_button_->draw(window);
    window.display();

    // create mouse position
    sf::Vector2i mouse_pos;

    // Track time for movement
    sf::Clock clock;

    // start playing the background music
    background_music.setLoop(true);
    background_music.play();

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
                else if(home_button_->buttonHover(mouse_pos))
                {
                    window.close();
                    close_window();
                }
                else if(next_page_button_->buttonHover(mouse_pos))
                {
                    page_num_++;
                }
            }
        }

        // update the state of the slam request button
        mouse_pos = sf::Mouse::getPosition(window);
        slam_request_button_->buttonHover(mouse_pos);
        home_button_->buttonHover(mouse_pos);
        next_page_button_->buttonHover(mouse_pos);


        // Clear the window with a black color
        window.clear(sf::Color::Black);

        // once we read the map, let the map know 
        map_->read_map_data();

        draw_frame(window, deltaTime);

        // Display the window content
        window.display();
    }

}