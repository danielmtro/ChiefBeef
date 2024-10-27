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

    // battery based stuff
    battery_percentage_ = 100.0f;
    initialise_battery_textures();


    // background music variables
    std::string music_filename = "/Music/MapWindowMusic.ogg";
    std::string music_path = ament_index_cpp::get_package_share_directory("gui") + music_filename;
    if (!background_music.openFromFile(music_path)) {
        std::cerr << "Error loading music file!" << std::endl;
        return;
    }
    background_music.setVolume(50.0f);

    std::cout << "Game Map Created" << std::endl;
}

GameMap::~GameMap()
{
    delete slam_request_button_;
    delete home_button_;
    delete next_page_button_;
    std::cout << "Game Map no longer running" << std::endl;
}

void GameMap::initialise_battery_textures()
{   
    const int NUM_BATTERY_IMAGES = 3;
    // initailise 3 possible battery photos
    battery_textures_.resize(NUM_BATTERY_IMAGES);
    for(int i = 0; i < NUM_BATTERY_IMAGES; ++i)
        battery_textures_[i] = std::make_shared<sf::Texture>();

    std::string tpath;
    std::string fname;

    fname = "/green_battery.png";
    tpath = ament_index_cpp::get_package_share_directory("gui") + "/Textures" + fname;
    if (!battery_textures_[0]->loadFromFile(tpath)) {
        std::cerr << "Failed to load texture from " << tpath << std::endl;
    }
    fname = "/orange_battery.png";
    tpath = ament_index_cpp::get_package_share_directory("gui") + "/Textures" + fname;
    if (!battery_textures_[1]->loadFromFile(tpath)) {
        std::cerr << "Failed to load texture from " << tpath << std::endl;
    }

    fname = "/red_battery.png";
    tpath = ament_index_cpp::get_package_share_directory("gui") + "/Textures" + fname;
    if (!battery_textures_[2]->loadFromFile(tpath)) {
        std::cerr << "Failed to load texture from " << tpath << std::endl;
    }

    // set the default position and battery percentage to be full
    battery_.setPosition(GmapWindow::SBUTTON_X, GmapWindow::ICON_Y - 40);
    battery_.setTextureRect(sf::IntRect(0, 0, battery_textures_[0]->getSize().x, battery_textures_[0]->getSize().y));  // Show full texture
    battery_.setTexture(*battery_textures_[0]);

    // update the position of the text
    battery_text_.setFont(font);
    battery_text_.setCharacterSize(30);
    battery_text_.setFillColor(sf::Color::Green);
    battery_text_.setString("100.0 %");
    battery_text_.setPosition(GmapWindow::SBUTTON_X, GmapWindow::ICON_Y- 40 + battery_textures_[0]->getSize().y);
}

void GameMap::update_battery_state()
{
    int bat_level;
    battery_percentage_ = map_->get_battery_percentage();
    if(battery_percentage_ > 70)
    {
        bat_level = 0;
        battery_text_.setFillColor(sf::Color::Green);
    }
    else if(battery_percentage_ < 35)
    {
        bat_level = 2;
        battery_text_.setFillColor(sf::Color::Red);
    }
    else
    {
        bat_level = 1;
        battery_text_.setFillColor(sf::Color(255, 128, 0));
    }

    battery_.setTextureRect(sf::IntRect(0, 0, battery_textures_[bat_level]->getSize().x, battery_textures_[bat_level]->getSize().y));  // Show full texture
    battery_.setTexture(*battery_textures_[bat_level]);
    battery_text_.setString(std::to_string(static_cast<int>(battery_percentage_)) + " %");

}

void GameMap::play_button_sound(int button_num_)
{
    std::string fname;

    // select which music to load
    if(button_num_ == 0)
        fname = "Zeev-stocktake.ogg";
    else if(button_num_ == 1)
        fname = "Zeev-goodbye.ogg";
    else
        fname = "Zeev-goodbye.ogg";

    // load in the sound that we want
    std::string path = ament_index_cpp::get_package_share_directory("gui") + "/Music/" + fname;
    if (!buffer_.loadFromFile(path)) {  // Use .ogg file here
        std::cerr << "Failed to load sound!" << std::endl;
        return;
    }
    click_sound_.setBuffer(buffer_);
    click_sound_.setVolume(500.0f);

    // play the sound
    click_sound_.play();

    return;
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
    for(int i = 0; i < GmapWindow::NUM_ITEMS; ++i)
    {
        items_in_store_[i]->initialise(window, "/" + MENU_INDEX_TO_ITEM.at(i) + ".png");
        
        // set the vertical position of each item and its number of the screen
        if(i%2 == 0)
        {
            items_in_store_[i]->set_position(GmapWindow::ICON_X, GmapWindow::ICON_Y);
            number_of_items_[i]->setPosition(GmapWindow::ICON_X + 60, GmapWindow::ICON_Y - 15);
        }
        else
        {
            items_in_store_[i]->set_position(GmapWindow::ICON_X, GmapWindow::ICON_Y + GmapWindow::ICON_SEP);
            number_of_items_[i]->setPosition(GmapWindow::ICON_X + 60, GmapWindow::ICON_Y + GmapWindow::ICON_SEP - 15);
        }
    }

    // deactivate the last question mark (only display one on the window)
    items_in_store_[GmapWindow::NUM_ITEMS - 1]->deactivate();

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
        std::string item = MENU_INDEX_TO_ITEM.at(i);
        int item_num = map_->get_item_logger()->get_num_items(item);
        number_of_items_[i]->setString(std::to_string(item_num));

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

    update_battery_state();
    window.draw(battery_);
    window.draw(battery_text_);

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
                    play_button_sound(0);
                    map_->publish_slam_request();
                    slam_request_button_->deactivate_button();
                }
                else if(home_button_->buttonHover(mouse_pos))
                {
                    play_button_sound(1);
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