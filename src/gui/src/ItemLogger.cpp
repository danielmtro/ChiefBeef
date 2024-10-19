/*
ItemLogger.cpp Implementation

This is the Implementation for the ItemLogger class
that will be used to save and store inventory from 
the robot
Written: Daniel Monteiro
Date: 17/10/2024
*/

#include "ItemLogger.hpp"

ItemLogger::ItemLogger()
{

    num_items_ = 0;
    std::cout << "Created an Item Logger" << std::endl;
}

ItemLogger::~ItemLogger()
{
    std::cout << "Item Logger destroyed" << std::endl;
}

void ItemLogger::add_item(std::string key)
{   
    // if the item hasn't been added before
    if(item_map_count_.find(key) == item_map_code_.end())
    {   
        // create the item and set it to 1
        item_map_count_[key] = 1;

        // give the current key a unique integer code
        item_map_code_[key]= num_items_;

        // create a map that lets us go both ways
        code_to_key_[num_items_] = key;

        ++num_items_;
        return;
    }

    // increment the number of the current item
    item_map_count_[key] = item_map_count_[key] + 1;

}

int ItemLogger::get_code(std::string key)
{
    if(item_map_code_.find(key) == item_map_code_.end())
        std::cerr << "The key " << key << " hasn't been logged yet!\n";
   
    return item_map_code_[key];
}

int ItemLogger::get_num_items(std::string key)
{
    if(item_map_code_.find(key) == item_map_code_.end())
        return 0;

    return item_map_count_[key];
}

int ItemLogger::get_num_items(int code)
{
    if(code_to_key_.find(code) == code_to_key_.end())
        return 0;

    std::string key = code_to_key_[code];
    return item_map_count_[key];
}

void ItemLogger::remove_item(std::string key)
{
    // check that we have the item
    if(item_map_count_.find(key) == item_map_code_.end())
    {
        std::cerr << "The key " << key << " hasn't been logged yet!\n";
    }

    // remove the item
    if(item_map_count_[key] > 0)
    {
        item_map_count_[key] = item_map_count_[key] - 1;
    }
    else
    {
        std::cout << "You have none of this item to remove.";
    }
    
}

void ItemLogger::remove_item(int code)
{
    if(code_to_key_.find(code) == code_to_key_.end())
        std::cerr << "The code " << code << " hasn't been logged yet!\n";
    
    std::string key = code_to_key_[code];
    if(item_map_count_[key] > 0)
    {
        item_map_count_[key] = item_map_count_[key] - 1;
    }
    else
    {
        std::cout << "You have none of this item to remove.";
    }

}