/*
ItemLogger.hpp Interface

This is the Interface for the ItemLogger class
that will be used to save and store inventory from 
the robot
Written: Daniel Monteiro
Date: 17/10/2024
*/

#ifndef _ITEM_LOGGER_HPP
#define _ITEM_LOGGER_HPP


#include <iostream>
#include <unordered_map>
#include <string>
#include "constants.hpp"

class ItemLogger
{
    public:

        ItemLogger();
        ~ItemLogger();

        /**
        * @brief gets the integer encoding for the item
        *
        * @param key the string output from an april tag that will be unique
        * for each item
        * 
        * @return the unique integer code that corresponds to this string item
        */
        int get_code(std::string key);

        /**
        * @brief gets the number of the presented item
        *
        * @param key the string output from an april tag that will be unique
        * for each item
        * 
        * @return the number of items that have been scanned
        */
        int get_num_items(std::string key);

        /**
        * @brief overloaded function of get_num_items(std::string key). Get's the item
        * based off the encoded integer instead of the string
        * 
        * @param code the integer encoding of an item
        * 
        * @return the number of items that have been scanned
        */
        int get_num_items(int code);

        /**
        * @brief adds an item to the itemlogger based off the string representation
        * @param key the string output from an april tag that will be unique
        * for each item
        */
        void add_item(std::string key);

        /**
        * @brief removes an item to the itemlogger based off the string representation
        * @param key the string output from an april tag that will be unique
        * for each item
        */
        void remove_item(std::string key);

        /**
        * @brief removes an item to the itemlogger based off the integer encoding
        * @param code the integer encoding of an item
        */
        void remove_item(int code);

    private:

        // keeps track of the number of each item
        std::unordered_map<std::string, int> item_map_count_;
        
        // keeps track of the encoding for each item
        std::unordered_map<std::string, int> item_map_code_;

        // converts code back to key
        std::unordered_map<int, std::string> code_to_key_;

        // the total number of items
        int num_items_;
};

#endif