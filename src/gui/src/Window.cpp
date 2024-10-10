/*
Window.cpp Implementation

This is the implementation for the parent Window Class
Written: Daniel Monteiro
*/

#include "Window.hpp"

Window::Window(const std::string& name, int width, int height)
    :  window_name_(name), window_width_(width), window_height_(height)

{
    window_active_ = false;
    std::cout << "Window " << window_name_ << " Created" << std::endl;
}

Window::~Window()
{
    std::cout << "Window " << window_name_  << " no longer active" << std::endl;
}

void Window::activate_window()
{
    window_active_ = true;
}

void Window::close_window()
{
    window_active_ = false;
}

bool Window::is_window_active() const
{
    return window_active_;
}


