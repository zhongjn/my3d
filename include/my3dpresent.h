#pragma once
#include "my3d.h"
#include <Windows.h>
namespace my3d {
namespace present {
#ifdef WIN32
#include <Windows.h>
void initialize(HWND hwnd, int w, int h, int pixel_size = 5);
void close();
void present();
void set_pixel(int x, int y, my3d::Color color);
void set_all_pixels(my3d::Context &context);
#else
#error("my3dpresent currently supports only Windows.")
#endif
}  // namespace present
}  // namespace my3d