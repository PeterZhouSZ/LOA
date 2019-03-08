#pragma once

#include "../camera/camera.hpp"
#include "../../math/vec/vec2/vec2.hpp"
#include <GLFW/glfw3.h>
#include <vector>

namespace vcl
{

class camera_control_glfw
{
public:

    void update_mouse_move(camera_scene& camera, GLFWwindow* window, float x1, float y1, std::vector<vec3> &draw_points);
    void update_mouse_click(camera_scene& camera, GLFWwindow* window,  int button, int action, int mods);

    bool update = true;

private:

    /** previous mouse position (x-coordinate) */
    float x0 = 0.0f;
    /** previous mouse position (y-coordinate) */
    float y0 = 0.0f;
    bool reset_stroke = false;//for the LOA drawing
};

}
