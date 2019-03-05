#pragma GCC diagnostic ignored "-Weffc++"

#pragma once

#include "vcl/core/math/math.hpp"
#include "vcl/core/scene/camera/camera.hpp"

namespace vcl
{

struct picking_info
{
    picking_info();

    bool picking_valid = false;
    vec3 intersection = {0,0,0};
    vec3 normal = {1,0,0};
};

struct ray
{
    vec3 p;
    vec3 u;
};


ray picking_ray(const camera_scene& camera, float x, float y);

picking_info picking_sphere(const ray& r, const vec3& center, float radius);
picking_info picking_plane(const ray& r, const vec3& position, const vec3& normal);

}
