#include "picking_sphere.hpp"

namespace vcl
{

picking_info::picking_info()
    :picking_valid(false), intersection(), normal({1,0,0})
{}

ray picking_ray(const camera_scene& camera, float x, float y)
{
    ray r;
    r.p = camera.camera_position();

    vec4 dir_screen = {x,y,-1.0f,1.0f};
    mat4 Proj_inv = camera.perspective.matrix_inverse();
    mat4 View_inv = camera.camera_matrix();

    vec4 dir_eye = Proj_inv * dir_screen;
    vec4 dir = View_inv * vec4(dir_eye.x,dir_eye.y,-1.0f,0.0f);

    r.u = normalize(vec3(dir[0],dir[1],dir[2]));

    return r;
}

picking_info picking_sphere(const ray& r, const vec3& center, float radius)
{

    picking_info pick;

    const vec3 d = r.p-center;
    const float b = dot(r.u,d);
    const float c = dot(d,d)-radius*radius;

    const float delta = b*b-c;
    if(delta >= 0)
    {
        const float t0 = -b - std::sqrt(delta);
        const float t1 = -b + std::sqrt(delta);

        const float t = t0>0? t0 : t1;

        pick.picking_valid = true;
        pick.intersection = r.p + t*r.u;
        pick.normal = normalize(pick.intersection - center);
    }

    return pick;
}

picking_info picking_plane(const ray& r, const vec3& p, const vec3& n)
{
    picking_info picking;

    const float t = - dot(r.p-p,n)/dot(r.u,n);
    if(t>0)
    {
        picking.picking_valid=true;
        picking.intersection = r.p + t*r.u;
        picking.normal= n;
    }
    return picking;
}

}

