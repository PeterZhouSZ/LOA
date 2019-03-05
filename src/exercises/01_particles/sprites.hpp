#pragma once

#include "../base_exercise/base_exercise.hpp"


#ifdef EXERCISE_SPRITES

struct particle_structure
{
    vcl::vec3 p0;
};

struct scene_exercise : base_scene_exercise
{

    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);


    std::vector<particle_structure> particles;
    vcl::mesh_drawable sprite;
    GLuint sprite_texture;
    static bool enable_sprite;

    vcl::mesh_drawable pipe;


};

#endif
