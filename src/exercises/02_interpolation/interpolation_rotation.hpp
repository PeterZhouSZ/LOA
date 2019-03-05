#pragma once

#include "../base_exercise/base_exercise.hpp"

#ifdef EXERCISE_INTERPOLATION_ROTATION

struct scene_exercise : base_scene_exercise
{

    vcl::vec3 translation_1;
    vcl::vec3 axis_1;
    float angle_1;

    vcl::vec3 translation_2;
    vcl::vec3 axis_2;
    float angle_2;

    vcl::mesh_drawable frame;
    vcl::timer_interval_reversing timer;


    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);

};

#endif
