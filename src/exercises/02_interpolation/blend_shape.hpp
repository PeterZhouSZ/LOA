#pragma once

#include "../base_exercise/base_exercise.hpp"

#ifdef EXERCISE_BLEND_SHAPE


struct scene_exercise : base_scene_exercise
{
    // Keyframe data position + time
    std::vector<std::vector<vcl::vec3> > keyframe_position;
    std::vector<float> keyframe_time;

    // Visual representation of the character
    std::vector<vcl::vec3> current_position;
    vcl::mesh_drawable face;
    vcl::mesh_drawable body;

    // Storage variables for the GUI related to this part
    bool display_wireframe = false;
    bool display_body      = true;
    vcl::timer_interval timer;


    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);


};


#endif

