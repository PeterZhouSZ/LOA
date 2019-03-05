#pragma once

#include "../base_exercise/base_exercise.hpp"

#ifdef EXERCISE_INTERPOLATION_POSITION

struct scene_exercise : base_scene_exercise
{


    // Keyframe data position + time
    std::vector<vcl::vec3>  keyframe_position;
    std::vector<float> keyframe_time;

    // Visual representation of a sphere
    vcl::mesh_drawable sphere;

    // Visual representation of a set of segments
    vcl::curve_drawable polygon_segments;
    vcl::timer_interval timer;


    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);

};

#endif
