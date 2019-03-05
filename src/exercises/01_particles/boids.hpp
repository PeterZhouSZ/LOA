#pragma once

#include "../base_exercise/base_exercise.hpp"

#ifdef EXERCISE_BOIDS

// Structure of a particle
struct particle_structure
{
    vcl::vec3 p; // Position
    vcl::vec3 v; // Speed
    vcl::vec3 f; // Forces
};

struct scene_exercise : base_scene_exercise
{

    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);


    std::vector<particle_structure> particles; // Boids
    vcl::mesh_drawable cone;                   // Visual display of boids as cones

    vcl::timer_basic timer;
};






#endif
