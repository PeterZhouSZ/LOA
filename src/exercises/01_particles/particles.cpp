
#include "particles.hpp"

#include <random>

#ifdef EXERCISE_PARTICLES

using namespace vcl;

// Generator for uniform random number
std::default_random_engine generator;
std::uniform_real_distribution<float> distrib(0.0,1.0);


static void set_gui(timer_event& timer);

/** This function is called before the beginning of the animation loop
    It is used to initialize all part-specific data */
void scene_exercise::setup_data(std::map<std::string,GLuint>& , scene_structure& scene, gui_structure& gui)
{
    // Create mesh for particles represented as spheres
    sphere = mesh_drawable(mesh_primitive_sphere(0.03f));
    sphere.uniform_parameter.color = vec3(0.6f, 0.6f, 1.0f);

    // Create mesh for the ground displayed as a disc
    ground = mesh_drawable(mesh_primitive_disc(1.25f,{0,-0.01f,0},{0,1,0},80));
    ground.uniform_parameter.color = vec3(1,1,1);

    // Initial camera position
    scene.camera.scale = 5.0f;
    const mat3 Ry = rotation_from_axis_angle_mat3({0,1,0},float(M_PI)/4.0f);
    const mat3 Rx = rotation_from_axis_angle_mat3({1,0,0},-float(M_PI)/8.0f);
    scene.camera.orientation = Ry*Rx;

    // Delay between emission of a new particles
    timer.periodic_event_time_step = 0.2f;

    // Default gui display
    gui.show_frame_worldspace = true;
    gui.show_frame_camera = false;
}



/** This function is called at each frame of the animation loop.
    It is used to compute time-varying argument and perform data data drawing */
void scene_exercise::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{

    timer.update();
    const float t_current = timer.t;
    set_gui(timer);

    // Draw ground
    ground.draw(shaders["mesh"],scene.camera);


    // Emission of new particle if needed
    const bool is_new_particle = timer.event;
    if( is_new_particle )
    {
        particle_structure new_particle;
        new_particle.t0 = t_current;
        new_particle.p0 = vec3(0,0,0);

        // Initial speed is random. (x,z) components are uniformly distributed along a circle.
        const float theta = 2*float(M_PI)*distrib(generator);
        new_particle.v0 = vec3( std::cos(theta),5.0f, std::sin(theta));

        particles.push_back(new_particle);
    }

    // Compute position of particles at current time
    const vec3 g = {0,-9.81f,0};
    for(particle_structure& particle : particles)
    {
        const float t = t_current-particle.t0; // local time of the current particle since its creation
        const vec3 p = g*t*t/2.0f + particle.v0*t + particle.p0;

        sphere.uniform_parameter.translation = p;
        sphere.draw(shaders["mesh"], scene.camera);
    }

    // Remove particles that have exceeded they lifetime
    for(auto it = particles.begin(); it!=particles.end(); ++it)
        if( t_current-it->t0 > 2)
            it = particles.erase(it);

}


/** Part specific GUI drawing */
static void set_gui(timer_event& timer)
{
    // Can set the speed of the animation
    float scale_min = 0.05f;
    float scale_max = 2.0f;
    ImGui::SliderScalar("Time scale", ImGuiDataType_Float, &timer.scale, &scale_min, &scale_max, "%.2f s");

    // Start and stop animation
    if (ImGui::Button("Stop"))
        timer.stop();
    if (ImGui::Button("Start"))
        timer.start();
}


#endif
