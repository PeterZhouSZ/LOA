
#include "sphere_gravity.hpp"

#include <random>

#ifdef EXERCISE_SPHERE_GRAVITY

using namespace vcl;

// Generator for uniform random number
std::default_random_engine generator;
std::uniform_real_distribution<float> distrib(0.0,1.0);

static void set_gui(timer_basic& timer);



void scene_exercise::setup_data(std::map<std::string,GLuint>& , scene_structure& , gui_structure& )
{

    sphere = mesh_drawable( mesh_primitive_sphere(1.0f));


    std::vector<vec3> borders_segments = {{-1,-1,-1},{1,-1,-1}, {1,-1,-1},{1,1,-1}, {1,1,-1},{-1,1,-1}, {-1,1,-1},{-1,-1,-1},
                                          {-1,-1,1} ,{1,-1,1},  {1,-1,1}, {1,1,1},  {1,1,1}, {-1,1,1},  {-1,1,1}, {-1,-1,1},
                                          {-1,-1,-1},{-1,-1,1}, {1,-1,-1},{1,-1,1}, {1,1,-1},{1,1,1},   {-1,1,-1},{-1,1,1}};
    borders = segments_gpu(borders_segments);
    borders.uniform_parameter.color = {0,0,0};
}





void scene_exercise::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    const float dt = timer.update();
    set_gui(timer);

    // Emission of new particle if needed
    timer.periodic_event_time_step = 0.3f;
    const bool is_new_particle = timer.event;
    const std::vector<vec3> color_lut = {{1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,0,1},{0,1,1}};
    if( is_new_particle )
    {
        particle_structure new_particle;

        new_particle.r = 0.08f;
        new_particle.c = color_lut[int(distrib(generator)*color_lut.size())];

        // Initial position
        new_particle.p = vec3(0,0,0);

        // Initial speed
        const float theta = 2*float(M_PI)*distrib(generator);
        new_particle.v = vec3( 2*std::cos(theta),5.0f, 2*std::sin(theta));

        particles.push_back(new_particle);
    }


    // Set forces
    const size_t N = particles.size();
    for(size_t k=0; k<N; ++k)
    {
        particles[k].f = vec3(0,-9.81f,0);
    }


    // Integrate position and speed of particles through time
    for(size_t k=0; k<N; ++k)
    {
        particle_structure& particle = particles[k];
        const float attenuation = 2.0f;

        particle.v = (1.0f-dt*attenuation)*particle.v + dt*particle.f;
        particle.p = particle.p + dt*particle.v;
    }

    // Collisions between spheres
    // ...

    // Collisions with cube
    // ...




    // Display particles
    for(size_t k=0; k<N; ++k)
    {
        const particle_structure& part = particles[k];

        sphere.uniform_parameter.translation = part.p;
        sphere.uniform_parameter.scaling = part.r;
        sphere.uniform_parameter.color = part.c;
        sphere.draw(shaders["mesh"], scene.camera);
    }

    borders.draw(shaders["curve"], scene.camera);
}


/** Part specific GUI drawing */
static void set_gui(timer_basic& timer)
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
