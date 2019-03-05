
#include "boids.hpp"

#include <random>

#ifdef EXERCISE_BOIDS

using namespace vcl;

// Generator for uniform random number
std::default_random_engine generator;
std::uniform_real_distribution<float> distrib(0.0,1.0);

void add_boids(std::vector<particle_structure>& particles);
static void set_gui(std::vector<particle_structure>& particles,timer_basic& timer);

void scene_exercise::setup_data(std::map<std::string,GLuint>& , scene_structure& , gui_structure& )
{
    // Create boids
    add_boids(particles);

    // Initialize the visual model of boids
    cone = mesh_drawable( mesh_primitive_cone(0.1f,{0,0,0},{0,0,0.5f}) );
    cone.uniform_parameter.scaling = 0.15f;
    cone.uniform_parameter.color = {0,0,1};
}


void scene_exercise::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    const float dt = timer.update();
    set_gui(particles,timer);

    // Initialize forces to zero
    const size_t N = particles.size();
    for(size_t k=0; k<N; ++k)
        particles[k].f = {0,0,0};

    // Add forces ...


    // Integrate position and speed of boids through time
    for(size_t k=0; k<N; ++k)
    {
        particle_structure& particle = particles[k];

        particle.v = particle.v + dt*particle.f;
        particle.p = particle.p + dt*particle.v;
    }


    // Display of boids
    for(size_t k=0; k<N; ++k)
    {
        const vec3& p = particles[k].p;
        const vec3& v = particles[k].v;
        const mat3 R = rotation_between_vector_mat3({0,0,1},v);

        cone.uniform_parameter.translation = p;
        cone.uniform_parameter.rotation = R;

        cone.draw(shaders["mesh"], scene.camera);
    }
}


/** Add new boids in the scene */
void add_boids(std::vector<particle_structure>& particles)
{
    // Max number of boids to be added
    const size_t N = 50;

    // Re-Initialize seed of the generator to a new value
    generator.seed( (unsigned int)(500*glfwGetTime()) );

    for(size_t k=0; k<N; ++k)
    {
        // Initial position and speed distribution is 2D to simplify the visualization

        particle_structure particle;
        particle.p = {distrib(generator)-0.5f,distrib(generator)-0.5f,0};
        particle.p *= 2.0f;

        const float theta = 2*float(M_PI)*distrib(generator);
        particle.v = {std::cos(theta),std::sin(theta),0};
        particle.f = {0,0,0};

        particles.push_back(particle);
    }
}


static void set_gui(std::vector<particle_structure>& particles,timer_basic& timer)
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

    if (ImGui::Button("Add boids"))
        add_boids(particles);
}



#endif
