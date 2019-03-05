
#include "sprites.hpp"

#include <random>

#ifdef EXERCISE_SPRITES


// Generator for uniform random number
std::default_random_engine generator;
std::uniform_real_distribution<float> distrib(0,1.0);

using namespace vcl;



static void set_gui(bool& enable_sprite);
bool scene_exercise::enable_sprite = true; // (special initialization for static variable. enable_sprite is set within the GUI, it needs to be static.)


/** This function is called before the beginning of the animation loop
    It is used to initialize all part-specific data */
void scene_exercise::setup_data(std::map<std::string,GLuint>& , scene_structure& , gui_structure& gui)
{
    // Sprite geometry is a simple quadrangle
    sprite = mesh_drawable(mesh_primitive_quad({-1,-1,0},{1,-1,0},{-1,1,0}));
    sprite.uniform_parameter.scaling = 0.6f;

    // The sprite texture .png file with alpha channel used as transparency parameter
    sprite_texture = texture_gpu(image_load_png("data/sprite_smoke/smoke.png"));


    // Set initial position of several particles
    const size_t N_particle = 40;
    for(size_t k=0; k<N_particle; ++k)
    {
        particle_structure particle;

        const float h = distrib(generator);
        const float r = std::sqrt(h);

        const float y = 2*h+0.5f;
        const float x = r*(distrib(generator)*2-1);
        const float z = r*(distrib(generator)*2-1);

        particle.p0 = {x,y,z};
        particles.push_back(particle);
    }

    pipe = mesh_drawable(mesh_primitive_cylinder(0.25f,{0,-2,0},{0,0,0}));

    enable_sprite = true;
    gui.show_frame_camera = false;
}



/** This function is called at each frame of the animation loop.
    It is used to compute time-varying argument and perform data data drawing */
void scene_exercise::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    set_gui(enable_sprite);

    // Display the pipe
    pipe.draw(shaders["mesh"],scene.camera);

    // **************************************** //
    // Sprites display
    // **************************************** //

    // Activate transparency - allow blend based on alpha channel value
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Set texture ID to be the one of the sprite
    glBindTexture(GL_TEXTURE_2D,sprite_texture);

    // Desactivate writing in the Z-buffer (semi-transparent object will not hide other semi-transparent objects)
    glDepthMask(false);

    // Set color of the sprite
    sprite.uniform_parameter.color = {1,1,1};
    // Sprite should not be shaded with lighting - Set ambiant value to 1 and 0 for diffuse and specular
    sprite.uniform_parameter.shading = {1,0,0};

    for(size_t k=0; k<particles.size(); ++k)
    {
        // Sprite should always face the camera
        sprite.uniform_parameter.rotation = scene.camera.orientation;
        sprite.uniform_parameter.translation = particles[k].p0;

        if( enable_sprite )
            sprite.draw(shaders["mesh"], scene.camera);
    }


    // In case we want to show the quads on which sprites are displayed
    if( !enable_sprite )
    {
        // return to standard mesh display
        glDepthMask(true);
        sprite.uniform_parameter.color = {0.1f,0.1f,0.5f};
        sprite.uniform_parameter.shading = {0.2f,0.8f,0.3f};
        glBindTexture(GL_TEXTURE_2D,scene.texture_white);

        for(size_t k=0; k<particles.size(); ++k)
        {
            sprite.uniform_parameter.translation = particles[k].p0;

            sprite.uniform_parameter.rotation = scene.camera.orientation;
            sprite.draw(shaders["mesh"], scene.camera);

            // avoids z-fighting between mesh and wireframe
            glEnable( GL_POLYGON_OFFSET_FILL ); glPolygonOffset( 1.0, 1.0 );

            sprite.draw(shaders["wireframe"], scene.camera);
        }

    }

    // Set the default settings for future drawings
    glDepthMask(true);
    glBindTexture(GL_TEXTURE_2D,scene.texture_white);


}



/** Part specific GUI drawing */
static void set_gui(bool& enable_sprite_param)
{
    ImGui::Checkbox("sprite", &enable_sprite_param);
}


#endif



