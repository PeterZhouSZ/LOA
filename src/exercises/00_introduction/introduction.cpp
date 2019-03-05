
#include "introduction.hpp"

#ifdef EXERCISE_INTRODUCTION

// Add vcl namespace within the current one - Allows to use function from vcl library without explicitely preceeding their name with vcl::
using namespace vcl;


/** This function is called before the beginning of the animation loop
    It is used to initialize all part-specific data */
void scene_exercise::setup_data(std::map<std::string,GLuint>& , scene_structure& , gui_structure& gui)
{
    // Create a mesh structure containing a quadrangle defined by three points.
    const mesh mesh_plane = mesh_primitive_quad({-2,-1,-2},{-2,-1,2},{2,-1,-2});
    // Convert the mesh structure into object that can be displayed (mesh_drawable)
    plane = mesh_drawable(mesh_plane);

    // Create similarily a cylinder
    const mesh mesh_cylinder = mesh_primitive_cylinder(0.2f,{0,-1,0},{0,1,0},20,20);
    cylinder = mesh_drawable(mesh_cylinder);
    cylinder.uniform_parameter.color = {0.8f,0.8f,1};

    // Create similarily a cube
    const mesh mesh_cube = mesh_primitive_parallelepiped();
    cube = mesh_drawable(mesh_cube);
    cube.uniform_parameter.color = {1,1,0};

    // Create a curve
    // **************************************** //
    std::vector<vec3> curve_cpu; // the basic structure of a curve is a vector of vec3
    const size_t N_curve = 150;
    for(size_t k=0; k<N_curve; ++k)
    {
        const float u = k/(N_curve-1.0f);

        // curve oscillating as a cosine
        const float x = 0;
        const float y = 0.1f * float(std::cos(u*16*M_PI));
        const float z = 4.0f * (u-0.5f);

        curve_cpu.push_back({x,y,z});
    }
    // send data to GPU and store it into a curve_drawable structure
    curve = curve_drawable(curve_cpu);
    // **************************************** //

    // allow by default the display of helper visual frames
    gui.show_frame_worldspace = true;
    gui.show_frame_camera = true;


}

// A function that will be used to update the GUI
static void set_gui(timer_basic& timer);

/** This function is called at each frame of the animation loop.
    It is used to compute time-varying argument and perform data data drawing */
void scene_exercise::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{

    // ********************************************* //
    // Update timer and GUI
    // ********************************************* //

    // Update the GUI with the timer passed as parameter
    set_gui(timer);

    // Update timer to get current time in the animation loop
    timer.update();
    const float time = timer.t;


    // ********************************************* //
    // Display objects
    // ********************************************* //

    // Display plane
    // ********************************************* //

    // the general syntax to display a mesh is:
    //    objectName.draw(shaderName, camera);
    plane.draw(shaders["mesh"],scene.camera);


    // Display cylinder
    // ********************************************* //

    // The cylinder is rotated around the axis (1,0,0), by an angle = time/2
    const vec3 axis_of_rotation = {1,0,0};
    const float angle_of_rotation = time/2.0f;
    // Creation of the 3x3 rotation matrix
    const mat3 rotation = rotation_from_axis_angle_mat3(axis_of_rotation, angle_of_rotation);

    // Set translation and rotation parameters (send onto shaders using uniform variables)
    cylinder.uniform_parameter.translation = {1.5,0,0};
    cylinder.uniform_parameter.rotation = rotation;

    // Display of the cylinder
    cylinder.draw(shaders["mesh"],scene.camera);

    // Meshes can also be displayed as wireframe using the specific shader
    cylinder.draw(shaders["wireframe"],scene.camera);


    // Display cube
    // ********************************************* //
    cube.uniform_parameter.rotation = rotation_from_axis_angle_mat3({0,1,0},std::sin(3*time));
    cube.uniform_parameter.translation = {-1,0,0};
    cube.draw(shaders["mesh"],scene.camera);


    // Display curve
    // ********************************************* //
    curve.uniform_parameter.translation = {1.9f,0,0};
    curve.uniform_parameter.rotation = rotation_from_axis_angle_mat3({0,0,1},time);
    curve.uniform_parameter.color = {0,1,0};
    curve.draw(shaders["curve"],scene.camera);
}

/** Update the visual GUI */
static void set_gui(timer_basic& timer)
{
    // Slider to set time scaling factor
    float scale_min = 0.05f;
    float scale_max = 4.0f;
    ImGui::SliderScalar("Time scale", ImGuiDataType_Float, &timer.scale, &scale_min, &scale_max, "%.2f s");

    // Stop time
    if (ImGui::Button("Stop"))
        timer.stop();
    // Start time
    if (ImGui::Button("Start"))
        timer.start();
}

#endif

