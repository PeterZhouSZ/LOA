
#include "interpolation_rotation.hpp"

#ifdef EXERCISE_INTERPOLATION_ROTATION

using namespace vcl;


/** Set elements of the gui */
static void set_gui(gui_structure& gui, timer_interval_reversing& timer);

/** This function is called before the beginning of the animation loop
    It is used to initialize all part-specific data */
void scene_exercise::setup_data(std::map<std::string,GLuint>& , scene_structure& , gui_structure& gui)
{
    translation_1 = {0,0,-0.5};
    axis_1  = {0,0,1};
    angle_1 = 0.0f;

    translation_2 = {0,0,0.5f};
    axis_2  = normalize(vec3{1,1,0});
    angle_2 = float(M_PI);

    frame = mesh_drawable(mesh_primitive_frame());
    frame.uniform_parameter.scaling = 0.35f;

    gui.show_frame_camera = false;

}


/** This function is called at each frame of the animation loop.
    It is used to compute time-varying argument and perform data data drawing */
void scene_exercise::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui)
{
    timer.update();
    set_gui(gui,timer);

    const vec3 tr1 = translation_1;
    const vec3 tr2 = translation_2;

    const mat3 R1 = rotation_from_axis_angle_mat3(axis_1,angle_1);
    const mat3 R2 = rotation_from_axis_angle_mat3(axis_2,angle_2);

    const float alpha = timer.t;

    // Linear interpolation of translation
    const vec3 tr = (1-alpha)*tr1 + alpha*tr2;

    // Linear interpolation of matrix
    const mat3 R = (1-alpha)*R1 + alpha*R2;

    frame.uniform_parameter.translation = tr1;
    frame.uniform_parameter.rotation = R1;
    frame.draw(shaders["mesh"],scene.camera);

    frame.uniform_parameter.translation = tr2;
    frame.uniform_parameter.rotation = R2;
    frame.draw(shaders["mesh"],scene.camera);

    frame.uniform_parameter.translation = tr;
    frame.uniform_parameter.rotation = R;
    frame.draw(shaders["mesh"],scene.camera);
}


/** Part specific GUI drawing */
static void set_gui(gui_structure& , timer_interval_reversing& timer)
{
    ImGui::SliderScalar("Time", ImGuiDataType_Float, &timer.t, &timer.t_min, &timer.t_max, "%.2f s");

    float scale_min = 0.05f;
    float scale_max = 2.0f;
    ImGui::SliderScalar("Time scale", ImGuiDataType_Float, &timer.scale, &scale_min, &scale_max, "%.2f s");

    if (ImGui::Button("Stop"))
        timer.stop();
    if (ImGui::Button("Start"))
        timer.start();
}



#endif
