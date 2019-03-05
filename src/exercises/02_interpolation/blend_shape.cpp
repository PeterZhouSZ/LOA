
#include "blend_shape.hpp"

#ifdef EXERCISE_BLEND_SHAPE

using namespace vcl;

/** Function returning the index i such that t \in [vt[i],vt[i+1]] */
static size_t index_at_value(float t, const std::vector<float>& vt);

/** Set elements of the gui */
static void set_gui(gui_structure& , timer_interval& timer,bool& display_wireframe,bool& display_body);

/** This function is called before the beginning of the animation loop
    It is used to initialize all part-specific data */
void scene_exercise::setup_data(std::map<std::string,GLuint>& , scene_structure& scene, gui_structure& gui)
{
    std::list<size_t> index_order = {0,1,2,3,4,5,0};

    size_t N = index_order.size();
    keyframe_position.resize(N);
    keyframe_time.resize(N);
    size_t k=0;
    for(size_t index : index_order)
    {
        const std::string filename = "data/face/face_"+zero_fill(std::to_string(index),2)+".obj";
        keyframe_position[k]  = mesh_load_file_obj_read_vertices(filename);
        keyframe_time[k]      = k;
        ++k;
    }

    const size_t N_vertex = keyframe_position[0].size();
    current_position.resize(N_vertex);

    // Set timer bounds
    timer.t_min = 0;
    timer.t_max = N-1-1.0f/100;

    // Prepare the visual representation of the face
    face = mesh_drawable(mesh_load_file_obj("data/face/face_00.obj"));
    body = mesh_drawable(mesh_load_file_obj("data/face/body.obj"));

    scene.camera.translation = {0,-6,0};
    scene.camera.scale = 6;

    gui.show_frame_camera = false;

}


/** This function is called at each frame of the animation loop.
    It is used to compute time-varying argument and perform data data drawing */
void scene_exercise::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui)
{
    timer.update();
    set_gui(gui,timer, display_wireframe, display_body);

    // ********************************************* //
    // Compute interpolated position at time t
    // ********************************************* //
    const float t = timer.t;
    const size_t idx = index_at_value(t, keyframe_time);

    // Nearest keyframe
    const std::vector<vec3>& p1 = keyframe_position[idx];

    const size_t N = p1.size();
    for(size_t k=0; k<N; ++k)
        current_position[k] = p1[k];

    face.data_gpu.update_position(current_position);

    // ********************************************* //
    // Draw results
    // ********************************************* //

    face.draw(shaders["mesh"],scene.camera);
    if(display_body)
        body.draw(shaders["mesh"],scene.camera);

    if(display_wireframe)
    {
        face.draw(shaders["wireframe"],scene.camera);
        if(display_body)
            body.draw(shaders["wireframe"],scene.camera);
    }

}


/** Part specific GUI drawing */
static void set_gui(gui_structure& , timer_interval& timer,bool& display_wireframe,bool& display_body)
{
    ImGui::SliderScalar("Time", ImGuiDataType_Float, &timer.t, &timer.t_min, &timer.t_max, "%.2f s");

    float scale_min = 0.05f;
    float scale_max = 2.0f;
    ImGui::SliderScalar("Time scale", ImGuiDataType_Float, &timer.scale, &scale_min, &scale_max, "%.2f s");

    if (ImGui::Button("Stop"))
        timer.stop();
    if (ImGui::Button("Start"))
        timer.start();

    if( ImGui::Button("Wireframe") )
        display_wireframe = !display_wireframe;
    if( ImGui::Button("Body") )
        display_body = !display_body;

}

static size_t index_at_value(float t, const std::vector<float>& vt)
{
    const size_t N = vt.size();
    assert(vt.size()>=2);
    assert(t>=vt[0]);
    assert(t<vt[N-1]);

    size_t k=0;
    while( vt[k+1]<t )
        ++k;
    return k;
}

#endif
