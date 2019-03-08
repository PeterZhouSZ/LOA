
#include "skinning.hpp"

#include <random>
#include <fstream>
#include <sstream>

#ifdef EXERCISE_SKINNING

using namespace vcl;

/** Helper function reading skinning data from file */
std::vector<joint_geometry> read_skeleton_geometry(const std::string& filename, float scaling);
std::vector<std::vector<joint_geometry_time> > read_skeleton_animation(const std::string& filename, float scaling);
std::vector<joint_connectivity> read_skeleton_connectivity(const std::string& filename);
std::vector<std::vector<skinning_influence> > read_skinning_influence(const std::string& filename);


vec3 hermit(float s, vec3 x0, vec3 x1, vec3 t0, vec3 t1){
    return pow(1-s,2)*(x0 + s*(t0 + 2*x0)) + pow(s,2)*(x1 - (1-s)*(t1 - 2*x1));
}

float dist(std::vector<vec3> line, vec3 t0, vec3 t1){
    vec3 x0=line[0], x1=line[line.size()-1];
    float d = 0;
    for (int i=0; i<line.size(); i++){
        float s = float(i)/(line.size()-1);
        d += norm(hermit(s, x0, x1, t0, t1)-line[i]);
    }
    return d;
}

vec3 dist_grad_t0(std::vector<vec3> line, vec3 t0, vec3 t1){
    vec3 x0=line[0], x1=line[line.size()-1];
    vec3 grad0(0,0,0);
    for (int i=0; i<line.size(); i++){
        float s = float(i)/(line.size()-1);
        grad0 += 2*pow(1-s,4)*(1+2*s)*s        * x0 +
                 2*pow(s,3)*pow(1-s,2)*(3-2*s) * x1 +
                 2*pow(1-s,4)*pow(s,2)         * t0 -
                 2*pow(1-s,3)*pow(s,3)         * t1 -
                 2*pow(1-s,2)*s                * line[i];
    }
    return grad0;
}

vec3 dist_grad_t1(std::vector<vec3> line, vec3 t0, vec3 t1){
    vec3 x0=line[0], x1=line[line.size()-1];
    vec3 grad1(0,0,0);
    for (int i=0; i<line.size(); i++){
        float s = float(i)/(line.size()-1);
        grad1 += -2*pow(1-s,3)*(1+2*s)*s*s     * x0 -
                 2*pow(s,4)*(1-s)*(3-2*s)      * x1 -
                 2*pow(1-s,3)*pow(s,3)         * t0 +
                 2*pow(1-s,2)*pow(s,4)         * t1 +
                 2*(1-s)*s*s                   * line[i];
    }
    return grad1;
}

std::vector<vec3> fit_LOA(std::vector<vec3> line){
   vec3 t0(2,0,0), t1(2,2,0);
   float d = dist(line, t0, t1);
   float dpred = d+1;
   vec3 grad0=dist_grad_t0(line,t0,t1), grad1=dist_grad_t1(line,t0,t1);
   while ((dpred-d) > 0.01){
       //gradient descent
       t0 = t0 - 0.1*grad0;
       t1 = t1 - 0.1*grad1;
       grad0=dist_grad_t0(line,t0,t1);
       grad1=dist_grad_t1(line,t0,t1);
       dpred=d;
       d = dist(line, t0, t1);
   }
   return std::vector<vec3>({t0,t1});
}

std::vector<vec3> interpolate_user_input(std::vector<vec3> line){
    std::vector<vec3> result = fit_LOA(line);
    vec3 x0=line[0], x1=line[line.size()-1], t0=result[0], t1=result[1];
    std::vector<vec3> interpolated_line;
    for (int i=0; i<line.size(); i++){
        float s = float(i)/(line.size()-1);
        interpolated_line.push_back(hermit(s, x0, x1, t0, t1));
    }
    return interpolated_line;
}

void scene_exercise::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& , gui_structure& )
{
    shaders["segment_immediate_mode"] = create_shader_program("shaders/segment_immediate_mode/segment_immediate_mode.vert.glsl","shaders/segment_immediate_mode/segment_immediate_mode.frag.glsl");
    segment_drawer.init();
    segment_drawer.uniform_parameter.color = {0.0f,0.0f,0.0f};
    glEnable(GL_POLYGON_OFFSET_FILL);

    // Init gui parameters
    gui_param.display_mesh = false;
    gui_param.display_wireframe = false;
    gui_param.display_rest_pose = false;
    gui_param.display_skeleton_bones = true;
    gui_param.display_skeleton_joints = true;
    gui_param.display_type = display_cylinder;

    // Sphere used to display joints
    sphere = mesh_primitive_sphere(0.005f);
    std::vector<vec3> line;
    for (int i=0; i<100; i++){
        float s = float(i)/99;
        line.push_back(vec3(s,s*s+0.05*sin(100*s),0));
    }
    for (int i=0; i<100; i++){
        float s = float(i)/99;
        line.push_back(vec3(1-s,2-(1-s)*(1-s)+0.05*sin(100*s),0));
    }
    //curve = vcl::curve_drawable(line);
    //curve1 = vcl::curve_drawable(interpolate_user_input(line));
    // Load initial cylinder model
    load_cylinder_data();
    compute_body_lines();
}

void scene_exercise::compute_body_lines(){
    std::cout << "Computing Body Lines... " << std::flush;
    terminal_bones.clear();
    body_lines.clear();
    int N = skeleton.rest_pose.size();
    //test for body lines
    int degrees[N];//counters of the degrees of each node, to spot the extremities
    for(size_t k=1; k<N; ++k)
        degrees[k]=0;
    for(size_t k=1; k<N; ++k)
        degrees[skeleton.connectivity[k].parent] +=1;

    for(size_t k=1; k<N; ++k){
        if(degrees[k]==0)
            terminal_bones.push_back(k);
    }
    if(terminal_bones.size()==1)//only one chain, so we count the root node as a terminal bone
        terminal_bones.push_back(0);
    //go through all pairs of terminal bones and compute the complete path to connect them
    for(int i=0 ; i<terminal_bones.size()-1 ; i++){
        for(int j=i+1 ; j<terminal_bones.size() ; j++){
            std::vector<int> body_line;
            std::vector<int> left;
            int node = terminal_bones[i];
            while(node != 0){//go back until the root node
                left.push_back(node);
                node = skeleton.connectivity[node].parent;
            }
            std::vector<int> right;
            node = terminal_bones[j];
            int meeting_node;
            bool stop = false;
            while(node != 0 && !stop){//go back until the root node
                right.push_back(node);
                node = skeleton.connectivity[node].parent;
                for(int n : left){
                    if(node == n){
                        meeting_node = n;
                        stop=true;
                        break;
                    }
                }
            }
            for(int n : left){
                if(n==meeting_node) //we add the nodes until the meeting point and we discard the portion between the meeting point and the root node
                    break;
                body_line.push_back(n);
            }
            for(int n : right)
                body_line.push_back(n);//no filter needed here, since we stopped adding nodes at the meeting point
            body_lines.push_back(body_line);
        }
    }
    std::cout << "Done. " << body_lines.size() << " body lines found." << std::endl;
}

std::vector<joint_geometry> interpolate_skeleton_at_time(float time, const std::vector< std::vector<joint_geometry_time> >& animation)
{

    // Compute skeleton corresponding to a given time from key poses
    // - animation[k] corresponds to the kth animated joint (vector of joint geometry at given time)
    // - animation[k][i] corresponds to the ith pose of the kth joint
    //      - animation[k][i].time         -> corresponding time
    //      - animation[k][i].geometry.p/r -> corresponding position, rotation

    size_t N_joint = animation.size();
    std::vector<joint_geometry> skeleton;
    skeleton.resize(N_joint);
    for(size_t k_joint=0; k_joint<N_joint; ++k_joint)
    {
        const std::vector<joint_geometry_time>& joint_anim = animation[k_joint];

        // Find the index corresponding to the current time
        size_t k_current = 0;
        assert(joint_anim.size()>k_current+1);
        while( time>joint_anim[k_current+1].time )
        {
            ++k_current;
            assert(joint_anim.size()>k_current+1);
        }

        // TO DO ...
        // Compute correct interpolation of joint geometry
        // (the following code corresponds to nearest neighbors, not to a smooth interpolation)
        const joint_geometry current_geometry = joint_anim[k_current].geometry;

        skeleton[k_joint] = current_geometry;
    }

    return skeleton;
}

void compute_skinning(skinning_structure& skinning, const std::vector<joint_geometry>& skeleton_current, const std::vector<joint_geometry>& skeleton_rest_pose)
{
    const size_t N_vertex = skinning.rest_pose.size();
    for(size_t k=0; k<N_vertex; ++k)
    {
        // TO DO ...
        // Compute skinning deformation
        // Change the following line to compute the deformed position from skinning relation
        skinning.deformed.position[k] = skinning.rest_pose[k];
    }
}


// Convert skeleton from local to global coordinates
std::vector<joint_geometry> local_to_global(const std::vector<joint_geometry>& local, const std::vector<joint_connectivity>& connectivity)
{
    const size_t N = connectivity.size();
    assert(local.size()==connectivity.size());
    std::vector<joint_geometry> global;
    global.resize(N);
    global[0] = local[0];

    // T_global = T_global^parent * T_local (T: 4x4 transformation matrix)
    //   => R_global = R_global^parent * R_local
    //   => P_global = R_global^parent * P_local + P_global^parent
    for(size_t k=1; k<N; ++k)
    {
        const int parent = connectivity[k].parent;
        global[k].r = global[parent].r * local[k].r;
        global[k].p = global[parent].r.apply(local[k].p) + global[parent].p;
    }
    return global;
}

void scene_exercise::load_character_data()
{
    const float scaling = 0.005f;

    skeleton.connectivity   = read_skeleton_connectivity("data/marine/skeleton_connectivity");
    skeleton.anim           = read_skeleton_animation("data/marine/skeleton_animation_run", scaling);
    skeleton.rest_pose      = read_skeleton_geometry("data/marine/skeleton_geometry_local", scaling);
    skinning.influence      = read_skinning_influence("data/marine/skinning_data");

    mesh character = mesh_load_file_obj("data/marine/mesh.obj");
    for(vec3& p : character.position) p *= scaling;             // scale vertices of the mesh
    character_visual.data_gpu.clear();
    character_visual = mesh_drawable(character);
    skinning.rest_pose = character.position;
    skinning.deformed  = character;

    timer = timer_interval();
    timer.t_max = 0.733f;
}

void scene_exercise::load_cylinder_data()
{
    skeleton.connectivity = { {-1,"joint_0"},
                              { 0,"joint_1"},
                              { 1,"joint_2"} };


    quaternion q0 = quaternion::axis_angle(normalize(vec3{0,0,1}),0.0f);      // no rotation
    quaternion q1 = quaternion::axis_angle(normalize(vec3{0,0,1}),3.14f/2.0f); // rotation of pi/2 around z-axis

    joint_geometry g0   = {{0.0f,0.0f,0.0f},q0}; // First joint
    joint_geometry g1_0 = {{0.5f,0.0f,0.0f},q0}; // Second joint, frame 0
    joint_geometry g1_1 = {{0.5f,0.0f,0.0f},q1}; // Second joint, frame 1
    joint_geometry g2   = {{0.5f,0.0f,0.0f},q0}; // Third joint (only used to display bone)

    // Skeleton at rest shape
    skeleton.rest_pose = { g0, g1_0, g2 };

    // Anim of cylinder skeleton
    //  Pose 0: straight
    //  Pose 1: First joint is rotated of pi/2 around z axis
    //  Pose 2: straight, similar to pose 0
    std::vector<joint_geometry_time> anim_g0 = {{0,g0},{1,g0},{2,g0}};
    std::vector<joint_geometry_time> anim_g1 = {{0,g1_0},{1,g1_1},{2,g1_0}};
    std::vector<joint_geometry_time> anim_g2 = {{0,g2},{1,g2},{2,g2}};
    skeleton.anim = {anim_g0,anim_g1,anim_g2};

    // Cylinder shape
    mesh cylinder;
    const size_t N=50;
    const float r = 0.1f;
    skinning.influence.clear();
    for(size_t ku=0; ku<N; ++ku)
    {
        for(size_t kv=0; kv<N; ++kv)
        {
            const float u = ku/float(N-1.0f);
            const float v = kv/float(N);

            const float theta = 2*float(M_PI)* v;

            const vec3 p = {u, r*std::cos(theta), r*std::sin(theta)};
            const vec3 n = {0, std::cos(theta), std::sin(theta)};

            cylinder.position.push_back(p);
            cylinder.normal.push_back(n);
        }
    }
    cylinder.connectivity = connectivity_grid(N,N,false,true);


    // Skinning weights
    for(size_t ku=0; ku<N; ++ku)
    {
        const float u = ku/float(N-1.0f);
        for(size_t kv=0; kv<N; ++kv)
        {
            float w0, w1;
            const float alpha = 3.0f; // power for skinning weights evolution
            if(u<0.5f)
            {
                w1 = 0.5f*std::pow(u/0.5f, alpha);
                w0 = 1-w1;
            }
            else
            {
                w0 = 0.5f*std::pow(1-(u-0.5f)/0.5f, alpha);
                w1 = 1-w0;
            }

            skinning_influence influence_bone_0 = {0, w0};
            skinning_influence influence_bone_1 = {1, w1};
            skinning.influence.push_back( {influence_bone_0, influence_bone_1} );
        }
    }

    skinning.rest_pose = cylinder.position;
    skinning.deformed  = cylinder;

    character_visual.data_gpu.clear();
    character_visual = mesh_drawable(cylinder);

    timer = timer_interval();
    timer.t_max = 2.0f;
}

void display_skeleton(const std::vector<joint_geometry>& skeleton_geometry,
                      const std::vector<joint_connectivity>& skeleton_connectivity,
                      const std::map<std::string,GLuint>& shaders,
                      const scene_structure& scene,
                      segment_drawable_immediate_mode& segment_drawer)
{
    const size_t N = skeleton_geometry.size();
    for(size_t k=1; k<N; ++k)
    {
        int parent = skeleton_connectivity[k].parent;
        const vec3& p1 = skeleton_geometry[parent].p;
        const vec3& p2 = skeleton_geometry[k].p;
        segment_drawer.uniform_parameter.p1 = p1;
        segment_drawer.uniform_parameter.p2 = p2;
        segment_drawer.draw(shaders.at("segment_immediate_mode"),scene.camera);
    }
}

void display_joints(const std::vector<joint_geometry>& skeleton_geometry,
                    const std::map<std::string,GLuint>& shaders,
                    const scene_structure& scene,
                    mesh_drawable& sphere)
{
    const size_t N = skeleton_geometry.size();
    for(size_t k=0; k<N; ++k)
    {
        sphere.uniform_parameter.translation = skeleton_geometry[k].p;
        sphere.draw(shaders.at("mesh"),scene.camera);
    }
}

void display_bodyline(const std::vector<int> bodyline,
                      const std::vector<joint_geometry>& skeleton_geometry,
                      const std::vector<joint_connectivity>& skeleton_connectivity,
                      const std::map<std::string,GLuint>& shaders,
                      const scene_structure& scene,
                      segment_drawable_immediate_mode& segment_drawer)
{
    const size_t N = skeleton_geometry.size();
    for(size_t k : bodyline)
    {
        int parent = skeleton_connectivity[k].parent;
        const vec3& p1 = skeleton_geometry[parent].p;
        const vec3& p2 = skeleton_geometry[k].p;
        segment_drawer.uniform_parameter.p1 = p1;
        segment_drawer.uniform_parameter.p2 = p2;
        segment_drawer.draw(shaders.at("segment_immediate_mode"),scene.camera);
    }
}


void scene_exercise::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    timer.update();
    set_gui();
    const float t = timer.t;

    const auto skeleton_geometry_local  = interpolate_skeleton_at_time(t, skeleton.anim);
    const auto skeleton_rest_pose = local_to_global(skeleton.rest_pose, skeleton.connectivity);
    auto skeleton_current = local_to_global(skeleton_geometry_local, skeleton.connectivity);

    if(gui_param.display_rest_pose)
        skeleton_current = skeleton_rest_pose;

    compute_skinning(skinning, skeleton_current, skeleton_rest_pose);
    character_visual.data_gpu.update_position(skinning.deformed.position);

    normal(skinning.deformed.position, skinning.deformed.connectivity, skinning.deformed.normal);
    character_visual.data_gpu.update_normal(skinning.deformed.normal);

    if(scene.update_body_line){
        current_body_line = (current_body_line+1)%body_lines.size();
        scene.update_body_line = false;
    }
    display_bodyline(body_lines[current_body_line], skeleton_current, skeleton.connectivity, shaders, scene, segment_drawer);

    //curve.draw(shaders["mesh"],scene.camera);
    //curve1.draw(shaders["mesh"],scene.camera);


    if(gui_param.display_skeleton_bones)
        display_skeleton(skeleton_current, skeleton.connectivity, shaders, scene, segment_drawer);
    if(gui_param.display_skeleton_joints)
        display_joints(skeleton_current,shaders, scene, sphere);

    if(gui_param.display_mesh) {
        glPolygonOffset( 1.0, 1.0 );
        character_visual.draw(shaders["mesh"],scene.camera);
    }
    if(gui_param.display_wireframe) {
        glPolygonOffset( 1.0, 1.0 );
        character_visual.draw(shaders["wireframe"],scene.camera);
    }

    if(scene.update_curve){
        if(two_splines)
        {
            std::vector<vec3> s1, s2;
            for(size_t j=0 ; j<=scene.draw_points.size()/2 ; j++)
                s1.push_back(scene.draw_points[j]);
            for(size_t j=scene.draw_points.size()/2 ; j<scene.draw_points.size() ; j++)
                s2.push_back(scene.draw_points[j]);
            current_spline = interpolate_user_input(s1);
            for(vcl::vec3 v : interpolate_user_input(s2))
                current_spline.push_back(v);
        }else
        current_spline = interpolate_user_input(scene.draw_points);
        scene.update_curve = false;
    }
    interpolated_LOA = vcl::curve_drawable(current_spline);
    input_stroke = vcl::curve_drawable(scene.draw_points);
    //display the user's stroke
    input_stroke.draw(shaders["mesh"], scene.camera);
    //if possible, diplay the interpolated spline
    if(!current_spline.empty())
        interpolated_LOA.draw(shaders["mesh"], scene.camera);
}


void scene_exercise::set_gui()
{
    // Can set the speed of the animation
    float scale_min = 0.05f;
    float scale_max = 2.0f;

    ImGui::SliderScalar("Timer", ImGuiDataType_Float, &timer.t, &timer.t_min, &timer.t_max, "%.2f s");
    ImGui::SliderScalar("Time scale", ImGuiDataType_Float, &timer.scale, &scale_min, &scale_max, "%.2f s");

    ImGui::Checkbox("Skeleton bones", &gui_param.display_skeleton_bones);
    ImGui::Checkbox("Skeleton joints", &gui_param.display_skeleton_joints);
    ImGui::Checkbox("Mesh", &gui_param.display_mesh);
    ImGui::Checkbox("Wireframe", &gui_param.display_wireframe);
    ImGui::Checkbox("Rest pose", &gui_param.display_rest_pose);
    if(ImGui::RadioButton("Cylinder", &gui_param.display_type,display_cylinder)){
        load_cylinder_data();
        compute_body_lines();
    }

    ImGui::SameLine();
    if(ImGui::RadioButton("Character", &gui_param.display_type,display_character)){
        load_character_data();
        compute_body_lines();
    }

    // Start and stop animation
    if (ImGui::Button("Stop"))
        timer.stop();
    ImGui::SameLine();
    if (ImGui::Button("Start"))
        timer.start();
}


std::vector<std::vector<skinning_influence> > read_skinning_influence(const std::string& filename)
{
    std::vector<std::vector<skinning_influence> > influence;

    std::ifstream fid(filename);

    // first line = number of influence per pertex (fixed for all vertices)
    size_t N_bone_influence=0;
    fid >> N_bone_influence;

    assert(fid.good());
    assert(N_bone_influence>0 && N_bone_influence<=6);

    // Read influence associated to each vertex
    std::vector<skinning_influence> skinning_vertex;
    skinning_vertex.resize(N_bone_influence);

    while(fid.good())
    {
        // read list of [bone index] [skinning weights]
        for(size_t k=0; k<N_bone_influence && fid.good(); ++k)
            fid >> skinning_vertex[k].bone >> skinning_vertex[k].weight;

        if(fid.good())
            influence.push_back(skinning_vertex);
    }

    fid.close();


    // normalize skinning weights (sum up to one)
    for(size_t kv=0; kv<influence.size(); ++kv)
    {
        float w = 0.0f;
        // compute weight sum
        for(size_t k=0; k<N_bone_influence; ++k)
            w += influence[kv][k].weight;
        // normalize
        if(w>1e-5f)
            for(size_t k=0; k<N_bone_influence; ++k)
                influence[kv][k].weight /= w;

    }

    return influence;
}

std::vector<std::vector<joint_geometry_time> > read_skeleton_animation(const std::string& filename, float scaling)
{
    std::vector<std::vector<joint_geometry_time> > skeleton;

    std::ifstream fid(filename);
    while(fid.good())
    {
        size_t N_key=0;
        fid >> N_key;

        if(fid.good())
        {
            std::vector<joint_geometry_time> animated_joint;
            animated_joint.resize(N_key);

            for(size_t k_key=0; k_key<N_key; ++k_key)
            {
                float key_time;
                vec3 p;
                vec4 q;

                fid >> key_time;
                fid >> p.x >> p.y >> p.z;
                fid >> q.x >> q.y >> q.z >> q.w;

                q = normalize(q);

                animated_joint[k_key] = {key_time, {p*scaling,q} };
            }

            skeleton.push_back(animated_joint);
        }
    }

    fid.close();

    return skeleton;
}

std::vector<joint_connectivity> read_skeleton_connectivity(const std::string& filename)
{
    std::vector<joint_connectivity> skeleton;

    std::ifstream fid(filename);

    while(fid.good())
    {
        std::string line;
        std::getline(fid, line);

        if(fid.good())
        {
            std::stringstream sstream(line);

            int k;
            int parent;
            std::string name;

            sstream >> k >> parent >> name;

            skeleton.push_back({parent,name});
        }
    }

    fid.close();

    return skeleton;
}

std::vector<joint_geometry> read_skeleton_geometry(const std::string& filename, float scaling)
{
    std::vector<joint_geometry> skeleton;

    std::ifstream fid(filename);
    while(fid.good())
    {
        std::string line;
        std::getline(fid, line);

        if(fid.good())
        {
            std::stringstream sstream(line);

            vec3 p;
            quaternion q;

            sstream >> p.x >> p.y >> p.z;
            sstream >> q.x >> q.y >> q.z >> q.w;

            q = normalize(q);

            skeleton.push_back({p*scaling,q});
        }
    }

    fid.close();

    return skeleton;
}


quaternion::quaternion(float x_arg,float y_arg,float z_arg,float w_arg)
    :vec<4>({x_arg,y_arg,z_arg,w_arg})
{}

quaternion::quaternion()
    :vec<4>(1,0,0,0)
{}
quaternion::quaternion(const vec4& v)
    :vec<4>(v)
{
}

quaternion operator*(const quaternion& q1,const quaternion& q2)
{
    return {q1.x*q2.w + q1.w*q2.x + q1.y*q2.z - q1.z*q2.y,
                q1.y*q2.w + q1.w*q2.y + q1.z*q2.x - q1.x*q2.z,
                q1.z*q2.w + q1.w*q2.z + q1.x*q2.y - q1.y*q2.x,
                q1.w*q2.w-q1.x*q2.x-q1.y*q2.y-q1.z*q2.z};
}
vcl::vec3 quaternion::apply(const vcl::vec3& p) const
{
    const quaternion q_p    = quaternion(p.x,p.y,p.z,0);
    const quaternion q_conj = {-x,-y,-z,w};

    const quaternion q_res  = (*this)*q_p*q_conj;
    return {q_res.x,q_res.y,q_res.z};
}
quaternion quaternion::axis_angle(const vcl::vec3& axis, float angle)
{
    const float c = std::cos(angle/2.0f);
    const float s = std::sin(angle/2.0f);
    return quaternion(axis.x*s, axis.y*s, axis.z*s, c);
}
quaternion conjugate(const quaternion& q)
{
    return {-q.x,-q.y,-q.z, q.w};
}

quaternion operator*(float s, const quaternion& q)
{
    return {s*q.x, s*q.y, s*q.z, s*q.w};
}
quaternion operator*(const quaternion& q, const float s)
{
    return s*q;
}
quaternion operator+(const quaternion& q1, const quaternion& q2)
{
    return {q1.x+q2.x, q1.y+q2.y, q1.z+q2.z, q1.w+q2.w};
}

quaternion slerp(quaternion q1, const quaternion& q2, float t)
{
    float cos_omega = dot(q1,q2);

    // perform linear interpolation for very small angle (avoid division by sin(omega) in this case)
    const float epsilon = 1e-5f;
    if( std::abs(cos_omega-1.0f)<epsilon )
    {
        quaternion q = (1-t)*q1+t*q2;
        q = normalize(q);
        return q;
    }

    // make sure we take the shortest interpolating path
    // (If you are using slerp interpolation, you may comment this part and look at the result on the running character)
    if( cos_omega<0 )
    {
        q1=-q1;
        cos_omega = -cos_omega;
    }


    const float omega = std::acos(cos_omega);
    quaternion q = std::sin( (1-t)*omega )/std::sin(omega)*q1 + std::sin(t*omega)/std::sin(omega)*q2;

    return q;
}



mat3 quaternion_to_mat3(const quaternion& q)
{
    const float x=q.x, y=q.y, z=q.z, w=q.w;
    return mat3(1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
                2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
                2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y));
}

#endif
