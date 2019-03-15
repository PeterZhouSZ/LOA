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

std::vector<joint_geometry> local_to_global(const std::vector<joint_geometry>& local, const std::vector<joint_connectivity>& connectivity);

vec3 scene_exercise::hermit(float s, vec3 x0, vec3 x1, vec3 t0, vec3 t1){//evaluate cubic spline at position s
    return pow(1-s,2)*(x0 + s*(t0 + 2*x0)) + pow(s,2)*(x1 - (1-s)*(t1 - 2*x1));
}

vec3 scene_exercise::hermit(float s, single_spline spline){
    return hermit(s, spline.p0, spline.p1, spline.t0, spline.t1);
}

float scene_exercise::dist(std::vector<vec3> line, vec3 t0, vec3 t1){
    vec3 x0=line[0], x1=line[line.size()-1];
    float d = 0;
    for (int i=0; i<line.size(); i++){
        float s = float(i)/(line.size()-1);
        d += norm(hermit(s, x0, x1, t0, t1)-line[i]);
    }
    return d;
}

vec3 scene_exercise::dist_grad_t0(std::vector<vec3> line, vec3 t0, vec3 t1){
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

vec3 scene_exercise::dist_grad_t1(std::vector<vec3> line, vec3 t0, vec3 t1){
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

std::vector<vec3> scene_exercise::fit_LOA(std::vector<vec3> line){
   vec3 t0(2,0,0), t1(2,2,0);
   float d = dist(line, t0, t1);
   float dpred = d+1;
   vec3 grad0=dist_grad_t0(line,t0,t1), grad1=dist_grad_t1(line,t0,t1);
   while ((dpred-d) > epsilon){
       //gradient descent
       t0 = t0 - alpha*grad0;
       t1 = t1 - alpha*grad1;
       grad0=dist_grad_t0(line,t0,t1);
       grad1=dist_grad_t1(line,t0,t1);
       dpred=d;
       d = dist(line, t0, t1);
   }
   return std::vector<vec3>({t0,t1});
}

std::vector<vec3> scene_exercise::interpolate_user_input(std::vector<vec3> line){
    std::vector<vec3> result = fit_LOA(line);
    vec3 x0=line[0], x1=line[line.size()-1], t0=result[0], t1=result[1];
    std::vector<vec3> interpolated_line;
    for (int i=0; i<line.size(); i++){
        float s = float(i)/(line.size()-1);
        interpolated_line.push_back(hermit(s, x0, x1, t0, t1));
    }
    if (gui_param.two_spline){
        if (interpolated_spline.first) interpolated_spline.spl1 = single_spline(x0, x1, t0, t1);
        else interpolated_spline.spl2 = single_spline(x0, x1, t0, t1);
    }
    else
        interpolated_spline.spl1 = single_spline(x0, x1, t0, t1);
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
    gui_param.display_bodyline = true;
    gui_param.display_input = true;
    gui_param.display_spline = true;
    gui_param.display_type = display_cylinder;

    // Sphere used to display joints
    sphere = mesh_primitive_sphere(0.01f);
    std::vector<vec3> line;
    for (int i=0; i<100; i++){
        float s = float(i)/99;
        line.push_back(vec3(s,s*s+0.05*sin(100*s),0));
    }
    for (int i=0; i<100; i++){
        float s = float(i)/99;
        line.push_back(vec3(1-s,2-(1-s)*(1-s)+0.05*sin(100*s),0));
    }
    //only display the character
    load_character_data();
    compute_body_lines();
    rest_pose = local_to_global(skeleton.rest_pose, skeleton.connectivity);
    current_pose = rest_pose; // static for now
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
            body_line bodyline;
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
            while(node != 0 && !stop){//go back until the meeting node
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
                bodyline.bones.push_back(n);
                if(n==meeting_node) //we add the nodes until the meeting point and we discard the portion between the meeting point and the root node
                    break;
            }
            bodyline.root = meeting_node;//the last index we added was that of the root node
            for(int i=right.size()-1 ; i>=0 ; i--) // reverse order, so the bodyline is ordered from one end to the other
                bodyline.bones.push_back(right[i]);//no filter needed here, since we stopped adding nodes at the meeting point
            body_lines.push_back(bodyline);
        }
    }
    std::cout << "Done. " << body_lines.size() << " body lines found." << std::endl;
}

void scene_exercise::compute_body_line_warping(body_line &bodyline){
    bodyline.S.clear();
    float S = 0.0f;
    bodyline.S.push_back(S);
    for(int bone : bodyline.bones){
        if(bone != bodyline.root){
            vec3 p0 = rest_pose[bone].p;
            vec3 p1 = rest_pose[skeleton.connectivity[bone].parent].p;
            S += norm(p1-p0); //size of the segment
            bodyline.S.push_back(S);
        }
    }
}

void scene_exercise::compute_body_line_position(std::vector<joint_geometry>& global,
                                                const std::vector<joint_connectivity>& connectivity){
    float s0 = 0.0f;
    float s1;
    vec3 p0 = hermit(s0, interpolated_spline.spl1);
    vec3 p1;
    float real_dist, target_dist;
    float step = 0.001f;
    single_spline current_spline =  interpolated_spline.spl1;

    body_line bodyline = body_lines[current_body_line];
    vec3 spl0 = interpolated_spline.spl1.p0;
    //find which end of the bodyline is closest to the LOA
    float d0 = norm(spl0-global[bodyline.bones[0]].p);
    float d2 = norm(spl0-global[bodyline.bones[bodyline.bones.size()-1]].p);
    if(d0>d2){
        //the beginning of the LOA and that of the bodyline are inverted, we need to invert the body line
        std::vector<int> inverted_bodyline;
        for(int bone=bodyline.bones.size()-1 ; bone>=0 ; bone--)
            inverted_bodyline.push_back(bodyline.bones[bone]);
        bodyline.bones = inverted_bodyline;
    }
    compute_body_line_warping(bodyline);
    //translation array, used to compute that of each bone in the body lines (the other bones will be translated by the translation of the closest parent in the body line, see below)
    std::vector<vec3> translations;
    for(int i=0 ; i<global.size() ; i++)
        translations.push_back(vec3(0.0f,0.0f,0.0f));
    for(int i=0 ; i<bodyline.bones.size()-1 ; i++){
        translations[bodyline.bones[i]] = p0-global[bodyline.bones[i]].p;
        //compute the next bone position
        s1 = s0 + step;
        p1 = hermit(s1, current_spline);
        target_dist = bodyline.S[i+1] - bodyline.S[i];//actual length of the bone
        real_dist = norm(p1-p0);//length of the chord
        if (gui_param.two_spline){
            while(real_dist<target_dist && s1 < 1.0f-step){
                s1 += step;
                p1 = hermit(s1, current_spline);
                real_dist = norm(p1-p0);
            }
            if(s1 >= 1.0f-step){
                current_spline = interpolated_spline.spl2;
                s1 = 0.0f;
                while(real_dist<target_dist){
                    s1 += step;
                    p1 = hermit(s1, current_spline);
                    real_dist = norm(p1-p0);
                }
            }
        } else {
            while(real_dist<target_dist){
                s1 += step;
                p1 = hermit(s1, current_spline);
                real_dist = norm(p1-p0);
            }
        }
        p0=p1;
        s0=s1;
    }
    translations[bodyline.bones[bodyline.bones.size()-1]] = p0-global[bodyline.bones[bodyline.bones.size()-1]].p;
    //determine which bones are in the bodyline
    std::vector<bool> in_chain;
    for(int i=0 ; i<global.size() ; i++)
        in_chain.push_back(false);
    for(int bone : bodyline.bones)
        in_chain[bone] = true;
    //translate all bones accordingly
    for(int i=0 ; i<global.size() ; i++){
        int node=i;
        if(!in_chain[i]){
            //we look for the closest parent in the body line
            while(!in_chain[node] && node!=0){
                node = connectivity[node].parent;
            }
            if(node==0)
                node = bodyline.root;
        }
        global[i].p += translations[node];
    }
    //compute the quaternion rotations for the skinning
    global[0].r = rest_pose[0].r;
    for(int i=1 ; i<global.size() ; i++){
        int parent = connectivity[i].parent;
        global[parent].r = compute_rotation(rest_pose[i].p - rest_pose[parent].p, global[i].p-global[parent].p) * rest_pose[parent].r;
    }
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
        const std::vector<skinning_influence>& influence = skinning.influence[k];

        // Transformation matrix for skinning
        mat4 M = mat4::zero();
        for(size_t kb=0; kb<influence.size(); ++kb) // Loop over influencing bones
        {
            const int idx = influence[kb].bone;
            const float w = influence[kb].weight;

            const quaternion& r = skeleton_current[idx].r;
            const vec3& p = skeleton_current[idx].p;
            const quaternion& r0 = skeleton_rest_pose[idx].r;
            const vec3& p0 = skeleton_rest_pose[idx].p;

            // Convert rotation/translation to matrix
            mat4 T = mat4::from_mat3_vec3(quaternion_to_mat3(r), p);
            mat4 T0_inv = mat4::from_mat3_vec3(quaternion_to_mat3(conjugate(r0)), conjugate(r0).apply(-p0)); // inverse

            // Skinning
            M += w*T*T0_inv;
        }

        // Apply skinning transform on vertex
        const vec3& p0 = skinning.rest_pose[k];
        const vec4 p1 = M * vec4(p0.x,p0.y,p0.z,1.0f);
        skinning.deformed.position[k] = {p1.x,p1.y,p1.z};

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

void display_bodyline(body_line bodyline,
                      const std::vector<joint_geometry>& skeleton_geometry,
                      const std::map<std::string,GLuint>& shaders,
                      const scene_structure& scene,
                      mesh_drawable& sphere)
{
    for(size_t k : bodyline.bones)
    {
        const vec3& p = skeleton_geometry[k].p;
        sphere.uniform_parameter.translation = p;
        sphere.draw(shaders.at("mesh"),scene.camera);
    }
}

void scene_exercise::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    timer.update();
    set_gui();
    const float t = timer.t;

    auto skeleton_current = (gui_param.display_rest_pose ? rest_pose : current_pose);
    compute_skinning(skinning, skeleton_current, rest_pose);
    character_visual.data_gpu.update_position(skinning.deformed.position);

    normal(skinning.deformed.position, skinning.deformed.connectivity, skinning.deformed.normal);
    character_visual.data_gpu.update_normal(skinning.deformed.normal);

    if(scene.update_body_line){//select another body line, triggered by hitting "B"
        current_body_line = (current_body_line+1)%body_lines.size();
        scene.update_body_line = false;
    }

    if(scene.update_curve){//compute the interpolated LOA and the associated pose, triggered by hitting "C"
        interpolated_spline = spline(gui_param.two_spline, true);
        if(gui_param.two_spline){
            std::vector<vec3> s1, s2;
            for(size_t j=0 ; j<=scene.draw_points.size()/2 ; j++)
                s1.push_back(scene.draw_points[j]);
            for(size_t j=scene.draw_points.size()/2 ; j<scene.draw_points.size() ; j++)
                s2.push_back(scene.draw_points[j]);
            current_spline = interpolate_user_input(s1);
            interpolated_spline.middle_pt = current_spline[current_spline.size()-1];
            interpolated_spline.first = false;
            for(vcl::vec3 v : interpolate_user_input(s2))
                current_spline.push_back(v);
        } else
        current_spline = interpolate_user_input(scene.draw_points);
        compute_body_line_position(current_pose, skeleton.connectivity);
        scene.update_curve = false;
    }

    interpolated_LOA = vcl::curve_drawable(current_spline);
    input_stroke = vcl::curve_drawable(scene.draw_points);
    //display the user's stroke
    if(gui_param.display_input)
        input_stroke.draw(shaders["mesh"], scene.camera);
    //if possible, diplay the interpolated spline
    if(gui_param.display_spline && !current_spline.empty())
        interpolated_LOA.draw(shaders["mesh"], scene.camera);
    //display the skeleton
    if(gui_param.display_skeleton_bones)
        display_skeleton(skeleton_current, skeleton.connectivity, shaders, scene, segment_drawer);
    //display the character
    if(gui_param.display_mesh) {
        glPolygonOffset( 1.0, 1.0 );
        character_visual.draw(shaders["mesh"],scene.camera);
    }
    //display the geometry
    if(gui_param.display_wireframe) {
        glPolygonOffset( 1.0, 1.0 );
        character_visual.draw(shaders["wireframe"],scene.camera);
    }
    //display the chosen bodyline
    if(gui_param.display_bodyline)
        display_bodyline(body_lines[current_body_line], skeleton_current, shaders, scene, sphere);
}


void scene_exercise::set_gui()
{
    // Can set the speed of the animation
    float scale_min = 0.05f;
    float scale_max = 2.0f;

    ImGui::Checkbox("Skeleton bones", &gui_param.display_skeleton_bones);
    ImGui::Checkbox("Mesh", &gui_param.display_mesh);
    ImGui::Checkbox("Wireframe", &gui_param.display_wireframe);
    ImGui::Checkbox("Rest pose", &gui_param.display_rest_pose);
    // LOA Parameters
    ImGui::Checkbox("2-Spline Interpolation", &gui_param.two_spline);
    ImGui::Checkbox("Interpolated spline", &gui_param.display_spline);
    ImGui::Checkbox("User input", &gui_param.display_input);
    ImGui::Checkbox("Body Line", &gui_param.display_bodyline);

    // Start and stop animation
    if (ImGui::Button("Reset"))
        current_pose = rest_pose;
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

quaternion compute_rotation(const vec3 &v1, const vec3 &v2){
    float cos_theta = dot(normalize(v1), normalize(v2));
    float angle = acos(cos_theta);
    vec3 w = cross(v1, v2);
    if(norm(w) < 0.0001f)
        angle = 0.0f;
    else
        w = normalize(w);
    return quaternion::axis_angle(w, angle);
}

mat3 quaternion_to_mat3(const quaternion& q)
{
    const float x=q.x, y=q.y, z=q.z, w=q.w;
    return mat3(1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
                2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
                2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y));
}

#endif
