#pragma once

#include "../../exercises/base_exercise/base_exercise.hpp"

#ifdef EXERCISE_SKINNING

// Helper quaternion structure with associated functions
struct quaternion : vcl::vec4 {
    quaternion();
    quaternion(const vcl::vec4& v);
    quaternion(float x_arg,float y_arg,float z_arg,float w_arg);

    // Build quaternion from axis/angle values
    static quaternion axis_angle(const vcl::vec3& axis, float angle);

    // Apply quaternion to
    vcl::vec3 apply(const vcl::vec3& p) const;
};
vcl::mat3 quaternion_to_mat3(const quaternion& q);
quaternion operator*(float s, const quaternion& q);
quaternion operator*(const quaternion& q, const float s);
quaternion operator+(const quaternion& q1, const quaternion& q2);
quaternion operator*(const quaternion& q1,const quaternion& q2);
quaternion conjugate(const quaternion& q);
quaternion slerp(quaternion q1, const quaternion& q2, float t);


// Connectivity information of a joint in the hierarchy
//  Store parent index, and the current name
struct joint_connectivity
{
    int parent;
    std::string name;
};

// 3D Geometry of a joint (position p, and rotation r)
struct joint_geometry
{
    vcl::vec3 p;
    quaternion r;
};

// Key pose of a joint (Key-time, and geometry at this time)
struct joint_geometry_time
{
    float time;
    joint_geometry geometry;
};

// Storage of the influence of a bone with respect to a vertex (given by the stored weight)
struct skinning_influence
{
    int bone;
    float weight;
};

// Structure storing skeleton data to perform skinning afterward
struct skeleton_structure
{
    std::vector<joint_connectivity> connectivity;           // Connectivity of the skeleton
    std::vector<joint_geometry>     rest_pose;              // Skeleton of the rest expressed in local coordinates
    std::vector<std::vector<joint_geometry_time> > anim;    // Skeleton animation expressed in local coordinates
};

// Storage structure to perform skinning deformation of a surface
struct skinning_structure
{
    std::vector< std::vector<skinning_influence> > influence; // Skinning weights: for each vertex, store all influence values (bone+weight)
    std::vector<vcl::vec3> rest_pose;                         // 3D position of the mesh in rest pose
    vcl::mesh deformed;                                       // Deformed mesh
};

enum gui_parameters_display_type {display_character, display_cylinder};
struct gui_parameters
{
    bool display_skeleton_bones;
    bool display_mesh;
    bool display_rest_pose;
    bool display_wireframe;
    //LOA params
    bool display_spline;
    bool display_input;
    bool display_bodyline;
    bool two_spline;
    int display_type;
};

struct body_line{
    std::vector<int> bones; //indices of the bones
    int root; //index of the root node
    vcl::vec3 xr; //translation of the root node
    std::vector<float> S; //body line coordinates
    std::vector<float> R; //bones' rotations
};

struct single_spline{
    single_spline(vcl::vec3 p, vcl::vec3 q, vcl::vec3 t, vcl::vec3 s):p0(p),p1(q),t0(t),t1(s){}
    single_spline(){}
    vcl::vec3 p0, p1;
    vcl::vec3 t0,t1;
};

struct spline{
    spline(bool two_spl, bool first, single_spline spl1, single_spline spl2):two_splines(two_spl),first(first),spl1(spl1),spl2(spl2){}
    spline(bool two_spl, bool first):two_splines(two_spl),first(first){}
    spline(){}
    bool two_splines;
    bool first;
    vcl::vec3 middle_pt;
    single_spline spl1;
    single_spline spl2;
};

struct scene_exercise : base_scene_exercise
{

    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void set_gui();

    void load_character_data();
    void load_cylinder_data();

    skeleton_structure skeleton;
    skinning_structure skinning;
    vcl::mesh_drawable character_visual;

    vcl::segment_drawable_immediate_mode segment_drawer;
    vcl::mesh_drawable sphere;

    gui_parameters gui_param;

    vcl::timer_interval timer;

    std::vector<joint_geometry> rest_pose;
    std::vector<joint_geometry> current_pose;
    //Spline interppolation
    float alpha = 0.1f; //gradient descent rate
    float epsilon = 0.001f; //precision
    std::vector<vcl::vec3> current_spline;
    vcl::curve_drawable input_stroke;
    vcl::curve_drawable interpolated_LOA;

    //interpolation methods
    vcl::vec3 hermit(float s, vcl::vec3 x0, vcl::vec3 x1, vcl::vec3 t0, vcl::vec3 t1);//evaluate given Hermit polynomial at s
    vcl::vec3 hermit(float s, single_spline spline);
    vcl::vec3 dist_grad_t0(std::vector<vcl::vec3> line, vcl::vec3 t0, vcl::vec3 t1);
    vcl::vec3 dist_grad_t1(std::vector<vcl::vec3> line, vcl::vec3 t0, vcl::vec3 t1);
    std::vector<vcl::vec3> fit_LOA(std::vector<vcl::vec3> line);
    std::vector<vcl::vec3> interpolate_user_input(std::vector<vcl::vec3> line);
    float dist(std::vector<vcl::vec3> line, vcl::vec3 t0, vcl::vec3 t1);
    spline interpolated_spline;
    //body lines
    void compute_body_lines();
    std::vector<body_line> body_lines;
    std::vector<int> terminal_bones;
    int current_body_line = 0;
    //Energies for computing the pose
    void compute_position_energy(const scene_structure& scene, const std::vector<int> bodyline, const std::vector<joint_geometry>& skeleton_geometry,
                                 const std::vector<joint_connectivity>& skeleton_connectivity);
    void compute_body_line_warping(body_line &bodyline);
    void compute_body_line_position(std::vector<joint_geometry> &global,
                                    const std::vector<joint_connectivity> &connectivity);
};






#endif
