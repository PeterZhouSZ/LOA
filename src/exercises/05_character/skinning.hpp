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
    bool display_skeleton_joints;
    bool display_mesh;
    bool display_rest_pose;
    bool display_wireframe;
    int display_type;
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
};








#endif
