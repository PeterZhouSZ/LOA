#version 330 core

layout (location = 0) in vec4 position;

out struct fragment_data
{
    vec4 position;
    vec4 normal;
} fragment;

// model transformation
uniform vec3 p1 = vec3(0.0, 0.0, 0.0);  // initial position
uniform vec3 p2 = vec3(1.0, 0.0, 0.0);  // final position

// view transform
uniform mat4 view;
// perspective matrix
uniform mat4 perspective;

const float PI = 3.14159;

mat3 rotation_between_vector(vec3 a, vec3 b)
{
    const vec3 u0 = normalize(a);
    const vec3 u1 = normalize(b);

    const float d = dot(u0,u1);
    const float angle = acos( d );
    const vec3 axis = normalize(cross(u0,u1));

    const float x = axis.x;
    const float y = axis.y;
    const float z = axis.z;
    const float c = cos(angle);
    const float s = sin(angle);

    return mat3( c+x*x*(1-c), y*x*(1-c)+z*s, z*x*(1-c)-y*s,
                 x*y*(1-c)-z*s, c+y*y*(1-c), z*y*(1-c)+x*s,
                 x*z*(1-c)+y*s, y*z*(1-c)-x*s, c+z*z*(1-c) );
}

void main()
{

    float r = 0.1;

    float u = position.x;
    float v = position.y;

    float L = length(p2-p1);
    vec3 u12 = (p2-p1)/L;

    float x = cos(2*PI*u);
    float y = sin(2*PI*v);
    float z = L*v;

    mat3 R = rotation_between_vector(vec3(0.0,0.0,1.0),u12);

    vec3 n = R*vec3(x,y,0);
    vec3 p = R*vec3(r*x,r*y,z);

    fragment.normal = n;
    fragment.position = p;
    gl_Position = perspective * view * p;
}
