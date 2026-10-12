#version 430 core

uniform mat4 u_mvp;
uniform vec3 u_min;
uniform vec3 u_size;

out vec3 v_position;
out vec3 v_normal;
out vec3 v_relative_pos;

const vec3 cube_vertices[36] = vec3[](
    vec3(0,0,1), vec3(1,0,1), vec3(1,1,1),
    vec3(0,0,1), vec3(1,1,1), vec3(0,1,1),
    vec3(1,0,0), vec3(0,0,0), vec3(0,1,0),
    vec3(1,0,0), vec3(0,1,0), vec3(1,1,0),
    vec3(0,0,0), vec3(0,0,1), vec3(0,1,1),
    vec3(0,0,0), vec3(0,1,1), vec3(0,1,0),
    vec3(1,0,1), vec3(1,0,0), vec3(1,1,0),
    vec3(1,0,1), vec3(1,1,0), vec3(1,1,1),
    vec3(0,1,1), vec3(1,1,1), vec3(1,1,0),
    vec3(0,1,1), vec3(1,1,0), vec3(0,1,0),
    vec3(0,0,0), vec3(1,0,0), vec3(1,0,1),
    vec3(0,0,0), vec3(1,0,1), vec3(0,0,1)
);

const vec3 cube_normals[36] = vec3[](
    vec3(0,0,1),  vec3(0,0,1),  vec3(0,0,1),  vec3(0,0,1),  vec3(0,0,1),  vec3(0,0,1),
    vec3(0,0,-1), vec3(0,0,-1), vec3(0,0,-1), vec3(0,0,-1), vec3(0,0,-1), vec3(0,0,-1),
    vec3(-1,0,0), vec3(-1,0,0), vec3(-1,0,0), vec3(-1,0,0), vec3(-1,0,0), vec3(-1,0,0),
    vec3(1,0,0),  vec3(1,0,0),  vec3(1,0,0),  vec3(1,0,0),  vec3(1,0,0),  vec3(1,0,0),
    vec3(0,1,0),  vec3(0,1,0),  vec3(0,1,0),  vec3(0,1,0),  vec3(0,1,0),  vec3(0,1,0),
    vec3(0,-1,0), vec3(0,-1,0), vec3(0,-1,0), vec3(0,-1,0), vec3(0,-1,0), vec3(0,-1,0)
);

void main() 
{
    vec3 local_pos = cube_vertices[gl_VertexID];
    
    vec3 world_pos = u_min + (local_pos * u_size);
    
    v_position = world_pos;
    v_normal = cube_normals[gl_VertexID];
    v_relative_pos = (world_pos - u_min) / u_size;
    
    gl_Position = u_mvp * vec4(world_pos, 1.0);
}