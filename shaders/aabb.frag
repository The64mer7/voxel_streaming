#version 430 core

in vec3 v_position;
in vec3 v_normal;
in vec3 v_relative_pos;

out vec4 frag_color;

void main() 
{
    frag_color = vec4(v_relative_pos, 1.0);
}