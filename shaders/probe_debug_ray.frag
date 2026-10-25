#version 460 core 

in vec3 v_color;
out vec4 FragColor;

uniform vec2 u_screen_size;
uniform sampler2D u_depth;
uniform float u_near;
uniform float u_far;

float linear_to_screen_depth(float linear_depth) 
{
    if (linear_depth <= 0.0) return 1.0;
    
    float ndc = ((u_far + u_near) - (2.0 * u_near * u_far) / linear_depth) / (u_far - u_near);
    
    return (ndc + 1.0) * 0.5;
}

void main()
{
    vec2 uv = vec2(gl_FragCoord.xy) / u_screen_size;
    if (gl_FragCoord.z >= linear_to_screen_depth(texture(u_depth, uv).r))
        discard;
        
    FragColor = vec4(v_color, 1.f);
}