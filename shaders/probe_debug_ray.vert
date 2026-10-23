#version 460 core 

uniform float u_probe_spacing;
uniform mat4 u_view;
uniform mat4 u_proj;

struct Probe
{
    vec3 indirect;
    vec3 last_ray_dir;
};

layout(std430, binding = 0) buffer IrradianceProbes
{
    Probe probes[];
} probes;

out vec3 v_color;

void main()
{
    
    int instance_id = gl_InstanceID;
    int z = instance_id / (16 * 16);
    int y = (instance_id / 16) % 16;
    int x = instance_id % 16;
    vec3 grid_pos = vec3(float(x), float(y), float(z)) * u_probe_spacing;

    v_color = probes.probes[instance_id].indirect;
    
    float vert_t = float(gl_VertexID%2);
    vec3 probe_rd = probes.probes[instance_id].last_ray_dir;
    if (length(probe_rd) > 99)
    {
        probe_rd *= vec3(0.f);
    }
    vec3 wp = grid_pos + vert_t * probe_rd;

    vec4 clip_center = u_proj * u_view * vec4(wp, 1.0);

    gl_Position = clip_center;
}