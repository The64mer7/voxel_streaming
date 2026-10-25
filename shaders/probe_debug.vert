#version 460 core 

uniform float u_probe_spacing;
uniform mat4 u_view;
uniform mat4 u_proj;
uniform vec2 u_screen_size;
uniform float u_point_size;

struct Probe
{
    vec3 indirect;
    float depth_moment;
    vec3 last_ray_dir;
    float depth_moment_sq;
};

layout(std430, binding = 0) buffer IrradianceProbes
{
    Probe probes[];
} probes;

const vec2 quad_verts[6] = vec2[](
    vec2(-0.5, -0.5),
    vec2( 0.5, -0.5),
    vec2( 0.5,  0.5),
    vec2(-0.5, -0.5),
    vec2( 0.5,  0.5),
    vec2(-0.5,  0.5)
);

out vec3 v_color;

void main()
{
    vec2 vertex_position = quad_verts[gl_VertexID%6];
    int instance_id = gl_InstanceID;
    int z = instance_id / (16 * 16);
    int y = (instance_id / 16) % 16;
    int x = instance_id % 16;

    Probe probe = probes.probes[instance_id];
    v_color = probe.indirect;
    v_color = vec3((probe.depth_moment_sq - pow(probe.depth_moment, 2.f)));
    vec3 grid_pos = vec3(float(x), float(y), float(z)) * u_probe_spacing;
    vec4 clip_center = u_proj * u_view * vec4(grid_pos, 1.0);
    clip_center.xy += vertex_position * u_point_size * (2.0 / u_screen_size) * clip_center.w;

    gl_Position = clip_center;
}