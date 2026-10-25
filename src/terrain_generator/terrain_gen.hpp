#pragma once
#include <glm/glm.hpp>

class TerrainGen
{
public:
    static float hash(glm::vec2 st)
    {
        return glm::fract(sin(dot(st, glm::vec2(12.9898, 78.233))) * 43758.5453123);
    }

    static float hash(glm::vec3 st)
    {
        return glm::fract(sin(dot(st, glm::vec3(12.9898, 78.233, 37.719))) * 43758.5453123);
    }

    static float fade(float x)
    {
        return 6.f * (pow(x, 3.f) * 1.f / 3.f + 0.5f * pow(x, 4.f) + pow(x, 3.f) * 1.f / 3.f);
    }

    static glm::vec2 fade(glm::vec2 x)
    {
        return 6.f * (pow(x, glm::vec2(3.f)) * 1.f / 3.f - 0.5f * pow(x, glm::vec2(4.f)) +
                      pow(x, glm::vec2(3.f)) * 1.f / 3.f);
    }

    static float smooth_step(float x) { return x * x * (3.0 - 2.0 * x); }

    static glm::vec2 smooth_step(glm::vec2 x) { return x * x * (glm::vec2(3.f) - 2.f * x); }

    static glm::vec3 smooth_step(glm::vec3 x) { return x * x * (glm::vec3(3.f) - 2.f * x); }

    static float value_noise(glm::vec2 p)
    {
        glm::vec2 p00 = floor(p);
        glm::vec2 p01 = p00 + glm::vec2(0, 1);
        glm::vec2 p10 = p00 + glm::vec2(1, 0);
        glm::vec2 p11 = p00 + glm::vec2(1, 1);

        float h00 = hash(p00);
        float h01 = hash(p01);
        float h10 = hash(p10);
        float h11 = hash(p11);

        glm::vec2 t = glm::fract(p);
        glm::vec2 ft = smooth_step(t);
        glm::vec2 n = mix(glm::vec2(h00, h01), glm::vec2(h10, h11), ft.x);
        return glm::mix(n.x, n.y, ft.y);
    }

    static float value_noise(glm::vec3 p)
    {
        glm::vec3 orig = floor(p);
        glm::vec3 t = glm::fract(p);
        glm::vec3 ft = smooth_step(t);
        glm::vec3 ift = 1.f - ft;

        glm::vec3 inv_ft_ft[2];
        inv_ft_ft[0] = ift;
        inv_ft_ft[1] = ft;

        float n = 0;
        for (int z = 0; z < 2; z++)
            for (int y = 0; y < 2; y++)
                for (int x = 0; x < 2; x++)
                {
                    glm::vec3 corner_offset = orig + glm::vec3(x, y, z);
                    n += hash(corner_offset) * inv_ft_ft[x].x * inv_ft_ft[y].y * inv_ft_ft[z].z;
                }
        return n;
    }

    static float sd_noise_conservative(glm::vec3 p, float scale, float amp)
    {
        float noise = value_noise(p * scale) * 2.f - 1.f;
        return noise * amp;
    }

    static glm::vec2 minmax_noise(glm::vec3 p, glm::vec3 half_size, float scale, float amp)
    {
        float noise = sd_noise_conservative(p, scale, amp);

        float lipschitz = 2.f * 1.5f * scale * amp;

        float max_dist = glm::length(half_size);
        float min_v = glm::max(noise - lipschitz * max_dist, -amp);
        float max_v = glm::min(noise + lipschitz * max_dist, +amp);

        return glm::vec2(min_v, max_v);
    }

    static float sd_gradient_y(glm::vec3 p, float f) { return p.y * f; }

    static glm::vec2 minmax_gradient_y(glm::vec3 p, glm::vec3 half_size, float f)
    {
        float min_v = (p.y - half_size.y) * f;
        float max_v = (p.y + half_size.y) * f;
        return glm::vec2(min_v, max_v);
    }

    static glm::vec2 op_add(glm::vec2 v0, glm::vec2 v1) { return v0 + v1; }

    static glm::vec2 op_minmax(glm::vec2 v0, glm::vec2 v1)
    {
        return glm::vec2(glm::min(v0.x, v1.x), glm::max(v0.y, v1.y));
    }

    static glm::vec2 op_union(glm::vec2 v0, glm::vec2 v1)
    {
        return glm::vec2(glm::min(v0.x, v1.x), glm::min(v0.y, v1.y));
    }

    static glm::vec2 op_intersection(glm::vec2 v0, glm::vec2 v1)
    {
        return glm::vec2(glm::max(v0.x, v1.x), glm::max(v0.y, v1.y));
    }

    static glm::vec2 op_subtraction(glm::vec2 v0, glm::vec2 v1)
    {
        glm::vec2 neg_v1 = glm::vec2(-v1.y, -v1.x);
        return glm::vec2(glm::max(v0.x, neg_v1.x), glm::max(v0.y, neg_v1.y));
    }

    inline static float terrain_scale = 0.075f;
    inline static float terrain_amp = 2.f;
    inline static float terrain_detail_scale = 2.5f;
    inline static float terrain_detail_amp = 0.025f;
    inline static float flat_terrain_factor = 2.3f;

    static glm::vec2 minmax_map(glm::vec3 p, glm::vec3 half_size)
    {
        glm::vec2 minmax;
        minmax = minmax_noise(p, half_size, terrain_scale, terrain_amp);
        minmax = op_add(minmax, minmax_gradient_y(p - glm::vec3(0.f, 16.f, 0.f), half_size,
                                                  flat_terrain_factor));

        minmax = op_add(minmax,
                        minmax_noise(p, half_size, terrain_detail_scale, terrain_detail_amp));

        return minmax;
    }

    static float map(glm::vec3 p)
    {
        float d;
        d = sd_noise_conservative(p, terrain_scale, terrain_amp);
        d += sd_gradient_y(p - glm::vec3(0.f, 16.f, 0.f), flat_terrain_factor);
        d += sd_noise_conservative(p, terrain_detail_scale, terrain_detail_amp);
        return d;
    }

    static glm::vec3 map_normal(glm::vec3 wp)
    {
        glm::vec2 e = glm::vec2(0.01f, 0.f);

        float map_wp = map(wp);
        float dx = map(wp + glm::xyy(e)) - map_wp;
        float dy = map(wp + glm::yxy(e)) - map_wp;
        float dz = map(wp + glm::yyx(e)) - map_wp;

        return glm::normalize(glm::vec3(dx, dy, dz));
    }

    static uint32_t pack_normal(const glm::vec3& normal)
    {
        static uint32_t rand_hash = 0;
        uint32_t packed = 0;
        uint32_t quantized_bits = 4;
        uint32_t max_val = (1 << quantized_bits) - 1;

        glm::vec3 normal01 = (normal * 0.5f + 0.5f);
        glm::vec3 scaled_normal = float(max_val) * normal01;

        float random = glm::fract(float(rand_hash) / 1024.f);
        rand_hash = 551 * rand_hash + 6251;
        random = 0;
        glm::uvec3 quantized = glm::uvec3(
            glm::min(glm::floor(scaled_normal + random), float(max_val)));

        return quantized.x | (quantized.y << quantized_bits) |
               (quantized.z << (2 * quantized_bits));
    }

    static glm::vec3 unpack_normal(uint32_t packed)
    {
        uint32_t quantized_bits = 4;
        uint32_t max_val = (1 << quantized_bits) - 1;

        glm::uvec3 quantized;
        quantized.x = packed & max_val;
        quantized.y = (packed >> quantized_bits) & max_val;
        quantized.z = (packed >> (2 * quantized_bits)) & max_val;

        glm::vec3 normal = glm::vec3(quantized) / float(max_val);
        normal = normal * 2.0f - 1.0f;

        return glm::normalize(normal);
    }

    static bool is_uniform(glm::vec2 minmax) { return glm::sign(minmax.x) == glm::sign(minmax.y); }

    static float chunk_size_at_lod(float max_size, int lod) { return max_size / pow(2.f, lod); }

    static glm::vec3 chunk_center_at_lod(float max_size, glm::vec3 wp, int lod)
    {
        float chunk_size = chunk_size_at_lod(max_size, lod);
        glm::vec3 chunk_origin = floor(wp / chunk_size) * chunk_size;
        return chunk_origin + chunk_size * 0.5f;
    }

}; // namespace terrain_gen