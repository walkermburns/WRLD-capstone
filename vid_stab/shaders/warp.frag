#version 330

in vec2 v_texcoord;
out vec4 fragColor;
uniform sampler2D tex;

uniform float w;
uniform float h;

uniform float h00; uniform float h01; uniform float h02;
uniform float h10; uniform float h11; uniform float h12;
uniform float h20; uniform float h21; uniform float h22;

void main() {
    vec2 pxy = v_texcoord * vec2(w, h);
    vec3 p = vec3(pxy, 1.0);

    vec3 q;
    q.x = h00*p.x + h01*p.y + h02;
    q.y = h10*p.x + h11*p.y + h12;
    q.z = h20*p.x + h21*p.y + h22;

    // Prevent divide-by-zero / invalid projection
    if (abs(q.z) < 1e-6) {
        fragColor = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }

    vec2 uv_pix = q.xy / q.z;

    // If projected pixel is outside image bounds, output black
    if (uv_pix.x < 0.0 || uv_pix.x >= w ||
        uv_pix.y < 0.0 || uv_pix.y >= h) {
        fragColor = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }

    vec2 uv = uv_pix / vec2(w, h);
    fragColor = texture(tex, uv);
}