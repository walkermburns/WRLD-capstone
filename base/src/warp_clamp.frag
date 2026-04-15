#version 100
#ifdef GL_ES
precision mediump float;
#endif

varying vec2 v_texcoord;
uniform sampler2D tex;

uniform float w;
uniform float h;

// inverse homography (dest->src) in pixel coordinates
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

    if (abs(q.z) < 1e-6) {
        gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }

    vec2 uv_pix = q.xy / q.z;

    // clamp to nearest valid source pixel
    uv_pix = clamp(uv_pix, vec2(0.0), vec2(w - 1.0, h - 1.0));

    vec2 uv = uv_pix / vec2(w, h);
    gl_FragColor = texture2D(tex, uv);
}