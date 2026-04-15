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

vec4 samplePixel(vec2 pix) {
    vec2 uv = (pix + vec2(0.5, 0.5)) / vec2(w, h);
    return texture2D(tex, uv);
}

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

    vec2 p0 = floor(uv_pix);
    vec2 f = uv_pix - p0;

    vec2 p00 = p0;
    vec2 p10 = p0 + vec2(1.0, 0.0);
    vec2 p01 = p0 + vec2(0.0, 1.0);
    vec2 p11 = p0 + vec2(1.0, 1.0);

    // if any bilinear neighbor would be out of bounds, output black
    if (p00.x < 0.0 || p00.y < 0.0 || p11.x > (w - 1.0) || p11.y > (h - 1.0)) {
        gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }

    vec4 c00 = samplePixel(p00);
    vec4 c10 = samplePixel(p10);
    vec4 c01 = samplePixel(p01);
    vec4 c11 = samplePixel(p11);

    vec4 c0 = mix(c00, c10, f.x);
    vec4 c1 = mix(c01, c11, f.x);
    gl_FragColor = mix(c0, c1, f.y);
}