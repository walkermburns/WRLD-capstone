#version 100
#ifdef GL_ES
precision mediump float;
#endif

varying vec2 v_texcoord;
uniform sampler2D tex;
uniform float k1;
uniform float zoom;

void main () {
    vec2 uv = (v_texcoord - 0.5) * 1.1;
    float r2 = dot(uv, uv);
    vec2 distorted_uv = uv * (1.0 + k1 * r2);
    gl_FragColor = texture2D(tex, distorted_uv + 0.5);
}
