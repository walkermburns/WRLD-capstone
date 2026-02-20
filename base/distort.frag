#version 100
#ifdef GL_ES
precision mediump float;
#endif

varying vec2 v_texcoord;
uniform sampler2D tex;

// These are provided by glshader but not strictly needed for this math
uniform float time;
uniform float width;
uniform float height;

void main () {
    // 1.0 = no zoom, 1.2 = slight zoom to crop distorted edges
    float zoom = 1.1;
    
    // Shift coordinates so (0,0) is the center of the image
    vec2 uv = (v_texcoord - 0.5) * zoom;
    
    // Calculate the distance from the center squared
    float r2 = dot(uv, uv);
    
    // Apply Barrel Distortion
    // Increase 0.3 to make the bulge stronger
    // Increase 0.1 to make the corners pull more
    vec2 distorted_uv = uv * (1.0 + 0.3 * r2 + 0.1 * r2 * r2);
    
    // Shift coordinates back to 0.0 - 1.0 range for texture sampling
    gl_FragColor = texture2D(tex, distorted_uv + 0.5);
}