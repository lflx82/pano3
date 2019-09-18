#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

uniform mat4 mview_matrix;
uniform mat4 mproj_matrix;

attribute vec3 a_position;
attribute vec2 a_texcoord;

varying vec2 v_texcoord;

//! [0]
void main()
{
    // Calculate vertex position in screen space
    //gl_Position = mproj_matrix*mview_matrix * a_position;

    gl_Position = mproj_matrix*mview_matrix * vec4(a_position, 1.0);

    // Pass texture coordinate to fragment shader
    // Value will be automatically interpolated to fragments inside polygon faces
    v_texcoord = a_texcoord;
}
//! [0]
