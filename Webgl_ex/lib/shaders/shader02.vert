// Vertex Shader

precision mediump int;
precision mediump float;
attribute vec3 a_Vertex;
attribute vec3 a_Color;

varying vec4 v_vertex_color;

void main() {
  // Transform the location of the vertex if needed
  gl_Position = vec4(a_Vertex, 1.0);
  
  v_vertex_color = vec4(a_Color, 1.0);
}

