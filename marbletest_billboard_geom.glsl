#version 330 core

layout (points) in;
layout (triangle_strip, max_vertices=4) out;

out vec2 uv;

void quad(vec4 pos, vec4 right, vec4 up) {
  gl_Position = pos - right - up;;
  uv = vec2(1,0);
  EmitVertex();

  gl_Position = pos + right -up;
  uv = vec2(0,0);
  EmitVertex();

  gl_Position = pos - right + up;
  uv = vec2(1,1);
  EmitVertex();

  gl_Position = pos + right + up;
  uv = vec2(0,1);
  EmitVertex();

  EndPrimitive();
}

void main() {
  quad(gl_in[0].gl_Position, vec4(0.5,0,0,0), vec4(0,0.5,0,0));
}
