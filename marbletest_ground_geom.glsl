#version 330 core
layout (triangles) in;
layout (triangle_strip, max_vertices=3) out;
out vec3 FragPos;
out vec3 Normal;
vec3 GetNormal()
{
  vec3 a = vec3(gl_in[0].gl_Position) - vec3(gl_in[1].gl_Position);
  vec3 b = vec3(gl_in[2].gl_Position) - vec3(gl_in[1].gl_Position);
  return normalize(cross(a, b));
}
void main() {
  vec3 n = GetNormal();

  FragPos = vec3(gl_in[0].gl_Position);
  Normal = n;
  gl_Position = gl_in[0].gl_Position;
  EmitVertex();

  FragPos = vec3(gl_in[1].gl_Position);
  Normal = n;
  gl_Position = gl_in[1].gl_Position;
  EmitVertex();

  FragPos = vec3(gl_in[2].gl_Position);
  Normal = n;
  gl_Position = gl_in[2].gl_Position;
  EmitVertex();

  EndPrimitive();
}
