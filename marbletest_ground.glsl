#version 330 core
out vec4 FragColor;
in vec3 FragPos;
in vec3 Normal;
uniform vec3 lightPos;
uniform vec3 lightColor;
uniform vec3 objectColor;
uniform vec3 sphere_pos;
uniform float sphere_r;
void main() {
  float ambientStrength = 0.1;
  vec3 ambient = ambientStrength * lightColor;
  vec3 norm = -normalize(Normal);
  //vec3 lightDir = normalize(lightPos - FragPos);
  vec3 lightDir = normalize(lightPos - vec3(0,0,0));
  float diff = max(dot(norm, lightDir), 0.0);
  vec3 diffuse = diff * lightColor;
  float specularStrength = 0.25;
  vec3 viewPos = vec3(0.0, 0.0, 10.0);
  //vec3 viewDir = normalize(viewPos - FragPos);
  vec3 viewDir = normalize(viewPos - vec3(0,0,0));
  vec3 reflectDir = reflect(-lightDir, norm);
  float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
  vec3 specular = specularStrength * spec * lightColor;

  vec2 spos = sphere_pos.xz;
  vec2 gpos = vec2(-FragPos.x, FragPos.y);
  bool in_circle = length(gpos - spos) < sphere_r;
  //bool in_circle = false;
  
  //vec3 result = (ambient + diffuse + specular) * objectColor;
  vec3 result;
  if (in_circle) result = (ambient) * objectColor;
  else result = (ambient + diffuse + specular) * objectColor;

  FragColor = vec4(result, 1.0);
}
