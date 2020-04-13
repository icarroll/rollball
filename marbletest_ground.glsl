#version 330 core
out vec4 FragColor;
in vec3 FragPos;
in vec3 Normal;
uniform vec3 cameraPos;
uniform vec3 lightPos;
uniform vec3 lightColor;
uniform vec3 objectColor;
uniform vec3 sphere_pos;
uniform float sphere_r;
uniform vec3 sphere2_pos;
uniform float sphere2_r;
void main() {
  float ambientStrength = 0.1;
  vec3 ambient = ambientStrength * lightColor;
  vec3 norm = normalize(Normal);
  vec3 lightDir = normalize(lightPos - FragPos);
  //vec3 lightDir = normalize(lightPos - vec3(0,0,0));
  float diff = max(dot(norm, lightDir), 0.0);
  vec3 diffuse = diff * lightColor;
  float specularStrength = 0.25;
  //vec3 viewPos = vec3(0.0, 0.0, 10.0);
  vec3 viewPos = cameraPos;
  vec3 viewDir = normalize(viewPos - FragPos);
  //vec3 viewDir = normalize(viewPos - vec3(0,0,0));
  vec3 reflectDir = reflect(lightDir, norm);
  float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
  vec3 specular = specularStrength * spec * lightColor;

  vec2 spos = sphere_pos.xz;
  vec2 gpos = FragPos.xz;
  vec2 s2pos = sphere2_pos.xz;
  float in_circle = max(0, (sphere_r - length(gpos - spos)) / sphere_r);
  float in_circle2 = max(0, (sphere2_r - length(gpos - s2pos)) / sphere2_r);
  
  //vec3 result = (ambient + diffuse + specular) * objectColor;
  //vec3 result = (ambient + (diffuse + specular)*(1-in_circle)) * objectColor;
  vec3 result;
  if (in_circle > 0.0 || in_circle2 > 0.0) result = (ambient + diffuse/4) * objectColor;
  else result = (ambient + diffuse + specular) * objectColor;

  FragColor = vec4(result, 1.0);
}
