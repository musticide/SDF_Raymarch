#version 330

uniform mat4 matVP;
uniform mat4 matGeo;

layout (location = 0) in vec3 pos;
layout (location = 1) in vec3 normal;

out vec3 normalPS;
out vec3 vertexPosWS;
out vec3 vertexPosOS;

void main() {
   normalPS = vec3(abs(normal));
   vertexPosWS= vec3(matGeo * vec4(pos,1));
   vertexPosOS= pos;
   gl_Position = matVP * matGeo * vec4(pos, 1);
}
