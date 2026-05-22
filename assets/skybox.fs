#version 330
in vec3 fragPosition;
uniform sampler2D texture0;
out vec4 finalColor;
void main() {
    vec3 dir = normalize(fragPosition);
    float phi = atan(dir.z, dir.x);
    float theta = acos(dir.y);
    vec2 uv = vec2((phi + 3.14159265) / (2.0 * 3.14159265), theta / 3.14159265);
    finalColor = texture(texture0, uv);
}