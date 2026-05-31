#version 330
in vec2 fragTexCoord;
uniform sampler2D texture0;
uniform float threshold;
out vec4 finalColor;

void main() {
    vec4  color      = texture(texture0, fragTexCoord);
    float brightness = dot(color.rgb, vec3(0.2126, 0.7152, 0.0722));
    float excess     = max(brightness - threshold, 0.0);
    // Scale so pixels just above threshold start dim, very bright pixels stay bright
    finalColor = vec4(color.rgb * (excess / max(brightness, 0.001)) * 2.5, 1.0);
}
