#version 330
in vec2 fragTexCoord;
uniform sampler2D texture0;
uniform vec2 texelSize;
uniform int  horizontal;
out vec4 finalColor;

void main() {
    // 9-tap Gaussian weights (sigma ≈ 1.4)
    float w[5] = float[](0.227027, 0.194595, 0.121622, 0.054054, 0.016216);
    vec2 dir = (horizontal == 1) ? vec2(1.0, 0.0) : vec2(0.0, 1.0);

    vec4 result = texture(texture0, fragTexCoord) * w[0];
    for (int i = 1; i < 5; i++) {
        vec2 off = dir * texelSize * float(i);
        result += texture(texture0, fragTexCoord + off) * w[i];
        result += texture(texture0, fragTexCoord - off) * w[i];
    }
    finalColor = result;
}
