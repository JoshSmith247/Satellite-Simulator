#version 330
in vec3 fragPosition;
uniform sampler2D texture0;
out vec4 finalColor;

void main() {
    vec3 dir   = normalize(fragPosition);
    float phi   = atan(dir.z, dir.x);
    float theta = acos(dir.y);
    vec2  uv    = vec2((phi + 3.14159265) / (2.0 * 3.14159265), theta / 3.14159265);

    vec3 stars = texture(texture0, uv).rgb;

    // Contrast S-curve: x^2 * 2 darkens background, boosts bright stars
    stars = stars * stars * 2.2;
    // Slight blue tint for deep space atmosphere
    stars *= vec3(0.90, 0.95, 1.0);

    finalColor = vec4(clamp(stars, 0.0, 1.0), 1.0);
}
