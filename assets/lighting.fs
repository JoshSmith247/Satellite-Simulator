#version 330
in vec2 fragTexCoord;
in vec3 fragNormal;
in vec3 fragPosition;

uniform sampler2D texture0;
uniform vec3 lightPos;
uniform vec3 ambient;

out vec4 finalColor;

void main() {
    vec4 texelColor = texture(texture0, fragTexCoord);
    // vec3 lightDir = normalize(lightPos - fragPosition);
    vec3 lightDir = normalize(lightPos); // fragPosition is negligible compared to 150 million km
    
    // Calculate diffuse light (Lambertian)
    float diff = max(dot(fragNormal, lightDir), 0.0);
    vec3 diffuse = diff * vec3(1.0, 1.0, 1.0); // White light
    
    finalColor = vec4(ambient + diffuse, 1.0) * texelColor;
}