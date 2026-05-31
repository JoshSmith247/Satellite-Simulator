#version 330
in vec2 fragTexCoord;
in vec3 fragNormal;
in vec3 fragPosition;

uniform sampler2D texture0;
uniform vec3 lightPos;
uniform vec3 ambient;
uniform vec3 viewPos;

out vec4 finalColor;

void main() {
    vec4 texelColor = texture(texture0, fragTexCoord);
    vec3 normal     = normalize(fragNormal);
    vec3 lightDir   = normalize(lightPos);
    vec3 viewDir    = normalize(viewPos - fragPosition);

    float diff    = max(dot(normal, lightDir), 0.0);
    vec3  diffuse = diff * vec3(1.0, 0.97, 0.90);

    // Metallic specular
    vec3  halfDir = normalize(lightDir + viewDir);
    float spec    = pow(max(dot(normal, halfDir), 0.0), 48.0) * diff;
    vec3  specular = spec * 0.5 * vec3(1.0, 1.0, 1.0);

    vec3 color = (ambient + diffuse) * texelColor.rgb + specular;
    finalColor = vec4(color, 1.0);
}
