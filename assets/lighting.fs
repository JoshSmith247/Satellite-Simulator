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

    // Lambertian diffuse — slightly warm sunlight
    float NdotL  = dot(normal, lightDir);
    float diff   = max(NdotL, 0.0);
    vec3  diffuse = diff * vec3(1.0, 0.98, 0.92);

    // Blinn-Phong specular — ocean glints, land is matte
    vec3  halfDir  = normalize(lightDir + viewDir);
    float NdotH    = max(dot(normal, halfDir), 0.0);
    float specStr  = pow(NdotH, 96.0) * diff;           // only on lit side
    float oceanMask = clamp((texelColor.b - texelColor.r) * 2.5, 0.0, 1.0);
    vec3  specular  = specStr * oceanMask * 0.7 * vec3(0.75, 0.88, 1.0);

    // Night-side city-light glow (fake warm scatter based on texture luma)
    float nightFactor = clamp(-NdotL * 5.0, 0.0, 1.0);
    float luma        = dot(texelColor.rgb, vec3(0.299, 0.587, 0.114));
    vec3  nightGlow   = nightFactor * luma * 0.4 * vec3(1.0, 0.72, 0.25);

    // Atmospheric Fresnel rim — blue halo on Earth limb
    float NdotV  = max(dot(normal, viewDir), 0.0);
    float rim    = pow(1.0 - NdotV, 4.5);
    float rimDay = clamp(NdotL + 0.35, 0.0, 1.0);
    vec3  rimColor = rim * rimDay * 0.65 * vec3(0.18, 0.50, 1.0);

    vec3 color = (ambient + diffuse) * texelColor.rgb + specular + nightGlow + rimColor;
    finalColor = vec4(color, 1.0);
}
