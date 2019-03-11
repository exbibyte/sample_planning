#version 430

in vec3 LightIntensity;

layout( location = 0 ) out vec4 FragColor;

in vec2 OutVertexTexCoord;

uniform sampler2D tex;
    
void main() {
    // FragColor = vec4(LightIntensity, 1.0);
    FragColor = texture2D( tex, OutVertexTexCoord );
    
}
