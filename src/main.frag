#version 460

#include <Canta/canta.glsl>

layout (location = 0) in VsOut {
    vec3 fragPos;
    vec3 normal;
} fsIn;

layout (location = 0) out vec4 FragOut;

struct Vertex {
    vec3 position;
    vec3 normal;
    vec2 uv;
};
declareBufferReference(VertexBuffer,
    Vertex vertices[];
);

struct Frustum {
    vec4 planes[6];
    vec4 corners[8];
};
struct GPUCamera {
    mat4 projection;
    mat4 view;
    vec3 position;
    float near;
    float far;
    Frustum frustum;
};
declareBufferReference(CameraBuffer,
    GPUCamera camera;
);

declareBufferReference(TransformBuffer,
    mat4 transform;
);

struct Light {
    vec3 position;
    vec3 colour;
    float intensity;
    float radius;
};

declareBufferReference(LightBuffer,
    Light light;
);

layout (push_constant) uniform Push {
    VertexBuffer vertexBuffer;
    CameraBuffer camera;
    TransformBuffer transform;
    LightBuffer lightBuffer;
};

void main() {
    Light light = lightBuffer.light;

    vec3 ambient = light.colour * 0.01;

    vec3 normal = normalize(fsIn.normal);
    vec3 lightDir = normalize(light.position - fsIn.fragPos);
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = light.colour * diff;

    vec3 viewDir = normalize(camera.camera.position - fsIn.fragPos);
    vec3 reflectDir = reflect(-lightDir, normal);
    vec3 halfwayDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(normal, halfwayDir), 0.0), 32.0);
    vec3 specular = light.colour * spec;

    float distance = length(light.position - fsIn.fragPos);
    float distanceSqr = distance * distance;
    float rangeSqr = light.radius * light.radius;
    float dpr = distanceSqr / max(0.0001, rangeSqr);
    dpr *= dpr;
    float attenuation = clamp(1 - dpr, 0, 1.0) / max(0.0001, distanceSqr);

    vec3 result = (ambient + diffuse + specular) * light.intensity * attenuation;

    FragOut = vec4(result, 1.0);
}