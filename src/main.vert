#version 460

#include <Canta/canta.glsl>

layout (location = 0) out VsOut {
    vec3 fragPos;
    vec3 normal;
} vsOut;

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
    LightBuffer light;
};

void main() {
    Vertex vertex = vertexBuffer.vertices[gl_VertexIndex];
    GPUCamera camera = camera.camera;

    vec4 fragPos = transform.transform * vec4(vertex.position, 1.0);
    vsOut.fragPos = fragPos.xyz;
    vsOut.normal = vertex.normal;

    gl_Position = camera.projection * camera.view * fragPos;
}