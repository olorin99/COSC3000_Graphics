#version 460

#include <Canta/canta.glsl>

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

layout (push_constant) uniform Push {
    VertexBuffer vertexBuffer;
    CameraBuffer camera;
};

void main() {
    Vertex vertex = vertexBuffer.vertices[gl_VertexIndex];
    GPUCamera camera = camera.camera;

    gl_Position = camera.projection * camera.view * vec4(vertex.position, 1.0);
}