#include <Canta/Device.h>
#include <Canta/SDLWindow.h>
#include <Canta/ImGuiContext.h>
#include <Canta/Camera.h>
#include <Canta/RenderGraph.h>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include <filesystem>

#include <Canta/PipelineManager.h>
#include "embeded_shaders_graphics_project.h"

struct Vertex {
    ende::math::Vec3f position;
    ende::math::Vec3f normal;
    ende::math::Vec<2, f32> texCoord;
};

std::pair<canta::BufferHandle, i32> loadObj(canta::Device& device, const std::filesystem::path& path) {
    tinyobj::ObjReaderConfig reader_config;
    reader_config.mtl_search_path = "./"; // Path to material files

    tinyobj::ObjReader reader;

    if (!reader.ParseFromFile(path, reader_config)) {
        if (!reader.Error().empty()) {
            device.logger().error("TinyObjReader: {}", reader.Error());
        }
        exit(1);
    }

    if (!reader.Warning().empty()) {
        device.logger().warn("TinyObjReader: {}", reader.Warning());
    }

    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
    auto& materials = reader.GetMaterials();

    std::vector<Vertex> vertices;

    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);

            // Loop over vertices in the face.
            for (size_t v = 0; v < fv; v++) {

                Vertex vertex = {};

                // access to vertex
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                tinyobj::real_t vx = attrib.vertices[3*size_t(idx.vertex_index)+0];
                tinyobj::real_t vy = attrib.vertices[3*size_t(idx.vertex_index)+1];
                tinyobj::real_t vz = attrib.vertices[3*size_t(idx.vertex_index)+2];

                vertex.position = { vx, vy, vz };

                // Check if `normal_index` is zero or positive. negative = no normal data
                if (idx.normal_index >= 0) {
                    tinyobj::real_t nx = attrib.normals[3*size_t(idx.normal_index)+0];
                    tinyobj::real_t ny = attrib.normals[3*size_t(idx.normal_index)+1];
                    tinyobj::real_t nz = attrib.normals[3*size_t(idx.normal_index)+2];

                    vertex.normal = { nx, ny, nz };
                }

                // Check if `texcoord_index` is zero or positive. negative = no texcoord data
                if (idx.texcoord_index >= 0) {
                    tinyobj::real_t tx = attrib.texcoords[2*size_t(idx.texcoord_index)+0];
                    tinyobj::real_t ty = attrib.texcoords[2*size_t(idx.texcoord_index)+1];
                    vertex.texCoord = { tx, ty };
                }

                vertices.push_back(vertex);
            }
            index_offset += fv;

            // per-face material
            shapes[s].mesh.material_ids[f];
        }
    }

    auto vertexBuffer = device.createBuffer({
        .size = static_cast<u32>(vertices.size() * sizeof(Vertex)),
        .usage = canta::BufferUsage::VERTEX,
        .type = canta::MemoryType::STAGING,
        .persistentlyMapped = true,
    });

    vertexBuffer->data(vertices);

    return { vertexBuffer, vertices.size() };
}

class Transform {
public:

    Transform(const ende::math::Vec3f& pos = { 0, 0, 0 }, const ende::math::Quaternion& rot = { 0, 0, 0, 1 }, const ende::math::Vec3f& scale = { 1, 1, 1 })
        : _position(pos), _rotation(rot), _scale(scale)
    {}

    auto toMatrix() const -> ende::math::Mat4f {
        auto translation = ende::math::translation<4, f32>(_position);
        auto rotation = _rotation.toMat();
        auto scale = ende::math::scale<4, f32>(_scale);

        return translation * rotation * scale;
    }

    auto pos() const -> ende::math::Vec3f { return _position; }
    auto rot() const -> ende::math::Quaternion { return _rotation; }
    auto scale() const -> ende::math::Vec3f { return _scale; }

    void setPos(const ende::math::Vec3f& pos) { _position = pos; }
    void setRot(const ende::math::Quaternion& rot) { _rotation = rot; }
    void setScale(const ende::math::Vec3f& scale) { _scale = scale; }

private:

    ende::math::Vec3f _position = {};
    ende::math::Quaternion _rotation = {};
    ende::math::Vec3f _scale = {};

};

int main() {

    auto window = canta::SDLWindow("graphics_project", 960, 540);

    auto device = canta::Device::create({
        .applicationName = "graphics_project",
        .enableMeshShading = false,
        .instanceExtensions = window.requiredExtensions(),
    }).value();

    auto swapchain = device->createSwapchain({
        .window = &window,
    });

    auto imguiContext = canta::ImGuiContext::create({
        .device = device.get(),
        .window = &window
    });

    auto pipelineManager = canta::PipelineManager::create({
        .device = device.get(),
        .rootPath = "./"
    });
    registerEmbededShadersgraphics_project(pipelineManager);

    auto pipeline = pipelineManager.getPipeline(canta::Pipeline::CreateInfo{
        .vertex = {
            .module = pipelineManager.getShader({
                .path = "/home/olorin99/Documents/UQ/COSC3000/graphics_project/src/main.vert",
                .stage = canta::ShaderStage::VERTEX,
            }).value()
        },
        .fragment = {
            .module = pipelineManager.getShader({
                .path = "main.frag",
                .stage = canta::ShaderStage::FRAGMENT,
            }).value()
        },
        .rasterState = {
            .cullMode = canta::CullMode::NONE
        },
        .depthState = {
            .test = false,
            .write = false,
            .compareOp = canta::CompareOp::GEQUAL
        },
        .colourFormats = { swapchain->format() }
    }).value();

    auto renderGraph = canta::RenderGraph::create({
        .device = device.get(),
        .multiQueue = false,
        .name = "primary_rendergraph",
    });
    renderGraph.setIndividualPipelineStatistics(true);

    auto vertexBuffer = loadObj(*device, "/home/olorin99/Documents/UQ/COSC3000/Graphics Project/models/10299_Monkey-Wrench_v1_L3.obj");

    auto camera = canta::Camera::create({
        .position = { 0, 0, 2 },
        .rotation = ende::math::Quaternion({ 0, 0, 1 }, ende::math::rad(180)),
        .width = static_cast<f32>(window.extent().x()),
        .height = static_cast<f32>(window.extent().y())
    });

    auto cameraBuffers = std::to_array({
        device->createBuffer({
            .size = sizeof(canta::GPUCamera),
            .usage = canta::BufferUsage::STORAGE,
            .type = canta::MemoryType::STAGING,
            .name = "camera_buffer_0"
        }),
        device->createBuffer({
            .size = sizeof(canta::GPUCamera),
            .usage = canta::BufferUsage::STORAGE,
            .type = canta::MemoryType::STAGING,
            .name = "camera_buffer_1"
        })
    });

    auto objectTransform = Transform();
    auto transformBuffers = std::to_array({
        device->createBuffer({
            .size = sizeof(ende::math::Mat4f),
            .usage = canta::BufferUsage::STORAGE,
            .type = canta::MemoryType::STAGING,
            .name = "transform_buffer_0"
        }),
        device->createBuffer({
            .size = sizeof(ende::math::Mat4f),
            .usage = canta::BufferUsage::STORAGE,
            .type = canta::MemoryType::STAGING,
            .name = "transform_buffer_1"
        })
    });

    bool running = true;
    SDL_Event event;
    f32 dt = 0.0f;
    while (running) {
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
            case SDL_QUIT:
                running = false;
                break;
            }
            imguiContext.processEvent(&event);
        }
        {
            auto cameraPosition = camera.position();
            auto cameraRotation = camera.rotation().unit();
            if (SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_W])
                camera.setPosition(cameraPosition + cameraRotation.front() * dt * 10);
            if (SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_S])
                camera.setPosition(cameraPosition + cameraRotation.back() * dt * 10);
            if (SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_A])
                camera.setPosition(cameraPosition + cameraRotation.left() * dt * 10);
            if (SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_D])
                camera.setPosition(cameraPosition + cameraRotation.right() * dt * 10);
            if (SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_LSHIFT])
                camera.setPosition(cameraPosition + cameraRotation.down() * dt * 10);
            if (SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_SPACE])
                camera.setPosition(cameraPosition + cameraRotation.up() * dt * 10);
            if (SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_LEFT])
                camera.setRotation(ende::math::Quaternion({ 0, 1, 0 }, ende::math::rad(90) * dt) * cameraRotation);
            if (SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_RIGHT])
                camera.setRotation(ende::math::Quaternion({ 0, 1, 0 }, ende::math::rad(-90) * dt) * cameraRotation);
            if (SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_UP])
                camera.setRotation(ende::math::Quaternion(cameraRotation.right(), ende::math::rad(-45) * dt) * cameraRotation);
            if (SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_DOWN])
                camera.setRotation(ende::math::Quaternion(cameraRotation.right(), ende::math::rad(45) * dt) * cameraRotation);
        }
        device->beginFrame();
        device->gc();
        imguiContext.beginFrame();
        // ImGui::ShowDemoWindow();

        // canta::drawRenderGraph(renderGraph);
        // canta::renderGraphDebugUi(renderGraph);

        if (ImGui::Begin("Settings")) {
            ImGui::Text("Position: { %f, %f, %f }", camera.position().x(), camera.position().y(), camera.position().z());

            auto pos = objectTransform.pos();
            if (ImGui::DragFloat3("Position", &pos[0], 0.1))
                objectTransform.setPos(pos);

            auto eulerAnglesRad = objectTransform.rot().toEuler();
            auto eulerAnglesDeg = ende::math::Vec3f{
                static_cast<f32>(ende::math::deg(eulerAnglesRad.x())),
                static_cast<f32>(ende::math::deg(eulerAnglesRad.y())),
                static_cast<f32>(ende::math::deg(eulerAnglesRad.z()))
            };
            if (ImGui::DragFloat3("Rotation", &eulerAnglesDeg[0], 1, 0, 360))
                objectTransform.setRot(ende::math::Quaternion(ende::math::rad(eulerAnglesDeg.x()), ende::math::rad(eulerAnglesDeg.y()), ende::math::rad(eulerAnglesDeg.z())));

            auto scale = objectTransform.scale();
            if (ImGui::DragFloat3("Scale", &scale[0], 0.1))
                objectTransform.setScale(scale);

            auto pipelineStatistics = renderGraph.pipelineStatistics();
            for (auto& pipelineStats : pipelineStatistics) {
                if (ImGui::TreeNode(pipelineStats.first.c_str())) {
                    auto stats = pipelineStats.second.result().value();
                    ImGui::Text("Input Assembly Vertices: %d", stats.inputAssemblyVertices);
                    ImGui::Text("Input Assembly Primitives: %d", stats.inputAssemblyPrimitives);
                    ImGui::Text("Vertex Shader Invocations: %d", stats.vertexShaderInvocations);
                    ImGui::Text("Geometry Shader Invocations: %d", stats.geometryShaderInvocations);
                    ImGui::Text("Geometry Shader Primitives: %d", stats.geometryShaderPrimitives);
                    ImGui::Text("Clipping Invocations: %d", stats.clippingInvocations);
                    ImGui::Text("Clipping Primitives: %d", stats.clippingPrimitives);
                    ImGui::Text("Fragment Shader Invocations: %d", stats.fragmentShaderInvocations);
                    ImGui::Text("Tessellation Control Shader Patches: %d", stats.tessellationControlShaderPatches);
                    ImGui::Text("Tessellation Evaluation Shader Invocations: %d", stats.tessellationEvaluationShaderInvocations);
                    ImGui::Text("Compute Shader Invocations: %d", stats.computeShaderInvocations);
                    ImGui::TreePop();
                }
            }
        }
        ImGui::End();

        ImGui::Render();

        cameraBuffers[device->flyingIndex()]->data(camera.gpuCamera());
        transformBuffers[device->flyingIndex()]->data(objectTransform.toMatrix());

        renderGraph.reset();
        auto swapImage = swapchain->acquire().value();

        auto swapchainIndex = renderGraph.addImage({
            .handle = swapImage,
            .name = "swapchain_image"
        });

        auto vertexBufferIndex = renderGraph.addBuffer({
            .handle = vertexBuffer.first,
            .name = "vertex_buffer"
        });

        auto cameraBufferIndex = renderGraph.addBuffer({
            .handle = cameraBuffers[device->flyingIndex()],
            .name = "camera_buffer"
        });

        auto transformBufferIndex = renderGraph.addBuffer({
            .handle = transformBuffers[device->flyingIndex()],
            .name = "transform_buffer"
        });

        renderGraph.addPass({ .name = "main", .type = canta::PassType::GRAPHICS })
            .setPipeline(pipeline)
            .addStorageBufferRead(vertexBufferIndex, canta::PipelineStage::VERTEX_SHADER)
            .addStorageBufferRead(cameraBufferIndex, canta::PipelineStage::VERTEX_SHADER)
            .addStorageBufferRead(transformBufferIndex, canta::PipelineStage::VERTEX_SHADER)
            .addColourWrite(swapchainIndex)
            .setExecuteFunction([vertexBuffer, cameraBufferIndex, transformBufferIndex](auto& cmd, auto& graph) {
                struct Push {
                    u64 vertexBuffer;
                    u64 cameraBuffer;
                    u64 transformBuffer;
                };
                cmd.pushConstants(canta::ShaderStage::VERTEX, Push {
                    .vertexBuffer = vertexBuffer.first->address(),
                    .cameraBuffer = graph.getBuffer(cameraBufferIndex)->address(),
                    .transformBuffer = graph.getBuffer(transformBufferIndex)->address(),
                });
                cmd.draw(vertexBuffer.second);
            });


        renderGraph.addPass({.name = "ui", .type = canta::PassType::GRAPHICS})
            .setManualPipeline(true)
            .addColourWrite(swapchainIndex)
            .setExecuteFunction([&imguiContext, &swapchain](canta::CommandBuffer& cmd, canta::RenderGraph& graph) {
            imguiContext.render(ImGui::GetDrawData(), cmd, swapchain->format());
        });

        renderGraph.setBackbuffer(swapchainIndex);

        if (!renderGraph.compile())
            return -1;

        auto waits = std::to_array({
            canta::SemaphorePair{ device->frameSemaphore(), device->framePrevValue() },
            canta::SemaphorePair(swapchain->acquireSemaphore()),
        });
        auto signals = std::to_array({
            canta::SemaphorePair(device->frameSemaphore()),
            canta::SemaphorePair(swapchain->presentSemaphore())
        });
        if (!renderGraph.execute(waits, signals, {}, false))
            return -2;

        swapchain->present();
        dt = device->endFrame() / 1000.f;
    }

    device->waitIdle();
    return 0;
}
