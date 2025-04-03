#include <Canta/Device.h>
#include <Canta/SDLWindow.h>
#include <Canta/ImGuiContext.h>

#include <Canta/RenderGraph.h>

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

    auto renderGraph = canta::RenderGraph::create({
        .device = device.get(),
        .multiQueue = false,
        .name = "primary_rendergraph",
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
        device->beginFrame();
        device->gc();
        imguiContext.beginFrame();
        ImGui::ShowDemoWindow();

        canta::drawRenderGraph(renderGraph);
        canta::renderGraphDebugUi(renderGraph);

        ImGui::Render();

        renderGraph.reset();
        auto swapImage = swapchain->acquire().value();

        auto swapchainIndex = renderGraph.addImage({
            .handle = swapImage,
            .name = "swapchain_image"
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
        dt = device->endFrame();
    }

    device->waitIdle();
    return 0;
}
