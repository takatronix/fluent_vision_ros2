// Headless retained Vulkan backend for the Fluent Scene stage-2 slice.
//
// Structural work (pipeline compilation, resource allocation) happens once in
// loadScene; renderFrame only uploads changed data and replays the retained
// plan (spec section 8.2). No window system integration: the composite is
// rendered offscreen and read back on demand.

#include <vulkan/vulkan.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "fluent_scene/render/renderer.hpp"
#include "render/box_smoother.hpp"
#include "render/scene_model.hpp"
#include "render/text_atlas.hpp"

namespace fluent_scene {
namespace {

// SPIR-V blobs generated at build time by glslc -mfmt=c.
static const uint32_t kImageVertSpv[] =
#include "image.vert.inc"
    ;
static const uint32_t kImageFragSpv[] =
#include "image.frag.inc"
    ;
static const uint32_t kBoxesVertSpv[] =
#include "boxes.vert.inc"
    ;
static const uint32_t kBoxesFragSpv[] =
#include "boxes.frag.inc"
    ;
static const uint32_t kTextVertSpv[] =
#include "text.vert.inc"
    ;
static const uint32_t kTextFragSpv[] =
#include "text.frag.inc"
    ;
static const uint32_t kCirclesVertSpv[] =
#include "circles.vert.inc"
    ;
static const uint32_t kCirclesFragSpv[] =
#include "circles.frag.inc"
    ;
static const uint32_t kPolylineVertSpv[] =
#include "polyline.vert.inc"
    ;
static const uint32_t kPolylineFragSpv[] =
#include "polyline.frag.inc"
    ;
static const uint32_t kFullscreenVertSpv[] =
#include "fullscreen.vert.inc"
    ;
static const uint32_t kBlurFragSpv[] =
#include "blur.frag.inc"
    ;
static const uint32_t kColorFragSpv[] =
#include "color.frag.inc"
    ;

struct PcImage {
    float dst_rect[4];
    float viewport[2];
    float pad[2];
};
struct PcBoxes {
    float color[4];
    float viewport[2];
    float thickness;
    float pad;
};
struct PcText {
    float color[4];
    float viewport[2];
    float offset[2];
};
struct PcCircles {
    float color[4];
    float viewport[2];
    float radius;
    float thickness;
};
struct PcPolyline {
    float color[4];
    float viewport[2];
    float thickness;
    float pad;
};
struct PcBlur {
    float texel[2];
    float radius;
    float pad;
};
struct PcColor {
    float brightness;
    float contrast;
    float saturation;
    float gamma;
};
static_assert(sizeof(PcImage) == 32 && sizeof(PcBoxes) == 32 && sizeof(PcText) == 32 &&
                  sizeof(PcCircles) == 32 && sizeof(PcPolyline) == 32,
              "push constant blocks must stay 32 bytes");

struct TextVertex {
    float pos[2];
    float uv[2];
};

constexpr float kBoxOutlineThickness = 3.0f;
constexpr float kShadowOffset = 1.5f;

class VulkanRenderer final : public Renderer {
public:
    explicit VulkanRenderer(const RendererOptions& options) : options_(options) {}

    ~VulkanRenderer() override { destroy(); }

    bool init(DiagnosticList& diagnostics) {
        return createInstance(diagnostics) && pickDevice(diagnostics) && createDevice(diagnostics);
    }

    bool loadScene(const ValidationResult& scene, const PlanResult& plan,
                   DiagnosticList& diagnostics) override {
        if (loaded_) {
            diag(diagnostics, "compile.invalid_input", "renderer already has a loaded scene");
            return false;
        }
        if (!render::buildSceneModel(scene, plan, model_, diagnostics)) {
            return false;
        }
        if (!createTargets(diagnostics) || !createPipelines(diagnostics) ||
            !createSceneResources(diagnostics)) {
            return false;
        }
        runtime_params_ = model_.mutable_params;
        loaded_ = true;
        return true;
    }

    bool renderFrame(const FrameInputs& inputs, DiagnosticList& diagnostics) override;

    bool readback(std::vector<uint8_t>& pixels, uint32_t& width, uint32_t& height) override;

    const RenderStats& stats() const override { return stats_; }
    const char* name() const override { return "vulkan"; }

    bool setParam(const std::string& name, float value) override {
        auto it = runtime_params_.find(name);
        if (it == runtime_params_.end()) {
            return false;
        }
        it->second = value;
        return true;
    }

private:
    float effectValue(float literal, const std::string& param_ref) const {
        if (param_ref.empty()) {
            return literal;
        }
        auto it = runtime_params_.find(param_ref);
        return it != runtime_params_.end() ? it->second : literal;
    }

    // ---- helpers -----------------------------------------------------------

    static void diag(DiagnosticList& diagnostics, const char* code, const std::string& message) {
        diagnostics.add(code, Severity::kError, Phase::kCompile, Span{}, message);
    }

    bool vkOk(VkResult result, DiagnosticList& diagnostics, const char* what) {
        if (result == VK_SUCCESS) {
            return true;
        }
        diag(diagnostics, "compile.gpu_error",
             std::string(what) + " failed with VkResult " + std::to_string(static_cast<int>(result)));
        return false;
    }

    uint32_t findMemoryType(uint32_t type_bits, VkMemoryPropertyFlags properties) const {
        VkPhysicalDeviceMemoryProperties memory{};
        vkGetPhysicalDeviceMemoryProperties(physical_, &memory);
        for (uint32_t i = 0; i < memory.memoryTypeCount; ++i) {
            if ((type_bits & (1u << i)) != 0 &&
                (memory.memoryTypes[i].propertyFlags & properties) == properties) {
                return i;
            }
        }
        return UINT32_MAX;
    }

    struct Buffer {
        VkBuffer buffer = VK_NULL_HANDLE;
        VkDeviceMemory memory = VK_NULL_HANDLE;
        void* mapped = nullptr;
        VkDeviceSize size = 0;
    };

    struct Image {
        VkImage image = VK_NULL_HANDLE;
        VkDeviceMemory memory = VK_NULL_HANDLE;
        VkImageView view = VK_NULL_HANDLE;
        uint32_t width = 0;
        uint32_t height = 0;
        VkImageLayout layout = VK_IMAGE_LAYOUT_UNDEFINED;
    };

    struct EffectState {
        const render::EffectOp* op = nullptr;
        Image target;
        Image tmp;  // blur ping buffer
        VkFramebuffer fb = VK_NULL_HANDLE;
        VkFramebuffer fb_tmp = VK_NULL_HANDLE;
        VkDescriptorSet source_set = VK_NULL_HANDLE;  // samples the effect source
        VkDescriptorSet tmp_set = VK_NULL_HANDLE;     // samples tmp (blur 2nd pass)
        VkDescriptorSet target_set = VK_NULL_HANDLE;  // sampled by downstream draws
        VkImageView last_source_view = VK_NULL_HANDLE;
        bool ready = false;
    };

    bool createBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties,
                      bool map, Buffer& out, DiagnosticList& diagnostics) {
        VkBufferCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        info.size = size;
        info.usage = usage;
        info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        if (!vkOk(vkCreateBuffer(device_, &info, nullptr, &out.buffer), diagnostics, "vkCreateBuffer")) {
            return false;
        }
        VkMemoryRequirements requirements{};
        vkGetBufferMemoryRequirements(device_, out.buffer, &requirements);
        VkMemoryAllocateInfo alloc{};
        alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        alloc.allocationSize = requirements.size;
        alloc.memoryTypeIndex = findMemoryType(requirements.memoryTypeBits, properties);
        if (alloc.memoryTypeIndex == UINT32_MAX ||
            !vkOk(vkAllocateMemory(device_, &alloc, nullptr, &out.memory), diagnostics,
                  "vkAllocateMemory")) {
            return false;
        }
        vkBindBufferMemory(device_, out.buffer, out.memory, 0);
        out.size = size;
        if (map &&
            !vkOk(vkMapMemory(device_, out.memory, 0, VK_WHOLE_SIZE, 0, &out.mapped), diagnostics,
                  "vkMapMemory")) {
            return false;
        }
        return true;
    }

    bool createImage(uint32_t width, uint32_t height, VkFormat format, VkImageUsageFlags usage,
                     Image& out, DiagnosticList& diagnostics) {
        VkImageCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        info.imageType = VK_IMAGE_TYPE_2D;
        info.format = format;
        info.extent = {width, height, 1};
        info.mipLevels = 1;
        info.arrayLayers = 1;
        info.samples = VK_SAMPLE_COUNT_1_BIT;
        info.tiling = VK_IMAGE_TILING_OPTIMAL;
        info.usage = usage;
        info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        if (!vkOk(vkCreateImage(device_, &info, nullptr, &out.image), diagnostics, "vkCreateImage")) {
            return false;
        }
        VkMemoryRequirements requirements{};
        vkGetImageMemoryRequirements(device_, out.image, &requirements);
        VkMemoryAllocateInfo alloc{};
        alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        alloc.allocationSize = requirements.size;
        alloc.memoryTypeIndex =
            findMemoryType(requirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        if (alloc.memoryTypeIndex == UINT32_MAX ||
            !vkOk(vkAllocateMemory(device_, &alloc, nullptr, &out.memory), diagnostics,
                  "vkAllocateMemory(image)")) {
            return false;
        }
        vkBindImageMemory(device_, out.image, out.memory, 0);
        VkImageViewCreateInfo view{};
        view.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        view.image = out.image;
        view.viewType = VK_IMAGE_VIEW_TYPE_2D;
        view.format = format;
        view.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        if (!vkOk(vkCreateImageView(device_, &view, nullptr, &out.view), diagnostics,
                  "vkCreateImageView")) {
            return false;
        }
        out.width = width;
        out.height = height;
        out.layout = VK_IMAGE_LAYOUT_UNDEFINED;
        return true;
    }

    void destroyImage(Image& image) {
        if (image.view != VK_NULL_HANDLE) {
            vkDestroyImageView(device_, image.view, nullptr);
        }
        if (image.image != VK_NULL_HANDLE) {
            vkDestroyImage(device_, image.image, nullptr);
        }
        if (image.memory != VK_NULL_HANDLE) {
            vkFreeMemory(device_, image.memory, nullptr);
        }
        image = Image{};
    }

    void destroyBuffer(Buffer& buffer) {
        if (buffer.mapped != nullptr) {
            vkUnmapMemory(device_, buffer.memory);
        }
        if (buffer.buffer != VK_NULL_HANDLE) {
            vkDestroyBuffer(device_, buffer.buffer, nullptr);
        }
        if (buffer.memory != VK_NULL_HANDLE) {
            vkFreeMemory(device_, buffer.memory, nullptr);
        }
        buffer = Buffer{};
    }

    static void recordImageBarrier(VkCommandBuffer cmd, Image& image, VkImageLayout new_layout,
                                   VkPipelineStageFlags src_stage, VkAccessFlags src_access,
                                   VkPipelineStageFlags dst_stage, VkAccessFlags dst_access) {
        VkImageMemoryBarrier barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        barrier.oldLayout = image.layout;
        barrier.newLayout = new_layout;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = image.image;
        barrier.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        barrier.srcAccessMask = src_access;
        barrier.dstAccessMask = dst_access;
        vkCmdPipelineBarrier(cmd, src_stage, dst_stage, 0, 0, nullptr, 0, nullptr, 1, &barrier);
        image.layout = new_layout;
    }

    // ---- setup -------------------------------------------------------------

    bool createInstance(DiagnosticList& diagnostics) {
        VkApplicationInfo app{};
        app.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
        app.pApplicationName = "fluent_scene";
        app.apiVersion = VK_API_VERSION_1_1;
        std::vector<const char*> layers;
        if (options_.enable_validation) {
            uint32_t count = 0;
            vkEnumerateInstanceLayerProperties(&count, nullptr);
            std::vector<VkLayerProperties> available(count);
            vkEnumerateInstanceLayerProperties(&count, available.data());
            for (const VkLayerProperties& layer : available) {
                if (std::strcmp(layer.layerName, "VK_LAYER_KHRONOS_validation") == 0) {
                    layers.push_back("VK_LAYER_KHRONOS_validation");
                    break;
                }
            }
        }
        VkInstanceCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
        info.pApplicationInfo = &app;
        info.enabledLayerCount = static_cast<uint32_t>(layers.size());
        info.ppEnabledLayerNames = layers.data();
        return vkOk(vkCreateInstance(&info, nullptr, &instance_), diagnostics, "vkCreateInstance");
    }

    bool pickDevice(DiagnosticList& diagnostics) {
        uint32_t count = 0;
        vkEnumeratePhysicalDevices(instance_, &count, nullptr);
        if (count == 0) {
            diag(diagnostics, "compile.gpu_error", "no Vulkan physical device is available");
            return false;
        }
        std::vector<VkPhysicalDevice> devices(count);
        vkEnumeratePhysicalDevices(instance_, &count, devices.data());
        int best_score = -1;
        for (VkPhysicalDevice candidate : devices) {
            uint32_t family_count = 0;
            vkGetPhysicalDeviceQueueFamilyProperties(candidate, &family_count, nullptr);
            std::vector<VkQueueFamilyProperties> families(family_count);
            vkGetPhysicalDeviceQueueFamilyProperties(candidate, &family_count, families.data());
            int graphics_family = -1;
            for (uint32_t i = 0; i < family_count; ++i) {
                if ((families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) != 0) {
                    graphics_family = static_cast<int>(i);
                    break;
                }
            }
            if (graphics_family < 0) {
                continue;
            }
            VkPhysicalDeviceProperties properties{};
            vkGetPhysicalDeviceProperties(candidate, &properties);
            // Prefer discrete GPUs (e.g. NVIDIA dGPU on an Intel+NVIDIA
            // machine), then integrated, then anything with graphics.
            int score = 1;
            if (properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
                score = 3;
            } else if (properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU) {
                score = 2;
            }
            if (score > best_score) {
                best_score = score;
                physical_ = candidate;
                graphics_family_ = static_cast<uint32_t>(graphics_family);
                device_name_ = properties.deviceName;
                timestamp_period_ns_ = properties.limits.timestampPeriod;
            }
        }
        if (physical_ == VK_NULL_HANDLE) {
            diag(diagnostics, "compile.gpu_error", "no Vulkan device exposes a graphics queue");
            return false;
        }
        return true;
    }

    bool createDevice(DiagnosticList& diagnostics) {
        const float priority = 1.0f;
        VkDeviceQueueCreateInfo queue{};
        queue.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        queue.queueFamilyIndex = graphics_family_;
        queue.queueCount = 1;
        queue.pQueuePriorities = &priority;
        VkDeviceCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
        info.queueCreateInfoCount = 1;
        info.pQueueCreateInfos = &queue;
        if (!vkOk(vkCreateDevice(physical_, &info, nullptr, &device_), diagnostics, "vkCreateDevice")) {
            return false;
        }
        vkGetDeviceQueue(device_, graphics_family_, 0, &queue_);

        VkCommandPoolCreateInfo pool{};
        pool.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        pool.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        pool.queueFamilyIndex = graphics_family_;
        if (!vkOk(vkCreateCommandPool(device_, &pool, nullptr, &command_pool_), diagnostics,
                  "vkCreateCommandPool")) {
            return false;
        }
        VkCommandBufferAllocateInfo alloc{};
        alloc.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        alloc.commandPool = command_pool_;
        alloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        alloc.commandBufferCount = 2;
        VkCommandBuffer buffers[2] = {};
        if (!vkOk(vkAllocateCommandBuffers(device_, &alloc, buffers), diagnostics,
                  "vkAllocateCommandBuffers")) {
            return false;
        }
        frame_cmd_ = buffers[0];
        aux_cmd_ = buffers[1];

        VkFenceCreateInfo fence{};
        fence.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
        if (!vkOk(vkCreateFence(device_, &fence, nullptr, &fence_), diagnostics, "vkCreateFence")) {
            return false;
        }
        VkQueryPoolCreateInfo query{};
        query.sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;
        query.queryType = VK_QUERY_TYPE_TIMESTAMP;
        query.queryCount = 2;
        return vkOk(vkCreateQueryPool(device_, &query, nullptr, &query_pool_), diagnostics,
                    "vkCreateQueryPool");
    }

    bool createTargets(DiagnosticList& diagnostics) {
        if (!createImage(model_.width, model_.height, VK_FORMAT_R8G8B8A8_UNORM,
                         VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT, target_,
                         diagnostics)) {
            return false;
        }
        VkAttachmentDescription color{};
        color.format = VK_FORMAT_R8G8B8A8_UNORM;
        color.samples = VK_SAMPLE_COUNT_1_BIT;
        color.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        color.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        color.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        color.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        color.finalLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        VkAttachmentReference color_ref{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1;
        subpass.pColorAttachments = &color_ref;
        VkSubpassDependency deps[2] = {};
        deps[0].srcSubpass = VK_SUBPASS_EXTERNAL;
        deps[0].dstSubpass = 0;
        deps[0].srcStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT;
        deps[0].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        deps[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        deps[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        deps[1].srcSubpass = 0;
        deps[1].dstSubpass = VK_SUBPASS_EXTERNAL;
        deps[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        deps[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        deps[1].dstStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT;
        deps[1].dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
        VkRenderPassCreateInfo pass{};
        pass.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        pass.attachmentCount = 1;
        pass.pAttachments = &color;
        pass.subpassCount = 1;
        pass.pSubpasses = &subpass;
        pass.dependencyCount = 2;
        pass.pDependencies = deps;
        if (!vkOk(vkCreateRenderPass(device_, &pass, nullptr, &render_pass_), diagnostics,
                  "vkCreateRenderPass")) {
            return false;
        }
        VkFramebufferCreateInfo framebuffer{};
        framebuffer.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        framebuffer.renderPass = render_pass_;
        framebuffer.attachmentCount = 1;
        framebuffer.pAttachments = &target_.view;
        framebuffer.width = model_.width;
        framebuffer.height = model_.height;
        framebuffer.layers = 1;
        return vkOk(vkCreateFramebuffer(device_, &framebuffer, nullptr, &framebuffer_), diagnostics,
                    "vkCreateFramebuffer");
    }

    VkShaderModule createShader(const uint32_t* code, size_t bytes, DiagnosticList& diagnostics) {
        VkShaderModuleCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        info.codeSize = bytes;
        info.pCode = code;
        VkShaderModule module = VK_NULL_HANDLE;
        if (!vkOk(vkCreateShaderModule(device_, &info, nullptr, &module), diagnostics,
                  "vkCreateShaderModule")) {
            return VK_NULL_HANDLE;
        }
        return module;
    }

    bool createPipeline(VkShaderModule vert, VkShaderModule frag, bool with_texture,
                        const VkVertexInputBindingDescription* binding,
                        const std::vector<VkVertexInputAttributeDescription>& attributes,
                        VkPipelineLayout layout, VkPipeline& out, DiagnosticList& diagnostics,
                        VkRenderPass target_pass = VK_NULL_HANDLE, bool enable_blend = true,
                        bool dynamic_viewport = false) {
        VkPipelineShaderStageCreateInfo stages[2] = {};
        stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
        stages[0].module = vert;
        stages[0].pName = "main";
        stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        stages[1].module = frag;
        stages[1].pName = "main";
        (void)with_texture;

        VkPipelineVertexInputStateCreateInfo vertex_input{};
        vertex_input.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
        if (binding != nullptr) {
            vertex_input.vertexBindingDescriptionCount = 1;
            vertex_input.pVertexBindingDescriptions = binding;
            vertex_input.vertexAttributeDescriptionCount = static_cast<uint32_t>(attributes.size());
            vertex_input.pVertexAttributeDescriptions = attributes.data();
        }
        VkPipelineInputAssemblyStateCreateInfo assembly{};
        assembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
        assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

        VkViewport viewport{0.0f, 0.0f, static_cast<float>(model_.width),
                            static_cast<float>(model_.height), 0.0f, 1.0f};
        VkRect2D scissor{{0, 0}, {model_.width, model_.height}};
        VkPipelineViewportStateCreateInfo viewport_state{};
        viewport_state.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
        viewport_state.viewportCount = 1;
        viewport_state.pViewports = &viewport;
        viewport_state.scissorCount = 1;
        viewport_state.pScissors = &scissor;

        VkPipelineRasterizationStateCreateInfo raster{};
        raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
        raster.polygonMode = VK_POLYGON_MODE_FILL;
        raster.cullMode = VK_CULL_MODE_NONE;
        raster.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        raster.lineWidth = 1.0f;

        VkPipelineMultisampleStateCreateInfo multisample{};
        multisample.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
        multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

        VkPipelineColorBlendAttachmentState blend_attachment{};
        blend_attachment.blendEnable = enable_blend ? VK_TRUE : VK_FALSE;
        blend_attachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
        blend_attachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        blend_attachment.colorBlendOp = VK_BLEND_OP_ADD;
        blend_attachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        blend_attachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        blend_attachment.alphaBlendOp = VK_BLEND_OP_ADD;
        blend_attachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                                          VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        VkPipelineColorBlendStateCreateInfo blend{};
        blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
        blend.attachmentCount = 1;
        blend.pAttachments = &blend_attachment;

        VkDynamicState dynamic_states[2] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
        VkPipelineDynamicStateCreateInfo dynamic{};
        dynamic.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
        dynamic.dynamicStateCount = 2;
        dynamic.pDynamicStates = dynamic_states;

        VkGraphicsPipelineCreateInfo pipeline{};
        pipeline.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
        pipeline.stageCount = 2;
        pipeline.pStages = stages;
        pipeline.pVertexInputState = &vertex_input;
        pipeline.pInputAssemblyState = &assembly;
        pipeline.pViewportState = &viewport_state;
        pipeline.pRasterizationState = &raster;
        pipeline.pMultisampleState = &multisample;
        pipeline.pColorBlendState = &blend;
        if (dynamic_viewport) {
            pipeline.pDynamicState = &dynamic;
        }
        pipeline.layout = layout;
        pipeline.renderPass = target_pass != VK_NULL_HANDLE ? target_pass : render_pass_;
        pipeline.subpass = 0;
        if (!vkOk(vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &pipeline, nullptr, &out),
                  diagnostics, "vkCreateGraphicsPipelines")) {
            return false;
        }
        ++stats_.pipeline_compiles;
        return true;
    }

    bool createPipelines(DiagnosticList& diagnostics) {
        // Descriptor set layout shared by the sampled-image pipelines.
        VkDescriptorSetLayoutBinding binding{};
        binding.binding = 0;
        binding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        binding.descriptorCount = 1;
        binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        VkDescriptorSetLayoutCreateInfo set_layout{};
        set_layout.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        set_layout.bindingCount = 1;
        set_layout.pBindings = &binding;
        if (!vkOk(vkCreateDescriptorSetLayout(device_, &set_layout, nullptr, &sampler_set_layout_),
                  diagnostics, "vkCreateDescriptorSetLayout")) {
            return false;
        }
        VkPushConstantRange push{};
        push.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
        push.offset = 0;
        push.size = 32;

        VkPipelineLayoutCreateInfo textured_layout{};
        textured_layout.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        textured_layout.setLayoutCount = 1;
        textured_layout.pSetLayouts = &sampler_set_layout_;
        textured_layout.pushConstantRangeCount = 1;
        textured_layout.pPushConstantRanges = &push;
        if (!vkOk(vkCreatePipelineLayout(device_, &textured_layout, nullptr, &textured_pipeline_layout_),
                  diagnostics, "vkCreatePipelineLayout")) {
            return false;
        }
        VkPipelineLayoutCreateInfo plain_layout{};
        plain_layout.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        plain_layout.pushConstantRangeCount = 1;
        plain_layout.pPushConstantRanges = &push;
        if (!vkOk(vkCreatePipelineLayout(device_, &plain_layout, nullptr, &plain_pipeline_layout_),
                  diagnostics, "vkCreatePipelineLayout(plain)")) {
            return false;
        }

        VkShaderModule image_vert = createShader(kImageVertSpv, sizeof(kImageVertSpv), diagnostics);
        VkShaderModule image_frag = createShader(kImageFragSpv, sizeof(kImageFragSpv), diagnostics);
        VkShaderModule boxes_vert = createShader(kBoxesVertSpv, sizeof(kBoxesVertSpv), diagnostics);
        VkShaderModule boxes_frag = createShader(kBoxesFragSpv, sizeof(kBoxesFragSpv), diagnostics);
        VkShaderModule text_vert = createShader(kTextVertSpv, sizeof(kTextVertSpv), diagnostics);
        VkShaderModule text_frag = createShader(kTextFragSpv, sizeof(kTextFragSpv), diagnostics);
        VkShaderModule circles_vert =
            createShader(kCirclesVertSpv, sizeof(kCirclesVertSpv), diagnostics);
        VkShaderModule circles_frag =
            createShader(kCirclesFragSpv, sizeof(kCirclesFragSpv), diagnostics);
        VkShaderModule polyline_vert =
            createShader(kPolylineVertSpv, sizeof(kPolylineVertSpv), diagnostics);
        VkShaderModule polyline_frag =
            createShader(kPolylineFragSpv, sizeof(kPolylineFragSpv), diagnostics);
        const bool modules_ok = image_vert != VK_NULL_HANDLE && image_frag != VK_NULL_HANDLE &&
                                boxes_vert != VK_NULL_HANDLE && boxes_frag != VK_NULL_HANDLE &&
                                text_vert != VK_NULL_HANDLE && text_frag != VK_NULL_HANDLE &&
                                circles_vert != VK_NULL_HANDLE && circles_frag != VK_NULL_HANDLE &&
                                polyline_vert != VK_NULL_HANDLE && polyline_frag != VK_NULL_HANDLE;
        bool ok = modules_ok;
        if (ok) {
            ok = createPipeline(image_vert, image_frag, true, nullptr, {}, textured_pipeline_layout_,
                                image_pipeline_, diagnostics);
        }
        if (ok) {
            VkVertexInputBindingDescription instance_binding{0, 16, VK_VERTEX_INPUT_RATE_INSTANCE};
            std::vector<VkVertexInputAttributeDescription> attributes = {
                {0, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 0}};
            ok = createPipeline(boxes_vert, boxes_frag, false, &instance_binding, attributes,
                                plain_pipeline_layout_, boxes_pipeline_, diagnostics);
        }
        if (ok) {
            VkVertexInputBindingDescription vertex_binding{0, sizeof(TextVertex),
                                                           VK_VERTEX_INPUT_RATE_VERTEX};
            std::vector<VkVertexInputAttributeDescription> attributes = {
                {0, 0, VK_FORMAT_R32G32_SFLOAT, 0}, {1, 0, VK_FORMAT_R32G32_SFLOAT, 8}};
            ok = createPipeline(text_vert, text_frag, true, &vertex_binding, attributes,
                                textured_pipeline_layout_, text_pipeline_, diagnostics);
        }
        if (ok) {
            VkVertexInputBindingDescription instance_binding{0, 8, VK_VERTEX_INPUT_RATE_INSTANCE};
            std::vector<VkVertexInputAttributeDescription> attributes = {
                {0, 0, VK_FORMAT_R32G32_SFLOAT, 0}};
            ok = createPipeline(circles_vert, circles_frag, false, &instance_binding, attributes,
                                plain_pipeline_layout_, circles_pipeline_, diagnostics);
        }
        if (ok) {
            VkVertexInputBindingDescription instance_binding{0, 16, VK_VERTEX_INPUT_RATE_INSTANCE};
            std::vector<VkVertexInputAttributeDescription> attributes = {
                {0, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 0}};
            ok = createPipeline(polyline_vert, polyline_frag, false, &instance_binding, attributes,
                                plain_pipeline_layout_, polyline_pipeline_, diagnostics);
        }
        VkShaderModule fullscreen_vert = VK_NULL_HANDLE;
        VkShaderModule blur_frag = VK_NULL_HANDLE;
        VkShaderModule color_frag = VK_NULL_HANDLE;
        if (ok && !model_.effects.empty()) {
            ok = createFilterPass(diagnostics);
            fullscreen_vert = createShader(kFullscreenVertSpv, sizeof(kFullscreenVertSpv), diagnostics);
            blur_frag = createShader(kBlurFragSpv, sizeof(kBlurFragSpv), diagnostics);
            color_frag = createShader(kColorFragSpv, sizeof(kColorFragSpv), diagnostics);
            ok = ok && fullscreen_vert != VK_NULL_HANDLE && blur_frag != VK_NULL_HANDLE &&
                 color_frag != VK_NULL_HANDLE;
            if (ok) {
                ok = createPipeline(fullscreen_vert, blur_frag, true, nullptr, {},
                                    textured_pipeline_layout_, blur_pipeline_, diagnostics,
                                    filter_pass_, /*blend=*/false, /*dynamic_viewport=*/true) &&
                     createPipeline(fullscreen_vert, color_frag, true, nullptr, {},
                                    textured_pipeline_layout_, color_pipeline_, diagnostics,
                                    filter_pass_, /*blend=*/false, /*dynamic_viewport=*/true);
            }
        }
        for (VkShaderModule module :
             {image_vert, image_frag, boxes_vert, boxes_frag, text_vert, text_frag, circles_vert,
              circles_frag, polyline_vert, polyline_frag, fullscreen_vert, blur_frag, color_frag}) {
            if (module != VK_NULL_HANDLE) {
                vkDestroyShaderModule(device_, module, nullptr);
            }
        }
        return ok;
    }

    // Offscreen pass for image-space effects: color rgba8, contents discarded
    // on load, left in SHADER_READ_ONLY for downstream sampling.
    bool createFilterPass(DiagnosticList& diagnostics) {
        VkAttachmentDescription color{};
        color.format = VK_FORMAT_R8G8B8A8_UNORM;
        color.samples = VK_SAMPLE_COUNT_1_BIT;
        color.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        color.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        color.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        color.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        color.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        VkAttachmentReference color_ref{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1;
        subpass.pColorAttachments = &color_ref;
        VkSubpassDependency deps[2] = {};
        deps[0].srcSubpass = VK_SUBPASS_EXTERNAL;
        deps[0].dstSubpass = 0;
        deps[0].srcStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        deps[0].srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
        deps[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        deps[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        deps[1].srcSubpass = 0;
        deps[1].dstSubpass = VK_SUBPASS_EXTERNAL;
        deps[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        deps[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        deps[1].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        deps[1].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        VkRenderPassCreateInfo pass{};
        pass.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        pass.attachmentCount = 1;
        pass.pAttachments = &color;
        pass.subpassCount = 1;
        pass.pSubpasses = &subpass;
        pass.dependencyCount = 2;
        pass.pDependencies = deps;
        return vkOk(vkCreateRenderPass(device_, &pass, nullptr, &filter_pass_), diagnostics,
                    "vkCreateRenderPass(filter)");
    }

    bool createSceneResources(DiagnosticList& diagnostics) {
        VkSamplerCreateInfo sampler{};
        sampler.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
        sampler.magFilter = VK_FILTER_LINEAR;
        sampler.minFilter = VK_FILTER_LINEAR;
        sampler.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        sampler.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        sampler.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        if (!vkOk(vkCreateSampler(device_, &sampler, nullptr, &sampler_), diagnostics,
                  "vkCreateSampler")) {
            return false;
        }

        // Per-draw retained state and shared streaming buffers.
        uint64_t total_instances = 0;
        uint64_t total_glyph_vertices = 0;
        uint64_t total_point_floats = 0;
        for (const render::DrawOp& draw : model_.draws) {
            DrawState state;
            state.op = &draw;
            if (draw.kind == render::DrawOp::Kind::kBoxes) {
                state.instance_offset = total_instances;
                total_instances += draw.max_instances;
            } else if (draw.kind == render::DrawOp::Kind::kCircles) {
                state.point_offset = total_point_floats;
                total_point_floats += draw.max_points * 2;  // vec2 per point
            } else if (draw.kind == render::DrawOp::Kind::kPolyline) {
                state.point_offset = total_point_floats;
                total_point_floats +=
                    (draw.max_points > 0 ? draw.max_points - 1 : 0) * 4;  // vec4 per segment
            } else if (draw.kind == render::DrawOp::Kind::kText) {
                state.vertex_offset = total_glyph_vertices;
                total_glyph_vertices += draw.max_glyphs * 6;
                if (atlases_.count(draw.font_uri) == 0) {
                    auto atlas = std::make_unique<render::TextAtlas>();
                    render::TextAtlas::Options atlas_options;
                    atlas_options.font_file = resolveFont(draw.font_uri);
                    atlas_options.pixel_size = options_.font_pixel_size;
                    atlas_options.glyph_capacity = static_cast<uint32_t>(
                        std::max<uint64_t>(1, std::min<uint64_t>(draw.glyph_capacity, 65536)));
                    if (!atlas->init(atlas_options, diagnostics)) {
                        return false;
                    }
                    Image image;
                    if (!createImage(atlas->width(), atlas->height(), VK_FORMAT_R8_UNORM,
                                     VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT, image,
                                     diagnostics)) {
                        return false;
                    }
                    AtlasEntry entry;
                    entry.atlas = std::move(atlas);
                    entry.image = image;
                    atlases_.emplace(draw.font_uri, std::move(entry));
                }
            }
            draw_states_.push_back(state);
        }
        if (total_instances > 0 &&
            !createBuffer(total_instances * 16, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                          true, instance_buffer_, diagnostics)) {
            return false;
        }
        if (total_glyph_vertices > 0 &&
            !createBuffer(total_glyph_vertices * sizeof(TextVertex), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                          true, text_vertex_buffer_, diagnostics)) {
            return false;
        }
        if (total_point_floats > 0 &&
            !createBuffer(total_point_floats * sizeof(float), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                          true, point_buffer_, diagnostics)) {
            return false;
        }

        uint64_t atlas_bytes = 0;
        for (const auto& [uri, entry] : atlases_) {
            atlas_bytes += static_cast<uint64_t>(entry.atlas->width()) * entry.atlas->height();
        }
        const uint64_t camera_bytes = static_cast<uint64_t>(model_.width) * model_.height * 4;
        if (!createBuffer(camera_bytes + atlas_bytes + 256, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                          true, staging_, diagnostics)) {
            return false;
        }
        if (!createBuffer(camera_bytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                          true, readback_, diagnostics)) {
            return false;
        }

        // Descriptor pool: one set per sampled image draw (inputs) + atlases
        // + three per effect (source / tmp / target sampling).
        uint32_t sampled_sets = static_cast<uint32_t>(atlases_.size());
        for (const render::DrawOp& draw : model_.draws) {
            if (draw.kind == render::DrawOp::Kind::kImage) {
                ++sampled_sets;
            }
        }
        sampled_sets += static_cast<uint32_t>(model_.effects.size()) * 3u;
        sampled_sets = std::max(sampled_sets, 1u);
        effect_states_.clear();
        effect_index_.clear();
        for (const render::EffectOp& effect : model_.effects) {
            EffectState state;
            state.op = &effect;
            effect_index_.emplace(effect.node_id, effect_states_.size());
            effect_states_.push_back(state);
        }
        VkDescriptorPoolSize pool_size{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, sampled_sets};
        VkDescriptorPoolCreateInfo pool{};
        pool.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        pool.maxSets = sampled_sets;
        pool.poolSizeCount = 1;
        pool.pPoolSizes = &pool_size;
        if (!vkOk(vkCreateDescriptorPool(device_, &pool, nullptr, &descriptor_pool_), diagnostics,
                  "vkCreateDescriptorPool")) {
            return false;
        }
        for (auto& [uri, entry] : atlases_) {
            if (!allocateSamplerSet(entry.image.view, entry.set, diagnostics)) {
                return false;
            }
        }
        return true;
    }

    bool allocateSamplerSet(VkImageView view, VkDescriptorSet& out, DiagnosticList& diagnostics) {
        VkDescriptorSetAllocateInfo alloc{};
        alloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        alloc.descriptorPool = descriptor_pool_;
        alloc.descriptorSetCount = 1;
        alloc.pSetLayouts = &sampler_set_layout_;
        if (!vkOk(vkAllocateDescriptorSets(device_, &alloc, &out), diagnostics,
                  "vkAllocateDescriptorSets")) {
            return false;
        }
        updateSamplerSet(out, view);
        return true;
    }

    void updateSamplerSet(VkDescriptorSet set, VkImageView view) {
        VkDescriptorImageInfo image{};
        image.sampler = sampler_;
        image.imageView = view;
        image.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        VkWriteDescriptorSet write{};
        write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        write.dstSet = set;
        write.dstBinding = 0;
        write.descriptorCount = 1;
        write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        write.pImageInfo = &image;
        vkUpdateDescriptorSets(device_, 1, &write, 0, nullptr);
    }

    bool createFilterFramebuffer(const Image& image, VkFramebuffer& out,
                                 DiagnosticList& diagnostics) {
        VkFramebufferCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        info.renderPass = filter_pass_;
        info.attachmentCount = 1;
        info.pAttachments = &image.view;
        info.width = image.width;
        info.height = image.height;
        info.layers = 1;
        return vkOk(vkCreateFramebuffer(device_, &info, nullptr, &out), diagnostics,
                    "vkCreateFramebuffer(filter)");
    }

    void destroyEffectImages(EffectState& state) {
        if (state.fb != VK_NULL_HANDLE) {
            vkDestroyFramebuffer(device_, state.fb, nullptr);
            state.fb = VK_NULL_HANDLE;
        }
        if (state.fb_tmp != VK_NULL_HANDLE) {
            vkDestroyFramebuffer(device_, state.fb_tmp, nullptr);
            state.fb_tmp = VK_NULL_HANDLE;
        }
        destroyImage(state.target);
        destroyImage(state.tmp);
    }

    // Lazily (re)creates effect intermediates to match their source size and
    // keeps descriptor sets pointing at current views.
    bool prepareEffects(DiagnosticList& diagnostics) {
        for (EffectState& state : effect_states_) {
            const Image* source = nullptr;
            if (!state.op->source_input.empty()) {
                auto it = input_images_.find(state.op->source_input);
                if (it != input_images_.end() && it->second.image != VK_NULL_HANDLE) {
                    source = &it->second;
                }
            } else if (!state.op->source_effect.empty()) {
                auto it = effect_index_.find(state.op->source_effect);
                if (it != effect_index_.end() && effect_states_[it->second].ready) {
                    source = &effect_states_[it->second].target;
                }
            }
            if (source == nullptr) {
                state.ready = false;
                continue;
            }
            const bool is_blur = state.op->kind == render::EffectOp::Kind::kBlur;
            if (state.target.image == VK_NULL_HANDLE || state.target.width != source->width ||
                state.target.height != source->height) {
                vkDeviceWaitIdle(device_);
                destroyEffectImages(state);
                const VkImageUsageFlags usage =
                    VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
                if (!createImage(source->width, source->height, VK_FORMAT_R8G8B8A8_UNORM, usage,
                                 state.target, diagnostics) ||
                    !createFilterFramebuffer(state.target, state.fb, diagnostics)) {
                    return false;
                }
                if (is_blur &&
                    (!createImage(source->width, source->height, VK_FORMAT_R8G8B8A8_UNORM, usage,
                                  state.tmp, diagnostics) ||
                     !createFilterFramebuffer(state.tmp, state.fb_tmp, diagnostics))) {
                    return false;
                }
                if (state.target_set == VK_NULL_HANDLE) {
                    if (!allocateSamplerSet(state.target.view, state.target_set, diagnostics)) {
                        return false;
                    }
                } else {
                    updateSamplerSet(state.target_set, state.target.view);
                }
                if (is_blur) {
                    if (state.tmp_set == VK_NULL_HANDLE) {
                        if (!allocateSamplerSet(state.tmp.view, state.tmp_set, diagnostics)) {
                            return false;
                        }
                    } else {
                        updateSamplerSet(state.tmp_set, state.tmp.view);
                    }
                }
            }
            if (state.source_set == VK_NULL_HANDLE) {
                if (!allocateSamplerSet(source->view, state.source_set, diagnostics)) {
                    return false;
                }
                state.last_source_view = source->view;
            } else if (state.last_source_view != source->view) {
                updateSamplerSet(state.source_set, source->view);
                state.last_source_view = source->view;
            }
            state.ready = true;
        }
        return true;
    }

    std::string resolveFont(const std::string& uri) const {
        auto it = options_.font_files.find(uri);
        if (it != options_.font_files.end()) {
            return it->second;
        }
        static const char* kCandidates[] = {
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        };
        for (const char* candidate : kCandidates) {
            std::FILE* file = std::fopen(candidate, "rb");
            if (file != nullptr) {
                std::fclose(file);
                return candidate;
            }
        }
        return {};
    }

    void destroy() {
        if (device_ != VK_NULL_HANDLE) {
            vkDeviceWaitIdle(device_);
            for (auto& [uri, entry] : atlases_) {
                destroyImage(entry.image);
            }
            atlases_.clear();
            for (EffectState& effect : effect_states_) {
                destroyEffectImages(effect);
            }
            effect_states_.clear();
            if (filter_pass_ != VK_NULL_HANDLE) {
                vkDestroyRenderPass(device_, filter_pass_, nullptr);
                filter_pass_ = VK_NULL_HANDLE;
            }
            for (VkPipeline pipeline : {blur_pipeline_, color_pipeline_}) {
                if (pipeline != VK_NULL_HANDLE) {
                    vkDestroyPipeline(device_, pipeline, nullptr);
                }
            }
            blur_pipeline_ = VK_NULL_HANDLE;
            color_pipeline_ = VK_NULL_HANDLE;
            for (auto& [name, image] : input_images_) {
                destroyImage(image);
            }
            input_images_.clear();
            destroyBuffer(instance_buffer_);
            destroyBuffer(point_buffer_);
            destroyBuffer(text_vertex_buffer_);
            destroyBuffer(staging_);
            destroyBuffer(readback_);
            if (descriptor_pool_ != VK_NULL_HANDLE) {
                vkDestroyDescriptorPool(device_, descriptor_pool_, nullptr);
            }
            if (sampler_ != VK_NULL_HANDLE) {
                vkDestroySampler(device_, sampler_, nullptr);
            }
            for (VkPipeline pipeline : {image_pipeline_, boxes_pipeline_, circles_pipeline_,
                                        polyline_pipeline_, text_pipeline_}) {
                if (pipeline != VK_NULL_HANDLE) {
                    vkDestroyPipeline(device_, pipeline, nullptr);
                }
            }
            if (textured_pipeline_layout_ != VK_NULL_HANDLE) {
                vkDestroyPipelineLayout(device_, textured_pipeline_layout_, nullptr);
            }
            if (plain_pipeline_layout_ != VK_NULL_HANDLE) {
                vkDestroyPipelineLayout(device_, plain_pipeline_layout_, nullptr);
            }
            if (sampler_set_layout_ != VK_NULL_HANDLE) {
                vkDestroyDescriptorSetLayout(device_, sampler_set_layout_, nullptr);
            }
            if (framebuffer_ != VK_NULL_HANDLE) {
                vkDestroyFramebuffer(device_, framebuffer_, nullptr);
            }
            if (render_pass_ != VK_NULL_HANDLE) {
                vkDestroyRenderPass(device_, render_pass_, nullptr);
            }
            destroyImage(target_);
            if (query_pool_ != VK_NULL_HANDLE) {
                vkDestroyQueryPool(device_, query_pool_, nullptr);
            }
            if (fence_ != VK_NULL_HANDLE) {
                vkDestroyFence(device_, fence_, nullptr);
            }
            if (command_pool_ != VK_NULL_HANDLE) {
                vkDestroyCommandPool(device_, command_pool_, nullptr);
            }
            vkDestroyDevice(device_, nullptr);
            device_ = VK_NULL_HANDLE;
        }
        if (instance_ != VK_NULL_HANDLE) {
            vkDestroyInstance(instance_, nullptr);
            instance_ = VK_NULL_HANDLE;
        }
    }

    // ---- state ---------------------------------------------------------------

    struct AtlasEntry {
        std::unique_ptr<render::TextAtlas> atlas;
        Image image;
        VkDescriptorSet set = VK_NULL_HANDLE;
        uint64_t uploaded_revision = 0;
    };

    struct DrawState {
        const render::DrawOp* op = nullptr;
        // boxes
        uint64_t instance_offset = 0;
        uint32_t instance_count = 0;
        render::BoxSmoother smoother;
        // circles / polyline (offset in floats within point_buffer_)
        uint64_t point_offset = 0;
        uint32_t point_instance_count = 0;
        // text
        uint64_t vertex_offset = 0;
        uint32_t vertex_count = 0;
        std::string last_text;
        bool text_valid = false;
        // image
        float dst_rect[4] = {0, 0, 0, 0};
        bool image_ready = false;
    };

    RendererOptions options_;
    render::SceneModel model_;
    RenderStats stats_;
    bool loaded_ = false;
    std::chrono::steady_clock::time_point last_frame_time_;
    bool has_last_frame_time_ = false;
    double frame_dt_ = 1.0 / 30.0;
    std::map<std::string, float> runtime_params_;

    VkInstance instance_ = VK_NULL_HANDLE;
    VkPhysicalDevice physical_ = VK_NULL_HANDLE;
    uint32_t graphics_family_ = 0;
    std::string device_name_;
    float timestamp_period_ns_ = 0.0f;
    VkDevice device_ = VK_NULL_HANDLE;
    VkQueue queue_ = VK_NULL_HANDLE;
    VkCommandPool command_pool_ = VK_NULL_HANDLE;
    VkCommandBuffer frame_cmd_ = VK_NULL_HANDLE;
    VkCommandBuffer aux_cmd_ = VK_NULL_HANDLE;
    VkFence fence_ = VK_NULL_HANDLE;
    VkQueryPool query_pool_ = VK_NULL_HANDLE;

    Image target_;
    VkRenderPass render_pass_ = VK_NULL_HANDLE;
    VkFramebuffer framebuffer_ = VK_NULL_HANDLE;
    VkRenderPass filter_pass_ = VK_NULL_HANDLE;
    VkPipeline blur_pipeline_ = VK_NULL_HANDLE;
    VkPipeline color_pipeline_ = VK_NULL_HANDLE;
    std::vector<EffectState> effect_states_;
    std::map<std::string, size_t> effect_index_;

    VkDescriptorSetLayout sampler_set_layout_ = VK_NULL_HANDLE;
    VkPipelineLayout textured_pipeline_layout_ = VK_NULL_HANDLE;
    VkPipelineLayout plain_pipeline_layout_ = VK_NULL_HANDLE;
    VkPipeline image_pipeline_ = VK_NULL_HANDLE;
    VkPipeline boxes_pipeline_ = VK_NULL_HANDLE;
    VkPipeline circles_pipeline_ = VK_NULL_HANDLE;
    VkPipeline polyline_pipeline_ = VK_NULL_HANDLE;
    VkPipeline text_pipeline_ = VK_NULL_HANDLE;
    VkSampler sampler_ = VK_NULL_HANDLE;
    VkDescriptorPool descriptor_pool_ = VK_NULL_HANDLE;

    Buffer instance_buffer_;
    Buffer point_buffer_;
    Buffer text_vertex_buffer_;
    Buffer staging_;
    Buffer readback_;

    std::map<std::string, Image> input_images_;          // scene input -> texture
    std::map<std::string, VkDescriptorSet> input_sets_;  // scene input -> descriptor
    std::map<std::string, AtlasEntry> atlases_;          // font uri -> atlas
    std::vector<DrawState> draw_states_;

    bool submitFrame(const std::vector<std::pair<Image*, VkBufferImageCopy>>& uploads,
                     DiagnosticList& diagnostics);
};

bool VulkanRenderer::renderFrame(const FrameInputs& inputs, DiagnosticList& diagnostics) {
    if (!loaded_) {
        diag(diagnostics, "compile.invalid_input", "renderFrame called before loadScene");
        return false;
    }
    const auto cpu_start = std::chrono::steady_clock::now();
    // Wall-clock delta drives rate-independent smoothing; clamped so pauses
    // and first frames stay well-behaved.
    if (has_last_frame_time_) {
        frame_dt_ = std::clamp(
            std::chrono::duration<double>(cpu_start - last_frame_time_).count(), 0.001, 0.1);
    }
    last_frame_time_ = cpu_start;
    has_last_frame_time_ = true;

    // ---- CPU snapshot phase: fill streaming buffers, decide uploads --------
    std::vector<std::pair<Image*, VkBufferImageCopy>> uploads;
    VkDeviceSize staging_offset = 0;
    const auto stage_bytes = [&](const void* data, VkDeviceSize bytes) -> VkDeviceSize {
        const VkDeviceSize at = staging_offset;
        std::memcpy(static_cast<uint8_t*>(staging_.mapped) + at, data, bytes);
        staging_offset = (at + bytes + 15) & ~VkDeviceSize(15);
        return at;
    };

    for (DrawState& state : draw_states_) {
        const render::DrawOp& op = *state.op;
        if (op.kind == render::DrawOp::Kind::kImage) {
            auto it = inputs.images.find(op.source_input);
            if (it == inputs.images.end() || it->second.pixels == nullptr) {
                continue;  // keep last uploaded frame if any
            }
            const CpuImageView& view = it->second;
            Image& texture = input_images_[op.source_input];
            if (texture.image == VK_NULL_HANDLE || texture.width != view.width ||
                texture.height != view.height) {
                vkDeviceWaitIdle(device_);
                const bool had_set = input_sets_.count(op.source_input) != 0;
                destroyImage(texture);
                if (!createImage(view.width, view.height, VK_FORMAT_R8G8B8A8_UNORM,
                                 VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT, texture,
                                 diagnostics)) {
                    return false;
                }
                if (!had_set) {
                    VkDescriptorSet set = VK_NULL_HANDLE;
                    if (!allocateSamplerSet(texture.view, set, diagnostics)) {
                        return false;
                    }
                    input_sets_[op.source_input] = set;
                } else {
                    updateSamplerSet(input_sets_[op.source_input], texture.view);
                }
            }
            const VkDeviceSize bytes = VkDeviceSize(view.width) * view.height * 4;
            if (staging_offset + bytes > staging_.size) {
                diag(diagnostics, "compile.budget_exceeded",
                     "frame inputs exceed the staging pool; input image larger than budgets allow");
                return false;
            }
            VkBufferImageCopy copy{};
            copy.bufferOffset = stage_bytes(view.pixels, bytes);
            copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
            copy.imageExtent = {view.width, view.height, 1};
            uploads.emplace_back(&texture, copy);
            ++stats_.image_uploads;

            // Fit computation (contain | cover | fill).
            const float target_w = static_cast<float>(model_.width);
            const float target_h = static_cast<float>(model_.height);
            const float source_w = static_cast<float>(view.width);
            const float source_h = static_cast<float>(view.height);
            float scale_x = target_w / source_w;
            float scale_y = target_h / source_h;
            if (op.fit == "contain") {
                scale_x = scale_y = std::min(scale_x, scale_y);
            } else if (op.fit == "cover") {
                scale_x = scale_y = std::max(scale_x, scale_y);
            }
            const float dst_w = source_w * scale_x;
            const float dst_h = source_h * scale_y;
            state.dst_rect[0] = (target_w - dst_w) * 0.5f;
            state.dst_rect[1] = (target_h - dst_h) * 0.5f;
            state.dst_rect[2] = dst_w;
            state.dst_rect[3] = dst_h;
            state.image_ready = true;
        } else if (op.kind == render::DrawOp::Kind::kBoxes) {
            std::vector<DetectionInstance> detections;
            auto it = inputs.detections.find(op.source_input);
            if (it != inputs.detections.end()) {
                detections = it->second;
            }
            if (detections.size() > op.max_instances) {
                if (op.overflow == "drop_lowest_score") {
                    std::sort(detections.begin(), detections.end(),
                              [](const DetectionInstance& a, const DetectionInstance& b) {
                                  return a.score > b.score;
                              });
                    detections.resize(op.max_instances);
                } else {  // drop_oldest: keep the newest tail
                    detections.erase(detections.begin(),
                                     detections.end() - static_cast<ptrdiff_t>(op.max_instances));
                }
            }
            if (op.smoothing > 0.0f) {
                detections = state.smoother.update(std::move(detections), frame_dt_, op.smoothing,
                                                   op.max_instances);
            }
            float* dst = static_cast<float*>(instance_buffer_.mapped) + state.instance_offset * 4;
            for (const DetectionInstance& detection : detections) {
                std::memcpy(dst, detection.bbox, sizeof(float) * 4);
                dst += 4;
            }
            state.instance_count = static_cast<uint32_t>(detections.size());
            if (state.instance_count > 0) {
                ++stats_.buffer_uploads;
            }
        } else if (op.kind == render::DrawOp::Kind::kCircles ||
                   op.kind == render::DrawOp::Kind::kPolyline) {
            std::vector<Point2f> points;
            auto it = inputs.points.find(op.source_input);
            if (it != inputs.points.end()) {
                points = it->second;
            }
            if (points.size() > op.max_points) {
                points.resize(op.max_points);  // declared truncate_end rule
            }
            float* dst = static_cast<float*>(point_buffer_.mapped) + state.point_offset;
            if (op.kind == render::DrawOp::Kind::kCircles) {
                for (const Point2f& point : points) {
                    dst[0] = point.x;
                    dst[1] = point.y;
                    dst += 2;
                }
                state.point_instance_count = static_cast<uint32_t>(points.size());
            } else {
                const size_t segments = points.size() > 1 ? points.size() - 1 : 0;
                for (size_t s = 0; s < segments; ++s) {
                    dst[0] = points[s].x;
                    dst[1] = points[s].y;
                    dst[2] = points[s + 1].x;
                    dst[3] = points[s + 1].y;
                    dst += 4;
                }
                state.point_instance_count = static_cast<uint32_t>(segments);
            }
            if (state.point_instance_count > 0) {
                ++stats_.buffer_uploads;
            }
        } else {
            auto it = inputs.strings.find(op.source_input);
            const std::string text = it != inputs.strings.end() ? it->second : op.default_text;
            AtlasEntry& entry = atlases_.at(op.font_uri);
            if (!state.text_valid || text != state.last_text) {
                std::vector<render::TextAtlas::GlyphQuad> quads;
                entry.atlas->layout(text, op.position[0], op.position[1], op.max_glyphs, quads);
                TextVertex* dst = static_cast<TextVertex*>(text_vertex_buffer_.mapped) +
                                  state.vertex_offset;
                for (const render::TextAtlas::GlyphQuad& quad : quads) {
                    const TextVertex v0{{quad.x, quad.y}, {quad.u0, quad.v0}};
                    const TextVertex v1{{quad.x + quad.w, quad.y}, {quad.u1, quad.v0}};
                    const TextVertex v2{{quad.x + quad.w, quad.y + quad.h}, {quad.u1, quad.v1}};
                    const TextVertex v3{{quad.x, quad.y + quad.h}, {quad.u0, quad.v1}};
                    dst[0] = v0; dst[1] = v1; dst[2] = v2;
                    dst[3] = v0; dst[4] = v2; dst[5] = v3;
                    dst += 6;
                }
                state.vertex_count = static_cast<uint32_t>(quads.size() * 6);
                state.last_text = text;
                state.text_valid = true;
                ++stats_.buffer_uploads;
            }
            stats_.missing_glyphs = 0;
            for (const auto& [uri, atlas_entry] : atlases_) {
                stats_.missing_glyphs += atlas_entry.atlas->missingGlyphs();
            }
            if (entry.uploaded_revision != entry.atlas->revision()) {
                const VkDeviceSize bytes =
                    VkDeviceSize(entry.atlas->width()) * entry.atlas->height();
                if (staging_offset + bytes > staging_.size) {
                    diag(diagnostics, "compile.budget_exceeded", "atlas upload exceeds staging pool");
                    return false;
                }
                VkBufferImageCopy copy{};
                copy.bufferOffset = stage_bytes(entry.atlas->pixels(), bytes);
                copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
                copy.imageExtent = {entry.atlas->width(), entry.atlas->height(), 1};
                uploads.emplace_back(&entry.image, copy);
                entry.uploaded_revision = entry.atlas->revision();
                ++stats_.atlas_updates;
            }
        }
    }

    if (!effect_states_.empty() && !prepareEffects(diagnostics)) {
        return false;
    }
    if (!submitFrame(uploads, diagnostics)) {
        return false;
    }
    ++stats_.frames;
    stats_.last_cpu_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - cpu_start)
            .count();
    return true;
}

bool VulkanRenderer::submitFrame(const std::vector<std::pair<Image*, VkBufferImageCopy>>& uploads,
                                 DiagnosticList& diagnostics) {
    VkCommandBufferBeginInfo begin{};
    begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    if (!vkOk(vkResetCommandBuffer(frame_cmd_, 0), diagnostics, "vkResetCommandBuffer") ||
        !vkOk(vkBeginCommandBuffer(frame_cmd_, &begin), diagnostics, "vkBeginCommandBuffer")) {
        return false;
    }
    vkCmdResetQueryPool(frame_cmd_, query_pool_, 0, 2);
    vkCmdWriteTimestamp(frame_cmd_, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, query_pool_, 0);

    for (const auto& [image, copy] : uploads) {
        recordImageBarrier(frame_cmd_, *image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                           VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_ACCESS_SHADER_READ_BIT,
                           VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_WRITE_BIT);
        vkCmdCopyBufferToImage(frame_cmd_, staging_.buffer, image->image,
                               VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);
        recordImageBarrier(frame_cmd_, *image, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                           VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_WRITE_BIT,
                           VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_ACCESS_SHADER_READ_BIT);
    }

    // ---- image-space effect passes (before the composite pass) -------------
    const auto recordFilter = [&](VkFramebuffer framebuffer, Image& destination, VkPipeline pipeline,
                                  VkDescriptorSet set, const void* pc, uint32_t pc_size) {
        VkRenderPassBeginInfo begin_pass{};
        begin_pass.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        begin_pass.renderPass = filter_pass_;
        begin_pass.framebuffer = framebuffer;
        begin_pass.renderArea = {{0, 0}, {destination.width, destination.height}};
        vkCmdBeginRenderPass(frame_cmd_, &begin_pass, VK_SUBPASS_CONTENTS_INLINE);
        VkViewport viewport{0.0f, 0.0f, static_cast<float>(destination.width),
                            static_cast<float>(destination.height), 0.0f, 1.0f};
        VkRect2D scissor{{0, 0}, {destination.width, destination.height}};
        vkCmdSetViewport(frame_cmd_, 0, 1, &viewport);
        vkCmdSetScissor(frame_cmd_, 0, 1, &scissor);
        vkCmdBindPipeline(frame_cmd_, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
        vkCmdBindDescriptorSets(frame_cmd_, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                textured_pipeline_layout_, 0, 1, &set, 0, nullptr);
        vkCmdPushConstants(frame_cmd_, textured_pipeline_layout_,
                           VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, pc_size, pc);
        vkCmdDraw(frame_cmd_, 6, 1, 0, 0);
        vkCmdEndRenderPass(frame_cmd_);
        destination.layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    };
    for (EffectState& effect : effect_states_) {
        if (!effect.ready) {
            continue;
        }
        if (effect.op->kind == render::EffectOp::Kind::kBlur) {
            const float radius = std::clamp(
                effectValue(effect.op->radius, effect.op->radius_param), 0.5f, 16.0f);
            PcBlur horizontal{{1.0f / static_cast<float>(effect.tmp.width), 0.0f}, radius, 0.0f};
            recordFilter(effect.fb_tmp, effect.tmp, blur_pipeline_, effect.source_set, &horizontal,
                         sizeof(horizontal));
            PcBlur vertical{{0.0f, 1.0f / static_cast<float>(effect.target.height)}, radius, 0.0f};
            recordFilter(effect.fb, effect.target, blur_pipeline_, effect.tmp_set, &vertical,
                         sizeof(vertical));
        } else {
            PcColor pc{effectValue(effect.op->brightness, effect.op->brightness_param),
                       effectValue(effect.op->contrast, effect.op->contrast_param),
                       effectValue(effect.op->saturation, effect.op->saturation_param),
                       effectValue(effect.op->gamma, effect.op->gamma_param)};
            recordFilter(effect.fb, effect.target, color_pipeline_, effect.source_set, &pc,
                         sizeof(pc));
        }
    }

    VkClearValue clear{};
    clear.color = {{0.0f, 0.0f, 0.0f, 1.0f}};
    VkRenderPassBeginInfo pass{};
    pass.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    pass.renderPass = render_pass_;
    pass.framebuffer = framebuffer_;
    pass.renderArea = {{0, 0}, {model_.width, model_.height}};
    pass.clearValueCount = 1;
    pass.pClearValues = &clear;
    vkCmdBeginRenderPass(frame_cmd_, &pass, VK_SUBPASS_CONTENTS_INLINE);

    const float viewport_w = static_cast<float>(model_.width);
    const float viewport_h = static_cast<float>(model_.height);
    for (const DrawState& state : draw_states_) {
        const render::DrawOp& op = *state.op;
        if (op.kind == render::DrawOp::Kind::kImage) {
            if (!state.image_ready) {
                continue;
            }
            VkDescriptorSet set = VK_NULL_HANDLE;
            if (!op.effect_chain.empty()) {
                const EffectState& effect =
                    effect_states_[effect_index_.at(op.effect_chain.back())];
                if (!effect.ready) {
                    continue;
                }
                set = effect.target_set;
            } else {
                set = input_sets_.at(op.source_input);
            }
            vkCmdBindPipeline(frame_cmd_, VK_PIPELINE_BIND_POINT_GRAPHICS, image_pipeline_);
            vkCmdBindDescriptorSets(frame_cmd_, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                    textured_pipeline_layout_, 0, 1, &set, 0, nullptr);
            PcImage pc{};
            std::memcpy(pc.dst_rect, state.dst_rect, sizeof(pc.dst_rect));
            pc.viewport[0] = viewport_w;
            pc.viewport[1] = viewport_h;
            vkCmdPushConstants(frame_cmd_, textured_pipeline_layout_,
                               VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(pc),
                               &pc);
            vkCmdDraw(frame_cmd_, 6, 1, 0, 0);
        } else if (op.kind == render::DrawOp::Kind::kBoxes) {
            if (state.instance_count == 0) {
                continue;
            }
            vkCmdBindPipeline(frame_cmd_, VK_PIPELINE_BIND_POINT_GRAPHICS, boxes_pipeline_);
            const VkDeviceSize offset = state.instance_offset * 16;
            vkCmdBindVertexBuffers(frame_cmd_, 0, 1, &instance_buffer_.buffer, &offset);
            PcBoxes pc{};
            std::memcpy(pc.color, op.color, sizeof(pc.color));
            pc.viewport[0] = viewport_w;
            pc.viewport[1] = viewport_h;
            pc.thickness = kBoxOutlineThickness;
            vkCmdPushConstants(frame_cmd_, plain_pipeline_layout_,
                               VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(pc),
                               &pc);
            vkCmdDraw(frame_cmd_, 6, state.instance_count, 0, 0);
        } else if (op.kind == render::DrawOp::Kind::kCircles ||
                   op.kind == render::DrawOp::Kind::kPolyline) {
            if (state.point_instance_count == 0) {
                continue;
            }
            const bool is_circles = op.kind == render::DrawOp::Kind::kCircles;
            vkCmdBindPipeline(frame_cmd_, VK_PIPELINE_BIND_POINT_GRAPHICS,
                              is_circles ? circles_pipeline_ : polyline_pipeline_);
            const VkDeviceSize offset = state.point_offset * sizeof(float);
            vkCmdBindVertexBuffers(frame_cmd_, 0, 1, &point_buffer_.buffer, &offset);
            if (is_circles) {
                PcCircles pc{};
                std::memcpy(pc.color, op.color, sizeof(pc.color));
                pc.viewport[0] = viewport_w;
                pc.viewport[1] = viewport_h;
                pc.radius = op.radius;
                pc.thickness = op.thickness;
                vkCmdPushConstants(frame_cmd_, plain_pipeline_layout_,
                                   VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0,
                                   sizeof(pc), &pc);
            } else {
                PcPolyline pc{};
                std::memcpy(pc.color, op.color, sizeof(pc.color));
                pc.viewport[0] = viewport_w;
                pc.viewport[1] = viewport_h;
                pc.thickness = op.thickness;
                vkCmdPushConstants(frame_cmd_, plain_pipeline_layout_,
                                   VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0,
                                   sizeof(pc), &pc);
            }
            vkCmdDraw(frame_cmd_, 6, state.point_instance_count, 0, 0);
        } else {
            if (state.vertex_count == 0) {
                continue;
            }
            vkCmdBindPipeline(frame_cmd_, VK_PIPELINE_BIND_POINT_GRAPHICS, text_pipeline_);
            const AtlasEntry& entry = atlases_.at(op.font_uri);
            vkCmdBindDescriptorSets(frame_cmd_, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                    textured_pipeline_layout_, 0, 1, &entry.set, 0, nullptr);
            const VkDeviceSize offset = state.vertex_offset * sizeof(TextVertex);
            vkCmdBindVertexBuffers(frame_cmd_, 0, 1, &text_vertex_buffer_.buffer, &offset);
            PcText pc{};
            pc.viewport[0] = viewport_w;
            pc.viewport[1] = viewport_h;
            if (op.shadow) {
                pc.color[0] = pc.color[1] = pc.color[2] = 0.0f;
                pc.color[3] = 0.7f;
                pc.offset[0] = pc.offset[1] = kShadowOffset;
                vkCmdPushConstants(frame_cmd_, textured_pipeline_layout_,
                                   VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0,
                                   sizeof(pc), &pc);
                vkCmdDraw(frame_cmd_, state.vertex_count, 1, 0, 0);
            }
            std::memcpy(pc.color, op.color, sizeof(pc.color));
            pc.offset[0] = pc.offset[1] = 0.0f;
            vkCmdPushConstants(frame_cmd_, textured_pipeline_layout_,
                               VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(pc),
                               &pc);
            vkCmdDraw(frame_cmd_, state.vertex_count, 1, 0, 0);
        }
    }
    vkCmdEndRenderPass(frame_cmd_);
    target_.layout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;  // render pass final layout
    vkCmdWriteTimestamp(frame_cmd_, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, query_pool_, 1);
    if (!vkOk(vkEndCommandBuffer(frame_cmd_), diagnostics, "vkEndCommandBuffer")) {
        return false;
    }

    VkSubmitInfo submit{};
    submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &frame_cmd_;
    if (!vkOk(vkQueueSubmit(queue_, 1, &submit, fence_), diagnostics, "vkQueueSubmit") ||
        !vkOk(vkWaitForFences(device_, 1, &fence_, VK_TRUE, UINT64_MAX), diagnostics,
              "vkWaitForFences")) {
        return false;
    }
    vkResetFences(device_, 1, &fence_);

    uint64_t timestamps[2] = {0, 0};
    if (vkGetQueryPoolResults(device_, query_pool_, 0, 2, sizeof(timestamps), timestamps,
                              sizeof(uint64_t),
                              VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT) == VK_SUCCESS) {
        stats_.last_gpu_ms = static_cast<double>(timestamps[1] - timestamps[0]) *
                             static_cast<double>(timestamp_period_ns_) / 1e6;
    }
    return true;
}

bool VulkanRenderer::readback(std::vector<uint8_t>& pixels, uint32_t& width, uint32_t& height) {
    if (!loaded_ || target_.layout != VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL) {
        return false;
    }
    VkCommandBufferBeginInfo begin{};
    begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkResetCommandBuffer(aux_cmd_, 0);
    vkBeginCommandBuffer(aux_cmd_, &begin);
    VkBufferImageCopy copy{};
    copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    copy.imageExtent = {model_.width, model_.height, 1};
    vkCmdCopyImageToBuffer(aux_cmd_, target_.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                           readback_.buffer, 1, &copy);
    vkEndCommandBuffer(aux_cmd_);
    VkSubmitInfo submit{};
    submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &aux_cmd_;
    if (vkQueueSubmit(queue_, 1, &submit, fence_) != VK_SUCCESS ||
        vkWaitForFences(device_, 1, &fence_, VK_TRUE, UINT64_MAX) != VK_SUCCESS) {
        return false;
    }
    vkResetFences(device_, 1, &fence_);
    const size_t bytes = static_cast<size_t>(model_.width) * model_.height * 4;
    pixels.resize(bytes);
    std::memcpy(pixels.data(), readback_.mapped, bytes);
    width = model_.width;
    height = model_.height;
    return true;
}

}  // namespace

std::unique_ptr<Renderer> createVulkanRenderer(const RendererOptions& options,
                                               DiagnosticList& diagnostics) {
    auto renderer = std::make_unique<VulkanRenderer>(options);
    if (!renderer->init(diagnostics)) {
        return nullptr;
    }
    return renderer;
}

}  // namespace fluent_scene
