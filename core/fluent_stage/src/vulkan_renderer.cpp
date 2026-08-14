// vulkan_renderer.cpp — the production GPU backend (§11, Phase L1).
//
// Structure mirrors the CPU reference draw-for-draw: the shared plan
// (render_shared.hpp) makes the same offscreen decisions and extents, the
// shared shape/filter bodies run as SPIR-V, coverage masks union multi-part
// content with MAX blending, and the four blend modes are the same
// fixed-function equations the CPU implements in blendPremul(). What
// differs is only who touches the pixels.
//
// Retained-mode contract: every pipeline is built at construction from
// SPIR-V embedded at build time — zero shader compilation at runtime.
// Per frame: upload changed data (camera frames, new glyphs, polygon
// points), record one command buffer, submit, read back.

#include "fluent_stage/vulkan_renderer.hpp"

#include <vulkan/vulkan.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "fluent_stage/stage.hpp"
#include "render_shared.hpp"
#include "text_atlas.hpp"

// Draw-mode ids shared with shape_mask.frag.
#include "shape_modes.h"

namespace fluent_stage {

namespace {

using plan::dashSegments;
using plan::IRect;
using plan::pathSegments;
using plan::rectPad;
using plan::rectUnion;
using plan::scaleOf;
using plan::Seg;
using plan::segBounds;
using plan::targetBBox;

// ---- embedded SPIR-V (glslc -mfmt=c output) --------------------------------

const uint32_t kQuadVert[] =
#include "quad.vert.inc"
    ;
const uint32_t kFullscreenVert[] =
#include "fullscreen.vert.inc"
    ;
const uint32_t kShapeMaskFrag[] =
#include "shape_mask.frag.inc"
    ;
const uint32_t kGlyphMaskFrag[] =
#include "glyph_mask.frag.inc"
    ;
const uint32_t kMaskCompositeFrag[] =
#include "mask_composite.frag.inc"
    ;
const uint32_t kShapeColorFrag[] =
#include "shape_color.frag.inc"
    ;
const uint32_t kImageFrag[] =
#include "image.frag.inc"
    ;
const uint32_t kFilterFrag[] =
#include "filter.frag.inc"
    ;
const uint32_t kBlurFrag[] =
#include "blur.frag.inc"
    ;
const uint32_t kCompositeFrag[] =
#include "composite.frag.inc"
    ;
const uint32_t kUnpremulFrag[] =
#include "unpremul.frag.inc"
    ;

// ---- small helpers ---------------------------------------------------------

void vkCheck(VkResult r, const char* what) {
    if (r != VK_SUCCESS) {
        throw std::runtime_error(std::string("fluent_stage vulkan: ") + what + " failed (" +
                                 std::to_string(static_cast<int>(r)) + ")");
    }
}

// The push-constant struct — must match push_common.glsl exactly.
struct Push {
    float rect[4];
    float meta[4];
    float inv0[4];
    float inv1[4];
    float pa[4];
    float pb[4];
    float px2[4];
    float color[4];
};
static_assert(sizeof(Push) == 128, "push constants must stay at the 128-byte spec minimum");

void setInv(Push& p, const Mat23& inv) {
    p.inv0[0] = inv.a;
    p.inv0[1] = inv.c;
    p.inv0[2] = inv.tx;
    p.inv1[0] = inv.b;
    p.inv1[1] = inv.d;
    p.inv1[2] = inv.ty;
}

void setRect(Push& p, IRect r) {
    p.rect[0] = static_cast<float>(r.x0);
    p.rect[1] = static_cast<float>(r.y0);
    p.rect[2] = static_cast<float>(r.x1 - r.x0);
    p.rect[3] = static_cast<float>(r.y1 - r.y0);
}

void setColor(Push& p, Color c) {
    p.color[0] = c.r;
    p.color[1] = c.g;
    p.color[2] = c.b;
    p.color[3] = c.a;
}

}  // namespace

// ===========================================================================
// Impl
// ===========================================================================

struct VulkanRenderer::Impl {
    // ---- device ------------------------------------------------------------

    VulkanRenderer::Options options;
    VkInstance instance = VK_NULL_HANDLE;
    VkPhysicalDevice physical = VK_NULL_HANDLE;
    VkPhysicalDeviceMemoryProperties mem_props{};
    uint32_t queue_family = 0;
    VkDevice device = VK_NULL_HANDLE;
    VkQueue queue = VK_NULL_HANDLE;
    VkCommandPool cmd_pool = VK_NULL_HANDLE;
    VkCommandBuffer cmd = VK_NULL_HANDLE;
    VkFence fence = VK_NULL_HANDLE;

    // ---- resources ---------------------------------------------------------

    struct Buffer {
        VkBuffer buffer = VK_NULL_HANDLE;
        VkDeviceMemory memory = VK_NULL_HANDLE;
        void* mapped = nullptr;
        VkDeviceSize size = 0;
    };

    struct Image {
        VkImage image = VK_NULL_HANDLE;
        VkImageView view = VK_NULL_HANDLE;
        VkDeviceMemory memory = VK_NULL_HANDLE;
        int w = 0, h = 0;
        VkFormat format = VK_FORMAT_UNDEFINED;
        VkImageLayout layout = VK_IMAGE_LAYOUT_UNDEFINED;
        bool in_use = false;      // transient-pool bookkeeping
        bool needs_clear = true;  // first beginTarget clears
    };

    // unique_ptr: acquireTransient hands out raw pointers that must stay
    // stable while the pool grows.
    std::vector<std::unique_ptr<Image>> transient_pool;
    Image canvas;    // premultiplied RGBA16F working target
    Image out_rgba;  // RGBA8 conversion target
    Buffer readback;
    Buffer staging;  // per-frame upload arena (host visible)
    VkDeviceSize staging_used = 0;
    Buffer points_ssbo;  // polygon points, rewritten per frame
    Buffer dummy_ssbo;   // bound when a draw needs no points

    struct ContentTex {
        Image image;
        uint64_t frame_used = 0;
    };
    std::map<const Layer*, ContentTex> image_cache;
    Image atlas_tex;
    uint64_t atlas_uploaded_revision = 0;
    uint64_t frame_counter = 0;

    detail::TextAtlas atlas;
    bool atlas_tried = false;

    std::vector<uint8_t> out_pixels;
    Surface surface;

    // ---- samplers / descriptors / pipelines --------------------------------

    VkSampler sampler_nearest = VK_NULL_HANDLE;        // filters, masks (clamp edge)
    VkSampler sampler_linear_edge = VK_NULL_HANDLE;    // images, atlas
    VkSampler sampler_linear_border = VK_NULL_HANDLE;  // composite (transparent border)

    VkDescriptorSetLayout set_layout_tex = VK_NULL_HANDLE;   // binding 0: sampler2D
    VkDescriptorSetLayout set_layout_ssbo = VK_NULL_HANDLE;  // binding 0: SSBO
    VkPipelineLayout layout_tex = VK_NULL_HANDLE;
    VkPipelineLayout layout_ssbo = VK_NULL_HANDLE;
    VkDescriptorPool desc_pool = VK_NULL_HANDLE;

    // Pipelines (built once; index [blend] where applicable).
    VkPipeline pipe_mask_shape = VK_NULL_HANDLE;   // R16F, MAX
    VkPipeline pipe_mask_glyph = VK_NULL_HANDLE;   // R16F, MAX
    VkPipeline pipe_mask_comp[4] = {};             // RGBA16F, blend modes
    VkPipeline pipe_shape_color[4] = {};
    VkPipeline pipe_image[4] = {};
    VkPipeline pipe_composite[4] = {};
    VkPipeline pipe_filter = VK_NULL_HANDLE;
    VkPipeline pipe_blur = VK_NULL_HANDLE;
    VkPipeline pipe_unpremul = VK_NULL_HANDLE;  // RGBA8

    // ---- recording state ---------------------------------------------------

    Image* bound_target = nullptr;

    // Per-frame text runs (glyph quads warmed in the pre-pass).
    struct TextRun {
        std::vector<detail::TextAtlas::GlyphQuad> quads;
        float width = 0;
    };
    std::map<std::string, TextRun> text_runs;  // keyed by utf8 (atlas-size quads)
    std::map<const Layer*, uint32_t> polygon_offsets;
    std::vector<Vec2> polygon_points;

    ~Impl() { destroy(); }

    // =======================================================================
    // init
    // =======================================================================

    void init() {
        createInstanceAndDevice();
        createFixedResources();
        createPipelines();
    }

    void createInstanceAndDevice() {
        VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
        app.pApplicationName = "fluent_stage";
        app.apiVersion = VK_API_VERSION_1_3;
        VkInstanceCreateInfo ii{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
        ii.pApplicationInfo = &app;
        vkCheck(vkCreateInstance(&ii, nullptr, &instance), "vkCreateInstance");

        uint32_t count = 0;
        vkEnumeratePhysicalDevices(instance, &count, nullptr);
        if (count == 0) {
            throw std::runtime_error("fluent_stage vulkan: no devices");
        }
        std::vector<VkPhysicalDevice> devices(count);
        vkEnumeratePhysicalDevices(instance, &count, devices.data());
        physical = devices[0];
        for (VkPhysicalDevice d : devices) {
            VkPhysicalDeviceProperties props;
            vkGetPhysicalDeviceProperties(d, &props);
            if (props.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU ||
                props.deviceType == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU) {
                physical = d;
                break;
            }
        }
        vkGetPhysicalDeviceMemoryProperties(physical, &mem_props);

        uint32_t qcount = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(physical, &qcount, nullptr);
        std::vector<VkQueueFamilyProperties> families(qcount);
        vkGetPhysicalDeviceQueueFamilyProperties(physical, &qcount, families.data());
        bool found = false;
        for (uint32_t i = 0; i < qcount; ++i) {
            if (families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) {
                queue_family = i;
                found = true;
                break;
            }
        }
        if (!found) {
            throw std::runtime_error("fluent_stage vulkan: no graphics queue");
        }

        const float priority = 1.0f;
        VkDeviceQueueCreateInfo qi{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
        qi.queueFamilyIndex = queue_family;
        qi.queueCount = 1;
        qi.pQueuePriorities = &priority;
        VkPhysicalDeviceVulkan13Features f13{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES};
        f13.dynamicRendering = VK_TRUE;
        VkDeviceCreateInfo di{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
        di.pNext = &f13;
        di.queueCreateInfoCount = 1;
        di.pQueueCreateInfos = &qi;
        vkCheck(vkCreateDevice(physical, &di, nullptr, &device), "vkCreateDevice");
        vkGetDeviceQueue(device, queue_family, 0, &queue);

        VkCommandPoolCreateInfo pi{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
        pi.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        pi.queueFamilyIndex = queue_family;
        vkCheck(vkCreateCommandPool(device, &pi, nullptr, &cmd_pool), "vkCreateCommandPool");
        VkCommandBufferAllocateInfo ai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        ai.commandPool = cmd_pool;
        ai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        ai.commandBufferCount = 1;
        vkCheck(vkAllocateCommandBuffers(device, &ai, &cmd), "vkAllocateCommandBuffers");
        VkFenceCreateInfo fi{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
        vkCheck(vkCreateFence(device, &fi, nullptr, &fence), "vkCreateFence");
    }

    uint32_t memoryType(uint32_t bits, VkMemoryPropertyFlags flags) {
        for (uint32_t i = 0; i < mem_props.memoryTypeCount; ++i) {
            if ((bits & (1u << i)) &&
                (mem_props.memoryTypes[i].propertyFlags & flags) == flags) {
                return i;
            }
        }
        throw std::runtime_error("fluent_stage vulkan: no suitable memory type");
    }

    Buffer createBuffer(VkDeviceSize size, VkBufferUsageFlags usage, bool host_visible) {
        Buffer b;
        b.size = size;
        VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bi.size = size;
        bi.usage = usage;
        vkCheck(vkCreateBuffer(device, &bi, nullptr, &b.buffer), "vkCreateBuffer");
        VkMemoryRequirements req;
        vkGetBufferMemoryRequirements(device, b.buffer, &req);
        VkMemoryAllocateInfo mi{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        mi.allocationSize = req.size;
        mi.memoryTypeIndex = memoryType(
            req.memoryTypeBits,
            host_visible ? VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
                         : VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        vkCheck(vkAllocateMemory(device, &mi, nullptr, &b.memory), "vkAllocateMemory");
        vkCheck(vkBindBufferMemory(device, b.buffer, b.memory, 0), "vkBindBufferMemory");
        if (host_visible) {
            vkCheck(vkMapMemory(device, b.memory, 0, VK_WHOLE_SIZE, 0, &b.mapped), "vkMapMemory");
        }
        return b;
    }

    void destroyBuffer(Buffer& b) {
        if (b.buffer != VK_NULL_HANDLE) {
            vkDestroyBuffer(device, b.buffer, nullptr);
        }
        if (b.memory != VK_NULL_HANDLE) {
            vkFreeMemory(device, b.memory, nullptr);
        }
        b = {};
    }

    Image createImage(int w, int h, VkFormat format, VkImageUsageFlags usage) {
        Image img;
        img.w = w;
        img.h = h;
        img.format = format;
        VkImageCreateInfo ii{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
        ii.imageType = VK_IMAGE_TYPE_2D;
        ii.format = format;
        ii.extent = {static_cast<uint32_t>(w), static_cast<uint32_t>(h), 1};
        ii.mipLevels = 1;
        ii.arrayLayers = 1;
        ii.samples = VK_SAMPLE_COUNT_1_BIT;
        ii.tiling = VK_IMAGE_TILING_OPTIMAL;
        ii.usage = usage;
        ii.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        vkCheck(vkCreateImage(device, &ii, nullptr, &img.image), "vkCreateImage");
        VkMemoryRequirements req;
        vkGetImageMemoryRequirements(device, img.image, &req);
        VkMemoryAllocateInfo mi{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        mi.allocationSize = req.size;
        mi.memoryTypeIndex = memoryType(req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        vkCheck(vkAllocateMemory(device, &mi, nullptr, &img.memory), "vkAllocateMemory(image)");
        vkCheck(vkBindImageMemory(device, img.image, img.memory, 0), "vkBindImageMemory");
        VkImageViewCreateInfo vi{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        vi.image = img.image;
        vi.viewType = VK_IMAGE_VIEW_TYPE_2D;
        vi.format = format;
        vi.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        vkCheck(vkCreateImageView(device, &vi, nullptr, &img.view), "vkCreateImageView");
        return img;
    }

    void destroyImage(Image& img) {
        if (img.view != VK_NULL_HANDLE) {
            vkDestroyImageView(device, img.view, nullptr);
        }
        if (img.image != VK_NULL_HANDLE) {
            vkDestroyImage(device, img.image, nullptr);
        }
        if (img.memory != VK_NULL_HANDLE) {
            vkFreeMemory(device, img.memory, nullptr);
        }
        img = {};
    }

    void createFixedResources() {
        const auto makeSampler = [&](VkFilter filter, VkSamplerAddressMode mode,
                                     VkBorderColor border) {
            VkSamplerCreateInfo si{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
            si.magFilter = filter;
            si.minFilter = filter;
            si.addressModeU = mode;
            si.addressModeV = mode;
            si.addressModeW = mode;
            si.borderColor = border;
            VkSampler s;
            vkCheck(vkCreateSampler(device, &si, nullptr, &s), "vkCreateSampler");
            return s;
        };
        sampler_nearest = makeSampler(VK_FILTER_NEAREST, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
                                      VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK);
        sampler_linear_edge = makeSampler(VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
                                          VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK);
        sampler_linear_border =
            makeSampler(VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER,
                        VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK);

        const auto makeSetLayout = [&](VkDescriptorType type) {
            VkDescriptorSetLayoutBinding binding{};
            binding.binding = 0;
            binding.descriptorType = type;
            binding.descriptorCount = 1;
            binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
            VkDescriptorSetLayoutCreateInfo li{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
            li.bindingCount = 1;
            li.pBindings = &binding;
            VkDescriptorSetLayout layout;
            vkCheck(vkCreateDescriptorSetLayout(device, &li, nullptr, &layout),
                    "vkCreateDescriptorSetLayout");
            return layout;
        };
        set_layout_tex = makeSetLayout(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        set_layout_ssbo = makeSetLayout(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);

        const auto makePipeLayout = [&](VkDescriptorSetLayout set) {
            VkPushConstantRange range{VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0,
                                      sizeof(Push)};
            VkPipelineLayoutCreateInfo li{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
            li.setLayoutCount = set == VK_NULL_HANDLE ? 0 : 1;
            li.pSetLayouts = &set;
            li.pushConstantRangeCount = 1;
            li.pPushConstantRanges = &range;
            VkPipelineLayout layout;
            vkCheck(vkCreatePipelineLayout(device, &li, nullptr, &layout),
                    "vkCreatePipelineLayout");
            return layout;
        };
        layout_tex = makePipeLayout(set_layout_tex);
        layout_ssbo = makePipeLayout(set_layout_ssbo);

        VkDescriptorPoolSize sizes[2] = {
            {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 8192},
            {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 512},
        };
        VkDescriptorPoolCreateInfo dpi{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        dpi.maxSets = 8704;
        dpi.poolSizeCount = 2;
        dpi.pPoolSizes = sizes;
        vkCheck(vkCreateDescriptorPool(device, &dpi, nullptr, &desc_pool),
                "vkCreateDescriptorPool");

        staging = createBuffer(16u << 20, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, true);
        points_ssbo = createBuffer(sizeof(float) * 2 * 65536,
                                   VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, true);
        dummy_ssbo = createBuffer(64, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, true);
    }

    VkShaderModule makeModule(const uint32_t* code, size_t bytes) {
        VkShaderModuleCreateInfo si{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
        si.codeSize = bytes;
        si.pCode = code;
        VkShaderModule module;
        vkCheck(vkCreateShaderModule(device, &si, nullptr, &module), "vkCreateShaderModule");
        return module;
    }

    // The four blend modes as fixed-function factors — the same equations
    // the CPU reference documents in blendPremul().
    static VkPipelineColorBlendAttachmentState blendState(int mode) {
        VkPipelineColorBlendAttachmentState s{};
        s.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                           VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        s.blendEnable = VK_TRUE;
        s.alphaBlendOp = VK_BLEND_OP_ADD;
        s.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        s.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        s.colorBlendOp = VK_BLEND_OP_ADD;
        switch (static_cast<Blend>(mode)) {
            case Blend::Normal:
                s.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
                s.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
                break;
            case Blend::Add:
                s.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
                s.dstColorBlendFactor = VK_BLEND_FACTOR_ONE;
                break;
            case Blend::Multiply:
                s.srcColorBlendFactor = VK_BLEND_FACTOR_DST_COLOR;
                s.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
                break;
            case Blend::Screen:
                s.srcColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_DST_COLOR;
                s.dstColorBlendFactor = VK_BLEND_FACTOR_ONE;
                break;
        }
        return s;
    }

    static VkPipelineColorBlendAttachmentState maxBlendState() {
        VkPipelineColorBlendAttachmentState s{};
        s.colorWriteMask = VK_COLOR_COMPONENT_R_BIT;
        s.blendEnable = VK_TRUE;
        s.colorBlendOp = VK_BLEND_OP_MAX;
        s.alphaBlendOp = VK_BLEND_OP_MAX;
        s.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
        s.dstColorBlendFactor = VK_BLEND_FACTOR_ONE;
        s.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        s.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        return s;
    }

    static VkPipelineColorBlendAttachmentState overwriteState() {
        VkPipelineColorBlendAttachmentState s{};
        s.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                           VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        s.blendEnable = VK_FALSE;
        return s;
    }

    VkPipeline makePipeline(VkShaderModule vert, VkShaderModule frag, VkPipelineLayout layout,
                            VkFormat color_format,
                            const VkPipelineColorBlendAttachmentState& blend) {
        VkPipelineShaderStageCreateInfo stages[2] = {};
        stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
        stages[0].module = vert;
        stages[0].pName = "main";
        stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        stages[1].module = frag;
        stages[1].pName = "main";

        VkPipelineVertexInputStateCreateInfo vin{
            VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
        VkPipelineInputAssemblyStateCreateInfo ia{
            VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
        ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP;
        VkPipelineViewportStateCreateInfo vp{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
        vp.viewportCount = 1;
        vp.scissorCount = 1;
        VkPipelineRasterizationStateCreateInfo rs{
            VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
        rs.polygonMode = VK_POLYGON_MODE_FILL;
        rs.cullMode = VK_CULL_MODE_NONE;
        rs.lineWidth = 1.0f;
        VkPipelineMultisampleStateCreateInfo ms{
            VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
        ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
        VkPipelineColorBlendStateCreateInfo cb{
            VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
        cb.attachmentCount = 1;
        cb.pAttachments = &blend;
        VkDynamicState dynamics[2] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
        VkPipelineDynamicStateCreateInfo dyn{
            VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
        dyn.dynamicStateCount = 2;
        dyn.pDynamicStates = dynamics;

        VkPipelineRenderingCreateInfo rendering{VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO};
        rendering.colorAttachmentCount = 1;
        rendering.pColorAttachmentFormats = &color_format;

        VkGraphicsPipelineCreateInfo pi{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
        pi.pNext = &rendering;
        pi.stageCount = 2;
        pi.pStages = stages;
        pi.pVertexInputState = &vin;
        pi.pInputAssemblyState = &ia;
        pi.pViewportState = &vp;
        pi.pRasterizationState = &rs;
        pi.pMultisampleState = &ms;
        pi.pColorBlendState = &cb;
        pi.pDynamicState = &dyn;
        pi.layout = layout;
        VkPipeline pipeline;
        vkCheck(vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pi, nullptr, &pipeline),
                "vkCreateGraphicsPipelines");
        return pipeline;
    }

    void createPipelines() {
        VkShaderModule quad = makeModule(kQuadVert, sizeof kQuadVert);
        VkShaderModule fullscreen = makeModule(kFullscreenVert, sizeof kFullscreenVert);
        VkShaderModule shape_mask = makeModule(kShapeMaskFrag, sizeof kShapeMaskFrag);
        VkShaderModule glyph_mask = makeModule(kGlyphMaskFrag, sizeof kGlyphMaskFrag);
        VkShaderModule mask_comp = makeModule(kMaskCompositeFrag, sizeof kMaskCompositeFrag);
        VkShaderModule shape_color = makeModule(kShapeColorFrag, sizeof kShapeColorFrag);
        VkShaderModule image = makeModule(kImageFrag, sizeof kImageFrag);
        VkShaderModule filter = makeModule(kFilterFrag, sizeof kFilterFrag);
        VkShaderModule blur = makeModule(kBlurFrag, sizeof kBlurFrag);
        VkShaderModule composite = makeModule(kCompositeFrag, sizeof kCompositeFrag);
        VkShaderModule unpremul = makeModule(kUnpremulFrag, sizeof kUnpremulFrag);

        const VkFormat kColor = VK_FORMAT_R32G32B32A32_SFLOAT;
        const VkFormat kMask = VK_FORMAT_R32_SFLOAT;

        pipe_mask_shape = makePipeline(quad, shape_mask, layout_ssbo, kMask, maxBlendState());
        pipe_mask_glyph = makePipeline(quad, glyph_mask, layout_tex, kMask, maxBlendState());
        for (int b = 0; b < 4; ++b) {
            pipe_mask_comp[b] = makePipeline(quad, mask_comp, layout_tex, kColor, blendState(b));
            pipe_shape_color[b] =
                makePipeline(quad, shape_color, layout_ssbo, kColor, blendState(b));
            pipe_image[b] = makePipeline(quad, image, layout_tex, kColor, blendState(b));
            pipe_composite[b] = makePipeline(quad, composite, layout_tex, kColor, blendState(b));
        }
        pipe_filter = makePipeline(fullscreen, filter, layout_tex, kColor, overwriteState());
        pipe_blur = makePipeline(fullscreen, blur, layout_tex, kColor, overwriteState());
        pipe_unpremul = makePipeline(fullscreen, unpremul, layout_tex, VK_FORMAT_R8G8B8A8_UNORM,
                                     overwriteState());

        for (VkShaderModule m : {quad, fullscreen, shape_mask, glyph_mask, mask_comp, shape_color,
                                 image, filter, blur, composite, unpremul}) {
            vkDestroyShaderModule(device, m, nullptr);
        }
    }

    void destroy() {
        if (device == VK_NULL_HANDLE) {
            if (instance != VK_NULL_HANDLE) {
                vkDestroyInstance(instance, nullptr);
            }
            return;
        }
        vkDeviceWaitIdle(device);
        for (auto& t : transient_pool) {
            destroyImage(*t);
        }
        for (auto& [_, tex] : image_cache) {
            destroyImage(tex.image);
        }
        destroyImage(atlas_tex);
        destroyImage(canvas);
        destroyImage(out_rgba);
        destroyBuffer(readback);
        destroyBuffer(staging);
        destroyBuffer(points_ssbo);
        destroyBuffer(dummy_ssbo);
        for (VkPipeline p : {pipe_mask_shape, pipe_mask_glyph, pipe_filter, pipe_blur,
                             pipe_unpremul}) {
            if (p) {
                vkDestroyPipeline(device, p, nullptr);
            }
        }
        for (int b = 0; b < 4; ++b) {
            vkDestroyPipeline(device, pipe_mask_comp[b], nullptr);
            vkDestroyPipeline(device, pipe_shape_color[b], nullptr);
            vkDestroyPipeline(device, pipe_image[b], nullptr);
            vkDestroyPipeline(device, pipe_composite[b], nullptr);
        }
        vkDestroyDescriptorPool(device, desc_pool, nullptr);
        vkDestroyPipelineLayout(device, layout_tex, nullptr);
        vkDestroyPipelineLayout(device, layout_ssbo, nullptr);
        vkDestroyDescriptorSetLayout(device, set_layout_tex, nullptr);
        vkDestroyDescriptorSetLayout(device, set_layout_ssbo, nullptr);
        vkDestroySampler(device, sampler_nearest, nullptr);
        vkDestroySampler(device, sampler_linear_edge, nullptr);
        vkDestroySampler(device, sampler_linear_border, nullptr);
        vkDestroyFence(device, fence, nullptr);
        vkDestroyCommandPool(device, cmd_pool, nullptr);
        vkDestroyDevice(device, nullptr);
        vkDestroyInstance(instance, nullptr);
    }

    // =======================================================================
    // recording helpers
    // =======================================================================

    void barrier(Image& img, VkImageLayout new_layout, VkPipelineStageFlags src_stage,
                 VkAccessFlags src_access, VkPipelineStageFlags dst_stage,
                 VkAccessFlags dst_access) {
        VkImageMemoryBarrier b{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
        b.srcAccessMask = src_access;
        b.dstAccessMask = dst_access;
        b.oldLayout = img.layout;
        b.newLayout = new_layout;
        b.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.image = img.image;
        b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        vkCmdPipelineBarrier(cmd, src_stage, dst_stage, 0, 0, nullptr, 0, nullptr, 1, &b);
        img.layout = new_layout;
    }

    // Suspend/resume dynamic rendering so offscreen subtrees can render
    // mid-parent (the CPU recursion order, expressed in passes).
    void endTarget() {
        if (bound_target != nullptr) {
            vkCmdEndRendering(cmd);
            bound_target = nullptr;
        }
    }

    void ensureTarget(Image& img) {
        if (bound_target == &img) {
            return;
        }
        endTarget();
        if (img.layout != VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL) {
            barrier(img, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                    VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT,
                    VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_TRANSFER_READ_BIT,
                    VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                    VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_COLOR_ATTACHMENT_READ_BIT);
        }
        VkRenderingAttachmentInfo color{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
        color.imageView = img.view;
        color.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        color.loadOp = img.needs_clear ? VK_ATTACHMENT_LOAD_OP_CLEAR : VK_ATTACHMENT_LOAD_OP_LOAD;
        color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        color.clearValue.color = {{0, 0, 0, 0}};
        VkRenderingInfo ri{VK_STRUCTURE_TYPE_RENDERING_INFO};
        ri.renderArea = {{0, 0}, {static_cast<uint32_t>(img.w), static_cast<uint32_t>(img.h)}};
        ri.layerCount = 1;
        ri.colorAttachmentCount = 1;
        ri.pColorAttachments = &color;
        vkCmdBeginRendering(cmd, &ri);
        img.needs_clear = false;
        bound_target = &img;

        VkViewport viewport{0, 0, static_cast<float>(img.w), static_cast<float>(img.h), 0, 1};
        VkRect2D scissor{{0, 0}, {static_cast<uint32_t>(img.w), static_cast<uint32_t>(img.h)}};
        vkCmdSetViewport(cmd, 0, 1, &viewport);
        vkCmdSetScissor(cmd, 0, 1, &scissor);
    }

    // Make an image sampleable (must not be the bound target).
    void toSampled(Image& img) {
        if (img.layout != VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL) {
            barrier(img, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                    VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT,
                    VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_TRANSFER_WRITE_BIT,
                    VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_ACCESS_SHADER_READ_BIT);
        }
    }

    VkDescriptorSet allocSet(VkDescriptorSetLayout layout) {
        VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        ai.descriptorPool = desc_pool;
        ai.descriptorSetCount = 1;
        ai.pSetLayouts = &layout;
        VkDescriptorSet set;
        vkCheck(vkAllocateDescriptorSets(device, &ai, &set), "vkAllocateDescriptorSets");
        return set;
    }

    VkDescriptorSet texSet(VkImageView view, VkSampler sampler) {
        VkDescriptorSet set = allocSet(set_layout_tex);
        VkDescriptorImageInfo info{sampler, view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
        VkWriteDescriptorSet w{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        w.dstSet = set;
        w.dstBinding = 0;
        w.descriptorCount = 1;
        w.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        w.pImageInfo = &info;
        vkUpdateDescriptorSets(device, 1, &w, 0, nullptr);
        return set;
    }

    VkDescriptorSet ssboSet(VkBuffer buffer) {
        VkDescriptorSet set = allocSet(set_layout_ssbo);
        VkDescriptorBufferInfo info{buffer, 0, VK_WHOLE_SIZE};
        VkWriteDescriptorSet w{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        w.dstSet = set;
        w.dstBinding = 0;
        w.descriptorCount = 1;
        w.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        w.pBufferInfo = &info;
        vkUpdateDescriptorSets(device, 1, &w, 0, nullptr);
        return set;
    }

    void drawQuad(VkPipeline pipeline, VkPipelineLayout layout, VkDescriptorSet set,
                  const Push& push) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
        if (set != VK_NULL_HANDLE) {
            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, 1, &set, 0,
                                    nullptr);
        }
        vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                           0, sizeof(Push), &push);
        vkCmdDraw(cmd, 4, 1, 0, 0);
    }

    void drawFullscreen(VkPipeline pipeline, VkDescriptorSet set, const Push& push) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout_tex, 0, 1, &set, 0,
                                nullptr);
        vkCmdPushConstants(cmd, layout_tex,
                           VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0,
                           sizeof(Push), &push);
        vkCmdDraw(cmd, 3, 1, 0, 0);
    }

    // ---- transient pool ----------------------------------------------------

    Image* acquireTransient(int w, int h, VkFormat format) {
        for (auto& t : transient_pool) {
            if (!t->in_use && t->w == w && t->h == h && t->format == format) {
                t->in_use = true;
                t->needs_clear = true;
                return t.get();
            }
        }
        transient_pool.push_back(std::make_unique<Image>(
            createImage(w, h, format,
                        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT)));
        transient_pool.back()->in_use = true;
        transient_pool.back()->needs_clear = true;
        return transient_pool.back().get();
    }

    void releaseAllTransients() {
        for (auto& t : transient_pool) {
            t->in_use = false;
        }
    }

    // ---- uploads (recorded before any rendering) ---------------------------

    VkDeviceSize stagePush(const void* data, VkDeviceSize bytes) {
        staging_used = (staging_used + 15) & ~VkDeviceSize(15);
        if (staging_used + bytes > staging.size) {
            // Grow between frames: submit-safe because uploads happen
            // before rendering; a bigger arena next frame.
            throw std::runtime_error("fluent_stage vulkan: staging arena exhausted");
        }
        std::memcpy(static_cast<uint8_t*>(staging.mapped) + staging_used, data, bytes);
        const VkDeviceSize offset = staging_used;
        staging_used += bytes;
        return offset;
    }

    void uploadImage(Image& img, const void* pixels, VkDeviceSize bytes) {
        const VkDeviceSize offset = stagePush(pixels, bytes);
        barrier(img, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                VK_ACCESS_SHADER_READ_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
                VK_ACCESS_TRANSFER_WRITE_BIT);
        VkBufferImageCopy copy{};
        copy.bufferOffset = offset;
        copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
        copy.imageExtent = {static_cast<uint32_t>(img.w), static_cast<uint32_t>(img.h), 1};
        vkCmdCopyBufferToImage(cmd, staging.buffer, img.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                               1, &copy);
        barrier(img, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_TRANSFER_BIT,
                VK_ACCESS_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                VK_ACCESS_SHADER_READ_BIT);
    }

    // =======================================================================
    // text
    // =======================================================================

    void ensureAtlas() {
        if (atlas.ready() || atlas_tried) {
            return;
        }
        atlas_tried = true;
        detail::initAtlasWithFallback(atlas, options.font_file, options.font_pixel_size);
    }

    Rect measureText(const TextContent& c) {
        ensureAtlas();
        if (!atlas.ready()) {
            return {};
        }
        const float s = c.size / static_cast<float>(atlas.pixelSize());
        const float w = atlas.measure(c.utf8) * s;
        const float h = atlas.lineHeight() * s;
        float x = c.position.x;
        if (c.align == Align::Center) {
            x -= w * 0.5f;
        } else if (c.align == Align::Right) {
            x -= w;
        }
        return {x, c.position.y, w, h};
    }

    const TextRun& textRun(const std::string& utf8) {
        auto it = text_runs.find(utf8);
        if (it == text_runs.end()) {
            TextRun run;
            atlas.layout(utf8, 0, 0, 4096, run.quads);
            run.width = atlas.measure(utf8);
            it = text_runs.emplace(utf8, std::move(run)).first;
        }
        return it->second;
    }

    static std::string boxLabel(const Box& box) {
        std::string label = box.label;
        if (box.score > 0) {
            char score[16];
            std::snprintf(score, sizeof score, "%.2f", box.score);
            label += (label.empty() ? "" : " ") + std::string(score);
        }
        return label;
    }

    // Pre-pass: warm every glyph this frame will draw (so the one atlas
    // upload below already contains them), gather polygon points, and
    // upload image content.
    void prepassLayer(const Layer& layer) {
        if (layer.hidden()) {
            return;
        }
        const Content& content = layer.content();
        if (const auto* t = std::get_if<TextContent>(&content)) {
            ensureAtlas();
            if (atlas.ready()) {
                textRun(t->utf8);
            }
        } else if (const auto* b = std::get_if<BoxesContent>(&content)) {
            if (b->show_label) {
                ensureAtlas();
                if (atlas.ready()) {
                    for (const Box& box : b->smoothed) {
                        const std::string label = boxLabel(box);
                        if (!label.empty()) {
                            textRun(label);
                        }
                    }
                }
            }
        } else if (const auto* p = std::get_if<PolygonContent>(&content)) {
            if (layer.thickness() <= 0 && p->points.size() >= 3) {
                polygon_offsets[&layer] = static_cast<uint32_t>(polygon_points.size());
                polygon_points.insert(polygon_points.end(), p->points.begin(), p->points.end());
            }
        } else if (const auto* img = std::get_if<ImageContent>(&content)) {
            if (img->view.valid()) {
                uploadContentImage(layer, img->view);
            }
        }
        for (const auto& child : layer.sublayers()) {
            prepassLayer(*child);
        }
    }

    void uploadContentImage(const Layer& layer, const ImageView& view) {
        auto& entry = image_cache[&layer];
        if (entry.image.w != static_cast<int>(view.width) ||
            entry.image.h != static_cast<int>(view.height)) {
            if (entry.image.image != VK_NULL_HANDLE) {
                vkDeviceWaitIdle(device);
                destroyImage(entry.image);
            }
            entry.image = createImage(static_cast<int>(view.width), static_cast<int>(view.height),
                                      VK_FORMAT_R8G8B8A8_UNORM,
                                      VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT);
        }
        entry.frame_used = frame_counter;
        // Premultiply during staging so bilinear filtering matches the CPU
        // reference (which premultiplies per tap before interpolating).
        const uint32_t stride = view.stride();
        std::vector<uint8_t> premul(static_cast<size_t>(view.width) * view.height * 4);
        for (uint32_t y = 0; y < view.height; ++y) {
            const uint8_t* src = view.pixels + static_cast<size_t>(y) * stride;
            uint8_t* dst = premul.data() + static_cast<size_t>(y) * view.width * 4;
            for (uint32_t x = 0; x < view.width; ++x) {
                const uint8_t a = src[x * 4 + 3];
                dst[x * 4 + 0] = static_cast<uint8_t>(src[x * 4 + 0] * a / 255);
                dst[x * 4 + 1] = static_cast<uint8_t>(src[x * 4 + 1] * a / 255);
                dst[x * 4 + 2] = static_cast<uint8_t>(src[x * 4 + 2] * a / 255);
                dst[x * 4 + 3] = a;
            }
        }
        uploadImage(entry.image, premul.data(), premul.size());
    }

    void uploadAtlasIfNeeded() {
        if (!atlas.ready() || atlas.revision() == atlas_uploaded_revision) {
            return;
        }
        if (atlas_tex.w != static_cast<int>(atlas.width()) ||
            atlas_tex.h != static_cast<int>(atlas.height())) {
            if (atlas_tex.image != VK_NULL_HANDLE) {
                vkDeviceWaitIdle(device);
                destroyImage(atlas_tex);
            }
            atlas_tex = createImage(static_cast<int>(atlas.width()),
                                    static_cast<int>(atlas.height()), VK_FORMAT_R8_UNORM,
                                    VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT);
        }
        uploadImage(atlas_tex, atlas.pixels(),
                    static_cast<VkDeviceSize>(atlas.width()) * atlas.height());
        atlas_uploaded_revision = atlas.revision();
    }

    // =======================================================================
    // drawing — mirrors cpu_renderer.cpp drawContent structure
    // =======================================================================

    static int blendIndex(Blend b) { return static_cast<int>(b); }

    struct MaskCtx {
        Image* mask = nullptr;
        IRect box;                // mask region in target coords
        Mat23 inv_for_mask;       // mask px → local units
        float aa = 0;
    };

    // Opens a coverage mask covering `local_bbox` under `m` (mirrors the
    // CPU rasterizer's Mask::reset + begin()).
    bool beginMask(MaskCtx& ctx, Image& target, const Mat23& m, const Mat23& inv, Rect local_bbox,
                   float pad_px, float aa) {
        ctx.box = targetBBox(m, local_bbox, pad_px, target.w, target.h);
        if (ctx.box.empty()) {
            return false;
        }
        endTarget();
        ctx.mask = acquireTransient(ctx.box.x1 - ctx.box.x0, ctx.box.y1 - ctx.box.y0,
                                    VK_FORMAT_R32_SFLOAT);
        // Mask fragment coords are mask-local; fold the origin shift into
        // the inverse transform: local = inv(mask_px + box_origin).
        ctx.inv_for_mask = inv * Mat23::translation({static_cast<float>(ctx.box.x0),
                                                     static_cast<float>(ctx.box.y0)});
        ctx.aa = aa;
        ensureTarget(*ctx.mask);
        return true;
    }

    // One shape part into the mask (quad clipped to the part's own bbox,
    // exactly like the CPU rasterizePart clip).
    void maskPart(MaskCtx& ctx, const Mat23& m, Rect part_bbox, float pad_px, int mode,
                  const float pa[4], const float pb[4]) {
        IRect r = targetBBox(m, part_bbox, pad_px, ctx.box.x1, ctx.box.y1);
        r.x0 = std::max(r.x0, ctx.box.x0);
        r.y0 = std::max(r.y0, ctx.box.y0);
        if (r.empty()) {
            return;
        }
        Push push{};
        push.rect[0] = static_cast<float>(r.x0 - ctx.box.x0);
        push.rect[1] = static_cast<float>(r.y0 - ctx.box.y0);
        push.rect[2] = static_cast<float>(r.x1 - r.x0);
        push.rect[3] = static_cast<float>(r.y1 - r.y0);
        push.meta[0] = static_cast<float>(ctx.mask->w);
        push.meta[1] = static_cast<float>(ctx.mask->h);
        push.meta[2] = ctx.aa;
        push.meta[3] = static_cast<float>(mode);
        setInv(push, ctx.inv_for_mask);
        std::memcpy(push.pa, pa, sizeof push.pa);
        std::memcpy(push.pb, pb, sizeof push.pb);
        drawQuad(pipe_mask_shape, layout_ssbo,
                 ssboSet(polygon_points.empty() ? dummy_ssbo.buffer : points_ssbo.buffer), push);
    }

    void maskGlyph(MaskCtx& ctx, const Mat23& m, Rect glyph_local, const float uv[4]) {
        IRect r = targetBBox(m, glyph_local, 1, ctx.box.x1, ctx.box.y1);
        r.x0 = std::max(r.x0, ctx.box.x0);
        r.y0 = std::max(r.y0, ctx.box.y0);
        if (r.empty()) {
            return;
        }
        Push push{};
        push.rect[0] = static_cast<float>(r.x0 - ctx.box.x0);
        push.rect[1] = static_cast<float>(r.y0 - ctx.box.y0);
        push.rect[2] = static_cast<float>(r.x1 - r.x0);
        push.rect[3] = static_cast<float>(r.y1 - r.y0);
        push.meta[0] = static_cast<float>(ctx.mask->w);
        push.meta[1] = static_cast<float>(ctx.mask->h);
        push.meta[2] = ctx.aa;
        setInv(push, ctx.inv_for_mask);
        push.pa[0] = glyph_local.x;
        push.pa[1] = glyph_local.y;
        push.pa[2] = glyph_local.w;
        push.pa[3] = glyph_local.h;
        std::memcpy(push.pb, uv, sizeof push.pb);
        drawQuad(pipe_mask_glyph, layout_tex, texSet(atlas_tex.view, sampler_linear_edge), push);
    }

    // Composites the finished mask into the target with the layer color.
    void finishMask(MaskCtx& ctx, Image& target, Color color, float fold_alpha, Blend blend) {
        endTarget();
        toSampled(*ctx.mask);
        ensureTarget(target);
        Push push{};
        setRect(push, ctx.box);
        push.meta[0] = static_cast<float>(target.w);
        push.meta[1] = static_cast<float>(target.h);
        push.pa[0] = fold_alpha;
        setColor(push, color);
        drawQuad(pipe_mask_comp[blendIndex(blend)], layout_tex,
                 texSet(ctx.mask->view, sampler_nearest), push);
    }

    void drawRoundedRect(Image& target, Rect r, float corner, float stroke_w, const Mat23& m,
                         Color color, float alpha, Blend blend) {
        if (r.w <= 0 || r.h <= 0 || color.a <= 0 || alpha <= 0) {
            return;
        }
        const float scale = scaleOf(m);
        if (scale <= 0) {
            return;
        }
        const IRect box = targetBBox(m, r, (stroke_w + 2) * scale, target.w, target.h);
        if (box.empty()) {
            return;
        }
        ensureTarget(target);
        Push push{};
        setRect(push, box);
        push.meta[0] = static_cast<float>(target.w);
        push.meta[1] = static_cast<float>(target.h);
        push.meta[2] = 1.0f / scale;
        setInv(push, m.inverse());
        const Vec2 center = r.center();
        push.pa[0] = center.x;
        push.pa[1] = center.y;
        push.pa[2] = r.w * 0.5f;
        push.pa[3] = r.h * 0.5f;
        push.pb[0] = std::min(corner, std::min(r.w, r.h) * 0.5f);
        push.pb[1] = stroke_w;
        push.px2[0] = alpha;
        setColor(push, color);
        drawQuad(pipe_shape_color[blendIndex(blend)], layout_ssbo, ssboSet(dummy_ssbo.buffer),
                 push);
    }

    void drawTextRun(Image& target, const TextContent& c, const Mat23& m, const Mat23& inv,
                     Color color, float alpha, Blend blend, float scale) {
        ensureAtlas();
        if (!atlas.ready() || c.utf8.empty() || atlas_tex.image == VK_NULL_HANDLE) {
            return;
        }
        const TextRun& run = textRun(c.utf8);
        const float s = c.size / static_cast<float>(atlas.pixelSize());
        float align_dx = 0;
        if (c.align != Align::Left) {
            const float w = run.width * s;
            align_dx = c.align == Align::Center ? -w * 0.5f : -w;
        }
        const Rect text_box = measureText(c);
        MaskCtx mask;
        if (!beginMask(mask, target, m, inv, rectPad(text_box, 2), 2 * scale, 1.0f / scale)) {
            return;
        }
        for (const auto& q : run.quads) {
            const Rect lq{c.position.x + align_dx + q.x * s, c.position.y + q.y * s, q.w * s,
                          q.h * s};
            const float uv[4] = {q.u0, q.v0, q.u1, q.v1};
            maskGlyph(mask, m, lq, uv);
        }
        finishMask(mask, target, color, alpha, blend);
    }

    void drawImageContent(Image& target, const Layer& layer, const ImageContent& c, Rect bounds,
                          const Mat23& m, const Mat23& inv, float alpha, Blend blend) {
        auto it = image_cache.find(&layer);
        if (it == image_cache.end() || !c.view.valid() || bounds.w <= 0 || bounds.h <= 0) {
            return;
        }
        Image& tex = it->second.image;
        Rect src{0, 0, static_cast<float>(c.view.width), static_cast<float>(c.view.height)};
        if (c.source_rect.w > 0 && c.source_rect.h > 0) {
            src = plan::rectIntersect(c.source_rect, src);
        }
        if (src.w <= 0 || src.h <= 0) {
            return;
        }
        Rect dest = bounds;
        if (c.fit == Fit::Contain) {
            const float s = std::min(bounds.w / src.w, bounds.h / src.h);
            dest = {bounds.x + (bounds.w - src.w * s) * 0.5f,
                    bounds.y + (bounds.h - src.h * s) * 0.5f, src.w * s, src.h * s};
        } else if (c.fit == Fit::Cover) {
            const float s = std::max(bounds.w / src.w, bounds.h / src.h);
            const float vis_w = bounds.w / s;
            const float vis_h = bounds.h / s;
            src = {src.x + (src.w - vis_w) * 0.5f, src.y + (src.h - vis_h) * 0.5f, vis_w, vis_h};
        }
        const IRect box = targetBBox(m, dest, 1, target.w, target.h);
        if (box.empty()) {
            return;
        }
        ensureTarget(target);
        Push push{};
        setRect(push, box);
        push.meta[0] = static_cast<float>(target.w);
        push.meta[1] = static_cast<float>(target.h);
        setInv(push, inv);
        push.pa[0] = dest.x;
        push.pa[1] = dest.y;
        push.pa[2] = dest.w;
        push.pa[3] = dest.h;
        push.pb[0] = src.x;
        push.pb[1] = src.y;
        push.pb[2] = src.w;
        push.pb[3] = src.h;
        push.px2[0] = static_cast<float>(tex.w);
        push.px2[1] = static_cast<float>(tex.h);
        push.px2[2] = alpha;
        drawQuad(pipe_image[blendIndex(blend)], layout_tex, texSet(tex.view, sampler_linear_edge),
                 push);
    }

    // Mirrors CpuRenderer::Impl::drawContent case by case.
    void drawContent(Image& target, const Layer& layer, Rect bounds, const Mat23& m,
                     float alpha) {
        const Content& content = layer.content();
        if (std::holds_alternative<std::monostate>(content)) {
            return;
        }
        const Mat23 inv = m.inverse();
        const float scale = scaleOf(m);
        if (scale <= 0) {
            return;
        }
        const float aa = 1.0f / scale;
        const float th = layer.thickness();
        const Color color = layer.colorValue();
        const Blend mode = layer.blendMode();
        const float pad_px = (th * 0.5f + 2.0f) * scale;

        MaskCtx mask;
        const auto begin = [&](Rect local_bbox, float extra_pad_logical) {
            return beginMask(mask, target, m, inv, local_bbox,
                             pad_px + extra_pad_logical * scale, aa);
        };
        const auto finish = [&] { finishMask(mask, target, color, alpha, mode); };

        const auto segPart = [&](const Seg& s, Cap capstyle) {
            float pa[4] = {s.a.x, s.a.y, s.b.x, s.b.y};
            float pb[4] = {capstyle == Cap::Butt ? th * 0.5f : th, 0, 0, 0};
            maskPart(mask, m, segBounds(s), pad_px,
                     capstyle == Cap::Butt ? SM_SEGMENT_BUTT : SM_SEGMENT_ROUND, pa, pb);
        };
        const auto strokeSegs = [&](const std::vector<Seg>& segs, Cap capstyle) {
            for (const Seg& s : segs) {
                segPart(s, capstyle);
            }
        };
        const auto roundedRectPart = [&](Rect r, float corner) {
            const Vec2 center = r.center();
            const float cr = std::min(corner, std::min(r.w, r.h) * 0.5f);
            float pa[4] = {center.x, center.y, r.w * 0.5f, r.h * 0.5f};
            float pb[4] = {cr, th, 0, 0};
            maskPart(mask, m, r, pad_px, SM_ROUNDED_RECT, pa, pb);
        };

        if (const auto* c = std::get_if<ImageContent>(&content)) {
            drawImageContent(target, layer, *c, bounds, m, inv, alpha, mode);
        } else if (const auto* c = std::get_if<TextContent>(&content)) {
            drawTextRun(target, *c, m, inv, color, alpha, mode, scale);
        } else if (const auto* c = std::get_if<LineContent>(&content)) {
            if (begin(segBounds({c->from, c->to}), 0)) {
                strokeSegs(dashSegments({{c->from, c->to}}, layer.dashValue()), layer.capValue());
                finish();
            }
        } else if (const auto* c = std::get_if<PolylineContent>(&content)) {
            if (c->points.size() >= 2 && begin(layer.contentBounds(), 0)) {
                strokeSegs(dashSegments(pathSegments(c->points, false), layer.dashValue()),
                           layer.capValue());
                finish();
            }
        } else if (const auto* c = std::get_if<PolygonContent>(&content)) {
            if (c->points.size() >= 3 && begin(layer.contentBounds(), 0)) {
                if (th > 0) {
                    strokeSegs(dashSegments(pathSegments(c->points, true), layer.dashValue()),
                               layer.capValue());
                } else {
                    auto it = polygon_offsets.find(&layer);
                    if (it != polygon_offsets.end()) {
                        float pa[4] = {static_cast<float>(it->second),
                                       static_cast<float>(c->points.size()), 0, 0};
                        float pb[4] = {0, 0, 0, 0};
                        maskPart(mask, m, layer.contentBounds(), pad_px, SM_POLYGON, pa, pb);
                    }
                }
                finish();
            }
        } else if (const auto* c = std::get_if<RectContent>(&content)) {
            if (begin(c->rect, 0)) {
                roundedRectPart(c->rect, layer.cornerRadius());
                finish();
            }
        } else if (const auto* c = std::get_if<CircleContent>(&content)) {
            if (begin(layer.contentBounds(), 0)) {
                float pa[4] = {c->center.x, c->center.y, c->radius, th};
                float pb[4] = {0, 0, 0, 0};
                maskPart(mask, m, layer.contentBounds(), pad_px, SM_CIRCLE, pa, pb);
                finish();
            }
        } else if (const auto* c = std::get_if<CirclesContent>(&content)) {
            if (!c->centers.empty() && begin(layer.contentBounds(), 0)) {
                for (const Vec2& center : c->centers) {
                    const Rect b{center.x - c->radius, center.y - c->radius, 2 * c->radius,
                                 2 * c->radius};
                    float pa[4] = {center.x, center.y, c->radius, th};
                    float pb[4] = {0, 0, 0, 0};
                    maskPart(mask, m, b, pad_px, SM_CIRCLE, pa, pb);
                }
                finish();
            }
        } else if (const auto* c = std::get_if<ArcContent>(&content)) {
            if (begin(layer.contentBounds(), 0)) {
                constexpr float kPi = 3.14159265358979f;
                float a0 = c->start_deg * kPi / 180.0f;
                float a1 = c->end_deg * kPi / 180.0f;
                if (a1 < a0) {
                    std::swap(a0, a1);
                }
                a1 = std::min(a1, a0 + 2 * kPi);
                float pa[4] = {c->center.x, c->center.y, c->radius, a0};
                float pb[4] = {a1, th, 0, 0};
                maskPart(mask, m, layer.contentBounds(), pad_px, SM_ARC, pa, pb);
                finish();
            }
        } else if (const auto* c = std::get_if<ArrowContent>(&content)) {
            const float len = std::hypot(c->to.x - c->from.x, c->to.y - c->from.y);
            if (len > 1e-4f) {
                const float head = c->head_size > 0 ? c->head_size : std::max(3.0f * th, 9.0f);
                if (begin(rectPad(layer.contentBounds(), head), 0)) {
                    const Vec2 dir{(c->to.x - c->from.x) / len, (c->to.y - c->from.y) / len};
                    const Vec2 neck{c->to.x - dir.x * head, c->to.y - dir.y * head};
                    const Vec2 side{-dir.y * head * 0.5f, dir.x * head * 0.5f};
                    segPart({c->from, neck}, layer.capValue());
                    float pa[4] = {c->to.x, c->to.y, neck.x + side.x, neck.y + side.y};
                    float pb[4] = {neck.x - side.x, neck.y - side.y, 0, 0};
                    maskPart(mask, m, rectPad(segBounds({neck, c->to}), head), pad_px, SM_TRIANGLE,
                             pa, pb);
                    finish();
                }
            }
        } else if (const auto* c = std::get_if<CrosshairContent>(&content)) {
            if (begin(layer.contentBounds(), 0)) {
                const float gap = c->gap > 0 ? c->gap : c->size * 0.25f;
                const Vec2 o = c->center;
                const Seg ticks[4] = {{{o.x + gap, o.y}, {o.x + c->size, o.y}},
                                      {{o.x - gap, o.y}, {o.x - c->size, o.y}},
                                      {{o.x, o.y + gap}, {o.x, o.y + c->size}},
                                      {{o.x, o.y - gap}, {o.x, o.y - c->size}}};
                strokeSegs({ticks[0], ticks[1], ticks[2], ticks[3]}, layer.capValue());
                finish();
            }
        } else if (const auto* c = std::get_if<GridContent>(&content)) {
            if (begin(bounds, 0)) {
                const Vec2 center = bounds.center();
                float pa[4] = {c->spacing, th, 0, 0};
                float pb[4] = {center.x, center.y, bounds.w * 0.5f, bounds.h * 0.5f};
                maskPart(mask, m, bounds, pad_px, SM_GRID, pa, pb);
                finish();
            }
        } else if (const auto* c = std::get_if<BoxesContent>(&content)) {
            if (!c->smoothed.empty() && begin(rectPad(layer.contentBounds(), 20.0f), 0)) {
                for (const Box& box : c->smoothed) {
                    roundedRectPart(box.rect, layer.cornerRadius());
                }
                finish();
                if (c->show_label) {
                    for (const Box& box : c->smoothed) {
                        const std::string label = boxLabel(box);
                        if (label.empty()) {
                            continue;
                        }
                        TextContent t;
                        t.utf8 = label;
                        t.size = 14;
                        t.position = {box.rect.x + 2, std::max(box.rect.y - 18.0f, 0.0f)};
                        drawTextRun(target, t, m, inv, color, alpha, mode, scale);
                    }
                }
            }
        }
    }

    // =======================================================================
    // layer recursion — mirrors cpu_renderer.cpp renderLayer
    // =======================================================================

    void renderLayer(Image& target, const Layer& layer, const Mat23& parent_m, Vec2 parent_size) {
        if (layer.hidden()) {
            return;
        }
        const float opacity = layer.presentedOpacity();
        if (opacity <= 0) {
            return;
        }
        const Layer::Resolved r = layer.resolve(parent_size);
        const Mat23 m = parent_m * r.to_parent;
        const bool needs_offscreen = plan::needsOffscreen(layer, opacity);

        if (!needs_offscreen) {
            const Blend mode = layer.blendMode();
            drawRoundedRect(target, r.bounds, layer.cornerRadius(), 0, m, layer.backgroundValue(),
                            opacity, mode);
            drawContent(target, layer, r.bounds, m, opacity);
            for (const auto& child : layer.sublayers()) {
                renderLayer(target, *child, m, r.bounds.size());
            }
            if (layer.borderValue()) {
                drawRoundedRect(target, r.bounds, layer.cornerRadius(),
                                layer.borderValue()->width, m, layer.borderValue()->color,
                                opacity, mode);
            }
            return;
        }

        // ---- offscreen path ----
        const float scale = scaleOf(m);
        Rect ext = plan::subtreeExtent(layer, parent_size,
                                       [this](const TextContent& c) { return measureText(c); });
        if (ext.w <= 0 || ext.h <= 0 || scale <= 0) {
            return;
        }
        ext = rectPad(ext, 2.0f / scale);
        const int bw = std::max(1, static_cast<int>(std::ceil(ext.w * scale)));
        const int bh = std::max(1, static_cast<int>(std::ceil(ext.h * scale)));
        if (static_cast<int64_t>(bw) * bh > 64LL * 1024 * 1024) {
            return;
        }
        endTarget();
        Image* buf = acquireTransient(bw, bh, VK_FORMAT_R32G32B32A32_SFLOAT);
        ensureTarget(*buf);  // clears
        const Mat23 mb = Mat23::scaling({scale, scale}) * Mat23::translation({-ext.x, -ext.y});

        drawRoundedRect(*buf, r.bounds, layer.cornerRadius(), 0, mb, layer.backgroundValue(),
                        1.0f, Blend::Normal);
        drawContent(*buf, layer, r.bounds, mb, 1.0f);
        for (const auto& child : layer.sublayers()) {
            renderLayer(*buf, *child, mb, r.bounds.size());
        }
        if (layer.borderValue()) {
            drawRoundedRect(*buf, r.bounds, layer.cornerRadius(), layer.borderValue()->width, mb,
                            layer.borderValue()->color, 1.0f, Blend::Normal);
        }

        // Filters: ping-pong through transients (fullscreen passes).
        for (const Filter& f : layer.filters()) {
            float values[5];
            plan::scaleFilterValues(f, scale, ext, static_cast<float>(bw),
                                    static_cast<float>(bh), values);
            if (f.mode == FS_BLUR) {
                buf = gaussianBlur(buf, values[0]);
                continue;
            }
            endTarget();
            toSampled(*buf);
            Image* dst = acquireTransient(bw, bh, VK_FORMAT_R32G32B32A32_SFLOAT);
            ensureTarget(*dst);
            Push push{};
            push.meta[3] = static_cast<float>(f.mode);
            push.pa[0] = values[0];
            push.pa[1] = values[1];
            push.pa[2] = values[2];
            push.pa[3] = values[3];
            push.pb[0] = values[4];
            drawFullscreen(pipe_filter, texSet(buf->view, sampler_nearest), push);
            buf->in_use = false;
            buf = dst;
        }

        endTarget();
        toSampled(*buf);
        ensureTarget(target);

        const Mat23 inv = m.inverse();
        const float aa = 1.0f / scale;
        const IRect box = targetBBox(m, ext, 2, target.w, target.h);

        if (const auto& sh = layer.shadowValue()) {
            Image* shadow_buf = gaussianBlurCopy(buf, sh->radius * scale, bw, bh);
            toSampled(*shadow_buf);
            ensureTarget(target);
            const IRect sbox = targetBBox(
                m, rectPad({ext.x + sh->offset.x, ext.y + sh->offset.y, ext.w, ext.h},
                           sh->radius * 2),
                2, target.w, target.h);
            if (!sbox.empty()) {
                Push push{};
                setRect(push, sbox);
                push.meta[0] = static_cast<float>(target.w);
                push.meta[1] = static_cast<float>(target.h);
                push.meta[2] = aa;
                push.meta[3] = 2.0f;  // tint flag
                setInv(push, inv);
                push.pa[0] = ext.x + sh->offset.x;
                push.pa[1] = ext.y + sh->offset.y;
                push.pa[2] = scale;
                push.pa[3] = sh->opacity * opacity;
                setColor(push, sh->color);
                drawQuad(pipe_composite[blendIndex(Blend::Normal)], layout_tex,
                         texSet(shadow_buf->view, sampler_linear_border), push);
            }
            shadow_buf->in_use = false;
        }

        if (!box.empty()) {
            Push push{};
            setRect(push, box);
            push.meta[0] = static_cast<float>(target.w);
            push.meta[1] = static_cast<float>(target.h);
            push.meta[2] = aa;
            push.meta[3] = layer.masksToBounds() ? 1.0f : 0.0f;
            setInv(push, inv);
            push.pa[0] = ext.x;
            push.pa[1] = ext.y;
            push.pa[2] = scale;
            push.pa[3] = opacity;
            const Vec2 center = r.bounds.center();
            push.pb[0] = center.x;
            push.pb[1] = center.y;
            push.pb[2] = r.bounds.w * 0.5f;
            push.pb[3] = r.bounds.h * 0.5f;
            push.px2[0] =
                std::min(layer.cornerRadius(), std::min(r.bounds.w, r.bounds.h) * 0.5f);
            drawQuad(pipe_composite[blendIndex(layer.blendMode())], layout_tex,
                     texSet(buf->view, sampler_linear_border), push);
        }
        buf->in_use = false;
    }

    // Separable gaussian into fresh transients; consumes `src` (marks it
    // free) and returns the blurred image. Same weights as the CPU.
    Image* gaussianBlur(Image* src, float radius_px) {
        if (radius_px <= 0.01f) {
            return src;
        }
        const float sigma = radius_px * 0.5f;
        const int half = std::max(1, static_cast<int>(std::ceil(sigma * 2.5f)));
        Image* current = src;
        for (int pass = 0; pass < 2; ++pass) {
            endTarget();
            toSampled(*current);
            Image* dst = acquireTransient(src->w, src->h, VK_FORMAT_R32G32B32A32_SFLOAT);
            ensureTarget(*dst);
            Push push{};
            push.pa[0] = pass == 0 ? 1.0f : 0.0f;
            push.pa[1] = pass == 0 ? 0.0f : 1.0f;
            push.pa[2] = sigma;
            push.pa[3] = static_cast<float>(half);
            drawFullscreen(pipe_blur, texSet(current->view, sampler_nearest), push);
            current->in_use = false;
            current = dst;
        }
        return current;
    }

    // Blur without consuming the source (shadows blur a copy's alpha).
    Image* gaussianBlurCopy(Image* src, float radius_px, int w, int h) {
        endTarget();
        toSampled(*src);
        Image* copy = acquireTransient(w, h, VK_FORMAT_R32G32B32A32_SFLOAT);
        ensureTarget(*copy);
        Push push{};
        push.pa[0] = 1.0f;
        push.pa[1] = 0.0f;
        push.pa[2] = std::max(radius_px * 0.5f, 1e-4f);
        push.pa[3] = static_cast<float>(std::max(1, static_cast<int>(std::ceil(radius_px * 0.5f * 2.5f))));
        // First blur pass doubles as the copy.
        drawFullscreen(pipe_blur, texSet(src->view, sampler_nearest), push);
        endTarget();
        toSampled(*copy);
        Image* out = acquireTransient(w, h, VK_FORMAT_R32G32B32A32_SFLOAT);
        ensureTarget(*out);
        push.pa[0] = 0.0f;
        push.pa[1] = 1.0f;
        drawFullscreen(pipe_blur, texSet(copy->view, sampler_nearest), push);
        copy->in_use = false;
        return out;
    }

    // =======================================================================
    // frame
    // =======================================================================

    const Surface& renderFrame(Stage& stage, uint32_t out_w, uint32_t out_h, float dt) {
        stage.setTextMeasurer([this](const TextContent& c) { return measureText(c); });
        stage.advance(dt);
        ++frame_counter;

        // (Re)create the per-size fixed targets.
        if (canvas.w != static_cast<int>(out_w) || canvas.h != static_cast<int>(out_h)) {
            vkDeviceWaitIdle(device);
            destroyImage(canvas);
            destroyImage(out_rgba);
            destroyBuffer(readback);
            canvas = createImage(static_cast<int>(out_w), static_cast<int>(out_h),
                                 VK_FORMAT_R32G32B32A32_SFLOAT,
                                 VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
            out_rgba = createImage(static_cast<int>(out_w), static_cast<int>(out_h),
                                   VK_FORMAT_R8G8B8A8_UNORM,
                                   VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT |
                                       VK_IMAGE_USAGE_TRANSFER_SRC_BIT);
            readback = createBuffer(static_cast<VkDeviceSize>(out_w) * out_h * 4,
                                    VK_BUFFER_USAGE_TRANSFER_DST_BIT, true);
        }

        vkCheck(vkResetFences(device, 1, &fence), "vkResetFences");
        vkCheck(vkResetDescriptorPool(device, desc_pool, 0), "vkResetDescriptorPool");
        vkCheck(vkResetCommandBuffer(cmd, 0), "vkResetCommandBuffer");
        VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        vkCheck(vkBeginCommandBuffer(cmd, &bi), "vkBeginCommandBuffer");

        staging_used = 0;
        text_runs.clear();
        polygon_offsets.clear();
        polygon_points.clear();
        bound_target = nullptr;

        // Pre-pass: uploads (images, glyphs, polygon points) before any draw.
        prepassLayer(stage.root());
        uploadAtlasIfNeeded();
        if (!polygon_points.empty()) {
            const VkDeviceSize bytes =
                std::min<VkDeviceSize>(polygon_points.size() * sizeof(Vec2), points_ssbo.size);
            std::memcpy(points_ssbo.mapped, polygon_points.data(), bytes);
        }
        // Drop image cache entries whose layers vanished.
        for (auto it = image_cache.begin(); it != image_cache.end();) {
            if (it->second.frame_used != frame_counter) {
                vkDeviceWaitIdle(device);
                destroyImage(it->second.image);
                it = image_cache.erase(it);
            } else {
                ++it;
            }
        }

        // Logical canvas → output mapping (identical to the CPU reference).
        const float W = stage.width();
        const float H = stage.height();
        float sx = out_w / W;
        float sy = out_h / H;
        if (stage.fit() == Fit::Contain) {
            sx = sy = std::min(sx, sy);
        } else if (stage.fit() == Fit::Cover) {
            sx = sy = std::max(sx, sy);
        }
        const Mat23 m0 = Mat23::translation({(out_w - W * sx) * 0.5f, (out_h - H * sy) * 0.5f}) *
                         Mat23::scaling({sx, sy});

        canvas.needs_clear = true;
        ensureTarget(canvas);
        renderLayer(canvas, stage.root(), m0, {W, H});

        // Convert premultiplied float → straight RGBA8 and read back.
        endTarget();
        toSampled(canvas);
        out_rgba.needs_clear = true;
        ensureTarget(out_rgba);
        Push push{};
        drawFullscreen(pipe_unpremul, texSet(canvas.view, sampler_nearest), push);
        endTarget();
        barrier(out_rgba, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
                VK_ACCESS_TRANSFER_READ_BIT);
        VkBufferImageCopy copy{};
        copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
        copy.imageExtent = {out_w, out_h, 1};
        vkCmdCopyImageToBuffer(cmd, out_rgba.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                               readback.buffer, 1, &copy);
        vkCheck(vkEndCommandBuffer(cmd), "vkEndCommandBuffer");

        VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
        si.commandBufferCount = 1;
        si.pCommandBuffers = &cmd;
        vkCheck(vkQueueSubmit(queue, 1, &si, fence), "vkQueueSubmit");
        vkCheck(vkWaitForFences(device, 1, &fence, VK_TRUE, UINT64_MAX), "vkWaitForFences");
        releaseAllTransients();

        const size_t bytes = static_cast<size_t>(out_w) * out_h * 4;
        out_pixels.resize(bytes);
        std::memcpy(out_pixels.data(), readback.mapped, bytes);
        surface.width = out_w;
        surface.height = out_h;
        surface.strideBytes = static_cast<size_t>(out_w) * 4;
        surface.pixels = out_pixels.data();
        return surface;
    }
};

// ===========================================================================
// VulkanRenderer
// ===========================================================================

VulkanRenderer::VulkanRenderer() : VulkanRenderer(Options{}) {}

VulkanRenderer::VulkanRenderer(Options options) : impl_(new Impl) {
    impl_->options = std::move(options);
    impl_->init();
}

VulkanRenderer::~VulkanRenderer() = default;

const Surface& VulkanRenderer::render(Stage& stage, float dt) {
    return impl_->renderFrame(stage, static_cast<uint32_t>(std::lround(stage.width())),
                              static_cast<uint32_t>(std::lround(stage.height())), dt);
}

const Surface& VulkanRenderer::render(Stage& stage, uint32_t out_width, uint32_t out_height,
                                      float dt) {
    return impl_->renderFrame(stage, out_width, out_height, dt);
}

}  // namespace fluent_stage
