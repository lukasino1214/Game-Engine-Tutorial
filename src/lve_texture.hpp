#pragma once

#include "lve_device.hpp"
#include <string.h>
#include <vulkan/vulkan_core.h>

namespace lve {
    class Texture {
    public:
        Texture(LveDevice &device, const std::string &filepath);
        ~Texture();

        VkSampler getSampler() { return sampler; }
        VkImageView getImageView() { return imageView; }
        VkImageLayout getImageLayout() { return imageLayout; }
    private:
        void transitionImageLayout(VkImageLayout oldLayout, VkImageLayout newLayout);
        void generateMipmaps();

        int width, height, mipLevels;

        LveDevice& lveDevice;
        VkImage image;
        VkDeviceMemory imageMemory;
        VkImageView imageView;
        VkSampler sampler;
        VkFormat imageFormat;
        VkImageLayout imageLayout;
    };
}