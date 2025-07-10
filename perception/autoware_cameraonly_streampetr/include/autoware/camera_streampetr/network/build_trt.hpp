#include "autoware/camera_streampetr/network/network.hpp"
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <fstream>
#include <stdexcept>

namespace autoware::camera_streampetr
{

void setSigmoidAndSoftmaxLayersToFP32(nvinfer1::INetworkDefinition* network, const rclcpp::Logger& logger) {
    for (int i = 0; i < network->getNbLayers(); ++i) {
        auto* layer = network->getLayer(i);
        std::string layerName = layer->getName();
        
        // Check if layer name contains "sigmoid" or "softmax" (case insensitive)
        std::transform(layerName.begin(), layerName.end(), layerName.begin(), ::tolower);
        
        if (layerName.find("sigmoid") != std::string::npos) {
            RCLCPP_INFO(logger, "Setting sigmoid layer '%s' to FP32 precision for stability", layer->getName());
            layer->setPrecision(nvinfer1::DataType::kFLOAT);
        }
        
        if (layerName.find("softmax") != std::string::npos) {
            RCLCPP_INFO(logger, "Setting softmax layer '%s' to FP32 precision for stability", layer->getName());
            layer->setPrecision(nvinfer1::DataType::kFLOAT);
        }
        
        if (layer->getType() == nvinfer1::LayerType::kACTIVATION) {
            auto* activationLayer = static_cast<nvinfer1::IActivationLayer*>(layer);
            if (activationLayer->getActivationType() == nvinfer1::ActivationType::kSIGMOID) {
                RCLCPP_INFO(logger, "Setting sigmoid activation layer '%s' to FP32 precision for stability", layer->getName());
                layer->setPrecision(nvinfer1::DataType::kFLOAT);
            }
        }
        
        if (layer->getType() == nvinfer1::LayerType::kSOFTMAX) {
            RCLCPP_INFO(logger, "Setting softmax layer '%s' to FP32 precision for stability", layer->getName());
            layer->setPrecision(nvinfer1::DataType::kFLOAT);
        }
    }
}

std::string initEngine(const std::string& onnx_file_path, bool fp16_mode, bool set_precision_constraints, const rclcpp::Logger& logger) {
    const std::string engine_file_path = std::filesystem::path(onnx_file_path).replace_extension(".engine").string();
    if (std::filesystem::exists(engine_file_path))
        return engine_file_path;
    RCLCPP_INFO(logger, "Building engine file: %s", engine_file_path.c_str());
    // Create builder
    auto builder = std::unique_ptr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(gLogger));
    if (!builder) {
        throw std::runtime_error("Failed to create TensorRT builder");
    }

    // Create network
    const auto explicitBatch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = std::unique_ptr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(explicitBatch));
    if (!network) {
        throw std::runtime_error("Failed to create network");
    }

    // Create ONNX parser
    auto parser = std::unique_ptr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, gLogger));
    if (!parser) {
        throw std::runtime_error("Failed to create ONNX parser");
    }

    // Parse ONNX file
    std::ifstream file(onnx_file_path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open ONNX file: " + onnx_file_path);
    }
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> buffer(size);
    if (!file.read(buffer.data(), size)) {
        throw std::runtime_error("Failed to read ONNX file");
    }

    if (!parser->parse(buffer.data(), buffer.size())) {
        throw std::runtime_error("Failed to parse ONNX file");
    }

    // Create config
    auto config = std::unique_ptr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
    if (!config) {
        throw std::runtime_error("Failed to create builder config");
    }

    // Set workspace size (8 GiB)
    config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1ULL << 33);

    // Enable FP16 if requested
    if (fp16_mode && builder->platformHasFastFp16()) {
        //TODO: ImplementFP16 mode
        // throw std::runtime_error("FP16 mode is not supported at the moment");
        RCLCPP_INFO(logger, "Building FP16 engine");
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
        
        // Set precision constraints to prefer our layer precision settings
        config->setFlag(nvinfer1::BuilderFlag::kPREFER_PRECISION_CONSTRAINTS);
        if (set_precision_constraints) 
            setSigmoidAndSoftmaxLayersToFP32(network.get(), logger);
    }
    
    // Build engine
    auto engine = std::unique_ptr<nvinfer1::IHostMemory>(builder->buildSerializedNetwork(*network, *config));
    if (!engine) {
        throw std::runtime_error("Failed to build engine");
    }

    // Save engine to file
    std::ofstream engine_file(engine_file_path, std::ios::binary);
    if (!engine_file.is_open()) {
        throw std::runtime_error("Failed to open engine file for writing: " + engine_file_path);
    }
    engine_file.write(static_cast<const char*>(engine->data()), engine->size());
    engine_file.close();

    RCLCPP_INFO(logger, "Engine file built successfully: %s", engine_file_path.c_str());
    return engine_file_path;
}
}