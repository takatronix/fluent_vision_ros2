#pragma once

// FluentLib unified public header
// Purpose:
// - Single include for reusable building blocks in ROS2/C++ vision apps
// - Hide boilerplate (cv_bridge, PC2 conversions, QoS, timers, threads, UI) behind short, readable APIs
// Modules:
// - fluent_cloud: point cloud filters (VoxelGrid/SOR), metrics (PCA length/diameter/curvature), depth→cloud IO
// - fluent_image: thin cv::Mat/ROS Image wrapper with implicit conversions and helpers
// - fluent_lib::ros: params/pub/sub/QoS/log/timers/DSL/FluentNode utilities
// - fluent_ui: lightweight animation and HUD rendering helpers

// Core Fluent utilities (existing aggregate)
#include <fluent.hpp>

// FluentCloud façade (new path)
#include "fluent_lib/fluent_cloud/io.hpp"
#include "fluent_lib/fluent_cloud/filters.hpp"

// Fluent ROS helpers
#include "fluent_lib/ros/timer.hpp"
#include "fluent_lib/ros/params.hpp"
#include "fluent_lib/ros/param_binder.hpp"
#include "fluent_lib/ros/pubsub.hpp"
#include "fluent_lib/ros/log.hpp"
#include "fluent_lib/ros/qos.hpp"
#include "fluent_lib/ros/dsl.hpp"
#include "fluent_lib/ros/timer_registry.hpp"
// Async utilities
#include "fluent_lib/async/worker.hpp"
// Cloud pipelines/metrics
#include "fluent_lib/fluent_cloud/pipeline.hpp"
// UI
#include "fluent_lib/ui/anim.hpp"
#include "fluent_lib/ui/renderer.hpp"
// Node
#include "fluent_lib/ros/fluent_node.hpp"
// Image
#include "fluent_lib/fluent_image/image.hpp"
// Utils
#include "fluent_lib/utils/system.hpp"

// Short, readable aliases for ultra-brief callsites
using FluentImage = fluent_image::Image;


