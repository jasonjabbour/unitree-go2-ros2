#ifndef NAV2_COSTMAP_2D__LATENTROS_KEY_HPP_
#define NAV2_COSTMAP_2D__LATENTROS_KEY_HPP_

#include <atomic>
#include <cstdint>

// LatentROS: shared state between obstacle_layer (receives scan) and
// costmap_2d_ros (publishes costmap).  C++17 inline variables ensure
// exactly one definition across all translation units in the same binary.
inline uint32_t g_latentros_costmap_key = 0;
// Flag: set by obstacle_layer when new scan data arrives,
// cleared by costmap_2d_ros after publishing.  Makes costmap
// effectively event-driven for benchmarking.
inline std::atomic<bool> g_latentros_new_data{false};

#endif  // NAV2_COSTMAP_2D__LATENTROS_KEY_HPP_
