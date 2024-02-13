#pragma once

// Std
#include <memory>
#include <unordered_map>
#include <vector>

// Aravis
extern "C"
{
#include "arv.h"
}

// ROS
#include "rclcpp/rclcpp.hpp"

// camera_aravis
#include "error.hpp"

namespace camera_aravis
{
class CameraAravis : public rclcpp::Node
{
    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Initialization constructor
     *
     * @param[in] options Node options.
     */
    explicit CameraAravis(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief Default destructor.
     *
     */
    ~CameraAravis() override;

  private:
    /**
     * @brief Set the up launch parameters.
     */
    void setup_parameters();

    /**
     * @brief Discover attached camera devices found by Aravis and open device specified by guid.
     *
     * @return True if successful. False, otherwise.
     */
    [[nodiscard]] bool discover_and_open_camera_device();

    /**
     * @brief Discover features available on the camera.
     */
    void discover_features();

    //--- MEMBER DECLARATION ---//
  private:
    /// Logger object of node.
    rclcpp::Logger logger_;

    /// Object for accepting error from aravis functions.
    GuardedGError err_;

    /// Pointer to Aravis device.
    ArvDevice* p_device_;

    /// Pointer to Aravis camera.
    ArvCamera* p_camera_;

    /// GUID of camera.
    std::string guid_;

    gint num_streams_;
    std::vector<ArvStream*> p_streams_;
    std::vector<std::string> stream_names_;

    /// TODO: Deprecated?
    bool verbose_;

    /// TODO: Deprecated?
    std::unordered_map<std::string, const bool> implemented_features_;
};

} // namespace camera_aravis
