// Std
#include <string>

// GLib
#include <glib-2.0/glib/gerror.h>

// ROS
#include "rclcpp/rclcpp.hpp"

namespace camera_aravis
{

class GuardedGError
{

    //--- METHOD DECLARATION ---//
  public:
    ~GuardedGError() { reset(); }

    void reset() {
      if (!err) return;
      g_error_free(err);
      err = nullptr;
    }

    GError** ref() { 
        this->reset();

        return &err; 
    }

    GError* operator->() noexcept { return err; }

    operator bool() const { return nullptr != err; }

    void log(const rclcpp::Logger& logger)  const {        
        if(err == nullptr) return;

        RCLCPP_ERROR(logger, "[%s] Code %i: %s", 
        g_quark_to_string(err->domain), err->code, err->message);
        
    }

    friend bool operator==(const GuardedGError& lhs, const GError* rhs);
    friend bool operator==(const GuardedGError& lhs, const GuardedGError& rhs);
    friend bool operator!=(const GuardedGError& lhs, std::nullptr_t);

    //--- MEMBER DECLARATION ---//

  private:

    GError *err = nullptr;
  };

  bool operator==(const GuardedGError& lhs, const GError* rhs) { return lhs.err == rhs; }
  bool operator==(const GuardedGError& lhs, const GuardedGError& rhs) { return lhs.err == rhs.err; }
  bool operator!=(const GuardedGError& lhs, std::nullptr_t) { return !!lhs; }

} // namespace camera_aravis