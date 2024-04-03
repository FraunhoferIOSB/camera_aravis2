// Copyright (c) 2024 Fraunhofer IOSB and contributors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Fraunhofer IOSB nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "camera_aravis2/camera_aravis_node_base.h"

// Std
#include <type_traits>

// camera_aravis2
#include "camera_aravis2/error.h"

namespace camera_aravis2
{

//==================================================================================================
CameraAravisNodeBase::CameraAravisNodeBase(const std::string& name,
                                           const rclcpp::NodeOptions& options) :
  Node(name, options),
  is_initialized_(false),
  logger_(this->get_logger()),
  p_device_(nullptr),
  p_camera_(nullptr),
  guid_("")
{
}

//==================================================================================================
CameraAravisNodeBase::~CameraAravisNodeBase()
{
    //--- unreference camera pointer
    if (p_camera_)
        g_object_unref(p_camera_);

    RCLCPP_INFO(logger_, "Node has shut down.");
}

//==================================================================================================
bool CameraAravisNodeBase::isInitialized() const
{
    return (is_initialized_);
}

//==================================================================================================
[[nodiscard]] bool CameraAravisNodeBase::listAvailableCameraDevices() const
{
    //--- Discover available interfaces and devices.

    arv_update_device_list();
    auto n_interfaces = arv_get_n_interfaces();
    auto n_devices    = arv_get_n_devices();

    if (n_devices == 0)
    {
        RCLCPP_FATAL(logger_, "No cameras detected.");
        return false;
    }

    RCLCPP_INFO(logger_, "Attached cameras (Num. Interfaces: %d | Num. Devices: %d):",
                n_interfaces, n_devices);
    for (uint i = 0; i < n_devices; i++)
        RCLCPP_INFO(logger_, "  Device %d: %s (%s)", i,
                    arv_get_device_id(i),
                    arv_get_device_address(i));

    return true;
}

//==================================================================================================
void CameraAravisNodeBase::setUpParameters()
{
    auto guid_desc = rcl_interfaces::msg::ParameterDescriptor{};
    guid_desc.description =
      "Serial number of the camera that is to be opened. If omitted any of the listed cameras "
      "will be opened.";
    declare_parameter<std::string>("guid", "", guid_desc);
}

//==================================================================================================
[[nodiscard]] bool CameraAravisNodeBase::discoverAndOpenCameraDevice()
{
    // Guarded error object
    GuardedGError err;

    //--- Discover available interfaces and devices.

    bool is_successful = listAvailableCameraDevices();
    if (!is_successful)
        return false;

    //--- connect to camera specified by guid parameter
    guid_ = get_parameter("guid").as_string();

    const int MAX_RETRIES = 10;
    int tryCount          = 1;
    while (!p_camera_ && tryCount <= MAX_RETRIES)
    {
        if (guid_ == "")
        {
            RCLCPP_WARN(logger_, "No guid specified.");
            RCLCPP_INFO(logger_, "Opening: (any)");
            p_camera_ = arv_camera_new(nullptr, err.ref());
        }
        else
        {
            RCLCPP_INFO(logger_, "Opening: %s ", guid_.c_str());
            p_camera_ = arv_camera_new(guid_.c_str(), err.ref());
        }

        if (!p_camera_)
        {
            CHECK_GERROR(err, logger_);
            RCLCPP_WARN(logger_, "Unable to open camera. Retrying (%i/%i) ...",
                        tryCount, MAX_RETRIES);
            rclcpp::sleep_for(std::chrono::seconds(1));
            tryCount++;
        }
    }

    if (!p_camera_)
    {
        RCLCPP_FATAL(logger_, "Failed to open any camera.");
        return false;
    }

    //--- get device pointer from camera
    p_device_ = arv_camera_get_device(p_camera_);

    //--- connect control-lost signal
    g_signal_connect(p_device_, "control-lost",
                     (GCallback)CameraAravisNodeBase::handleControlLostSignal, this);

    return true;
}

//==================================================================================================
[[nodiscard]] bool CameraAravisNodeBase::getNestedParameter(
  const std::string& parent_name,
  const std::string& param_name,
  rclcpp::ParameterValue& param_value) const
{
    std::string key = parent_name + "." + param_name;
    if (parameter_overrides_.find(key) != parameter_overrides_.end())
    {
        param_value = parameter_overrides_.at(key);
        return true;
    }
    else
    {
        return false;
    }
}

//==================================================================================================
[[nodiscard]] bool CameraAravisNodeBase::getNestedParameterList(
  const std::string& parent_name,
  const std::string& param_name,
  std::vector<std::pair<std::string, rclcpp::ParameterValue>>& param_values) const
{
    param_values.clear();

    std::string key = parent_name + "." + param_name;
    for (auto itr = parameter_overrides_.begin();
         itr != parameter_overrides_.end();
         ++itr)
    {
        //--- if parameter override begins with key
        if (itr->first.rfind(key) == 0)
        {
            std::string feature_name = itr->first.substr(key.size() + 1);
            param_values.push_back(std::make_pair(feature_name, itr->second));
        }
    }

    if (param_values.empty())
        return false;
    else
        return true;
}

//==================================================================================================
template <typename T>
bool CameraAravisNodeBase::getFeatureValue(const std::string& feature_name, T& value) const
{
    bool is_successful = true;
    GuardedGError err;

    //--- assert that p_device is set
    if (!p_device_)
        return false;

    //--- check if feature is available
    if (!arv_device_is_feature_available(p_device_, feature_name.c_str(), err.ref()))
    {
        return false;
    }

    if constexpr (std::is_same_v<T, bool>)
    {
        value = arv_device_get_boolean_feature_value(p_device_, feature_name.c_str(), err.ref());
    }
    else if constexpr (std::is_same_v<T, std::string>)
    {
        value = arv_device_get_string_feature_value(p_device_, feature_name.c_str(), err.ref());
    }
    else if constexpr (std::is_same_v<T, int>)
    {
        value = arv_device_get_integer_feature_value(p_device_, feature_name.c_str(), err.ref());
    }
    else if constexpr (std::is_same_v<T, float>)
    {
        value = static_cast<float>(
          arv_device_get_float_feature_value(p_device_, feature_name.c_str(), err.ref()));
    }
    else if constexpr (std::is_same_v<T, double>)
    {
        value = arv_device_get_float_feature_value(p_device_, feature_name.c_str(), err.ref());
    }
    else
    {
        RCLCPP_WARN(logger_, "Getting feature of type '%s' is currently not supported.",
                    typeid(T).name());
    }

    ASSERT_GERROR_MSG(err, logger_,
                      "In getting value for feature '" + feature_name + "'.", is_successful);

    return is_successful;
}
template bool CameraAravisNodeBase::getFeatureValue(const std::string&, bool&) const;
template bool CameraAravisNodeBase::getFeatureValue(const std::string&, std::string&) const;
template bool CameraAravisNodeBase::getFeatureValue(const std::string&, int&) const;
template bool CameraAravisNodeBase::getFeatureValue(const std::string&, float&) const;
template bool CameraAravisNodeBase::getFeatureValue(const std::string&, double&) const;

//==================================================================================================
template <typename T>
bool CameraAravisNodeBase::setFeatureValue(const std::string& feature_name, const T& value) const
{
    bool is_successful = true;
    GuardedGError err;

    //--- assert that p_device is set
    if (!p_device_)
        return false;

    //--- check if feature is available
    if (!arv_device_is_feature_available(p_device_, feature_name.c_str(), err.ref()))
    {
        RCLCPP_WARN(logger_, "Feature '%s' is not available. Value will not be set.",
                    feature_name.c_str());
        ASSERT_GERROR(err, logger_, is_successful);
        return false;
    }

    if constexpr (std::is_same_v<T, bool>)
    {
        arv_device_set_boolean_feature_value(p_device_, feature_name.c_str(), value, err.ref());
    }
    else if constexpr (std::is_same_v<T, std::string>)
    {
        arv_device_set_string_feature_value(p_device_, feature_name.c_str(),
                                            static_cast<std::string>(value).c_str(), err.ref());
    }
    else if constexpr (std::is_same_v<T, int>)
    {
        arv_device_set_integer_feature_value(p_device_, feature_name.c_str(), value, err.ref());
    }
    else if constexpr (std::is_same_v<T, int64_t>)
    {
        arv_device_set_integer_feature_value(p_device_, feature_name.c_str(), value, err.ref());
    }
    else if constexpr (std::is_same_v<T, float>)
    {
        arv_device_set_float_feature_value(p_device_, feature_name.c_str(),
                                           static_cast<double>(value), err.ref());
    }
    else if constexpr (std::is_same_v<T, double>)
    {
        arv_device_set_float_feature_value(p_device_, feature_name.c_str(),
                                           value, err.ref());
    }
    else
    {
        RCLCPP_WARN(logger_, "Setting feature of type '%s' is currently not supported. "
                             "Value will not be set.",
                    typeid(T).name());
    }

    ASSERT_GERROR_MSG(err, logger_,
                      "In setting value for feature '" + feature_name + "'.", is_successful);

    return is_successful;
}
template bool CameraAravisNodeBase::setFeatureValue(const std::string&, const bool&) const;
template bool CameraAravisNodeBase::setFeatureValue(const std::string&, const std::string&) const;
template bool CameraAravisNodeBase::setFeatureValue(const std::string&, const int&) const;
template bool CameraAravisNodeBase::setFeatureValue(const std::string&, const int64_t&) const;
template bool CameraAravisNodeBase::setFeatureValue(const std::string&, const float&) const;
template bool CameraAravisNodeBase::setFeatureValue(const std::string&, const double&) const;

//==================================================================================================
template <typename T>
bool CameraAravisNodeBase::setFeatureValueFromParameter(
  const std::string& feature_name,
  const rclcpp::ParameterValue& parameter_value,
  const uint& idx) const
{
    T value;

    try
    {
        //--- check if single parameter of parameter array
        //--- BYTE_ARRAY is the first 'array' type in the list
        if (parameter_value.get_type() < rclcpp::PARAMETER_BYTE_ARRAY)
        {
            value = parameter_value.get<T>();
        }
        else
        {
            // List of values that are to be set. If the list is smaller than the number of streams
            // the last value of the is used for the remaining streams.
            std::vector<T> value_list = parameter_value.get<std::vector<T>>();

            if (value_list.empty())
                return false;

            value = value_list.at(std::min(idx, static_cast<uint>(value_list.size() - 1)));
        }
    }
    catch (const rclcpp::ParameterTypeException& e)
    {
        RCLCPP_ERROR(logger_, "Exception while trying to set value for '%s'. "
                              "Reason: %s",
                     feature_name.c_str(), e.what());
        return false;
    }

    return setFeatureValue<T>(feature_name, value);
}
template bool CameraAravisNodeBase::setFeatureValueFromParameter<bool>(
  const std::string&, const rclcpp::ParameterValue&, const uint&) const;
template bool CameraAravisNodeBase::setFeatureValueFromParameter<std::string>(
  const std::string&, const rclcpp::ParameterValue&, const uint&) const;
template bool CameraAravisNodeBase::setFeatureValueFromParameter<int64_t>(
  const std::string&, const rclcpp::ParameterValue&, const uint&) const;
template bool CameraAravisNodeBase::setFeatureValueFromParameter<double>(
  const std::string&, const rclcpp::ParameterValue&, const uint&) const;

//==================================================================================================
bool CameraAravisNodeBase::setFeatureValuesFromParameterList(
  const std::vector<std::pair<std::string, rclcpp::ParameterValue>>& param_values,
  const uint& idx) const
{
    bool is_successful = true;

    for (auto itr = param_values.begin(); itr != param_values.end(); ++itr)
    {
        if (itr->second.get_type() == rclcpp::PARAMETER_BOOL ||
            itr->second.get_type() == rclcpp::PARAMETER_BOOL_ARRAY)
        {
            is_successful &= setFeatureValueFromParameter<bool>(itr->first, itr->second, idx);
        }
        else if (itr->second.get_type() == rclcpp::PARAMETER_STRING ||
                 itr->second.get_type() == rclcpp::PARAMETER_STRING_ARRAY)
        {
            is_successful &= setFeatureValueFromParameter<std::string>(itr->first, itr->second,
                                                                       idx);
        }
        else if (itr->second.get_type() == rclcpp::PARAMETER_INTEGER ||
                 itr->second.get_type() == rclcpp::PARAMETER_INTEGER_ARRAY)
        {
            is_successful &= setFeatureValueFromParameter<int64_t>(itr->first, itr->second, idx);
        }
        else if (itr->second.get_type() == rclcpp::PARAMETER_DOUBLE ||
                 itr->second.get_type() == rclcpp::PARAMETER_DOUBLE_ARRAY)
        {
            is_successful &= setFeatureValueFromParameter<double>(itr->first, itr->second, idx);
        }
        else
        {
            RCLCPP_ERROR(logger_, "Parameter '%s' is of unknown type. ",
                         itr->first.c_str());
            is_successful = false;
        }
    }

    return is_successful;
}

//==================================================================================================
template <typename T>
bool CameraAravisNodeBase::setBoundedFeatureValueFromParameter(
  const std::string& feature_name,
  const T& min, const T& max,
  const rclcpp::ParameterValue& parameter_value,
  const uint& idx) const
{
    T bounded_value;

    try
    {
        //--- check if single parameter of parameter array
        //--- BYTE_ARRAY is the first 'array' type in the list
        if (parameter_value.get_type() < rclcpp::PARAMETER_BYTE_ARRAY)
        {
            bounded_value = std::max(min, std::min(parameter_value.get<T>(), max));
        }
        else
        {
            // List of values that are to be set. If the list is smaller than the number of streams
            // the last value of the is used for the remaining streams.
            std::vector<T> value_list = parameter_value.get<std::vector<T>>();

            if (value_list.empty())
                return false;

            T unbounded_value = value_list.at(std::min(idx,
                                                       static_cast<uint>(value_list.size() - 1)));
            bounded_value     = std::max(min, std::min(unbounded_value, max));
        }
    }
    catch (const rclcpp::ParameterTypeException& e)
    {
        RCLCPP_ERROR(logger_, "Exception while trying to set value for '%s'. "
                              "Reason: %s",
                     feature_name.c_str(), e.what());
        return false;
    }

    return setFeatureValue<T>(feature_name, bounded_value);
}
template bool CameraAravisNodeBase::setBoundedFeatureValueFromParameter<bool>(
  const std::string&, const bool&, const bool&,
  const rclcpp::ParameterValue&, const uint&) const;
template bool CameraAravisNodeBase::setBoundedFeatureValueFromParameter<std::string>(
  const std::string&, const std::string&, const std::string&,
  const rclcpp::ParameterValue&, const uint&) const;
template bool CameraAravisNodeBase::setBoundedFeatureValueFromParameter<int64_t>(
  const std::string&, const int64_t&, const int64_t&,
  const rclcpp::ParameterValue&, const uint&) const;
template bool CameraAravisNodeBase::setBoundedFeatureValueFromParameter<double>(
  const std::string&, const double&, const double&,
  const rclcpp::ParameterValue&, const uint&) const;

//==================================================================================================
template <typename T>
bool CameraAravisNodeBase::isParameterValueEqualTo(const rclcpp::ParameterValue& parameter_value,
                                                   const T& test_value,
                                                   const uint& idx) const
{
    T value;

    try
    {
        //--- check if single parameter of parameter array
        //--- BYTE_ARRAY is the first 'array' type in the list
        if (parameter_value.get_type() < rclcpp::PARAMETER_BYTE_ARRAY)
        {
            value = parameter_value.get<T>();
        }
        else
        {
            // List of values that are to be set. If the list is smaller than the number of streams
            // the last value of the is used for the remaining streams.
            std::vector<T> value_list = parameter_value.get<std::vector<T>>();

            if (value_list.empty())
                return false;

            value = value_list.at(std::min(idx, static_cast<uint>(value_list.size() - 1)));
        }
    }
    catch (const rclcpp::ParameterTypeException& e)
    {
        RCLCPP_ERROR_STREAM(logger_, "Exception while trying to compare parameter value to '"
                                       << test_value << "'. Reason: " << e.what());
        return false;
    }

    return (value == test_value);
}
template bool CameraAravisNodeBase::isParameterValueEqualTo<bool>(
  const rclcpp::ParameterValue&, const bool&, const uint&) const;
template bool CameraAravisNodeBase::isParameterValueEqualTo<std::string>(
  const rclcpp::ParameterValue&, const std::string&, const uint&) const;
template bool CameraAravisNodeBase::isParameterValueEqualTo<int64_t>(
  const rclcpp::ParameterValue&, const int64_t&, const uint&) const;
template bool CameraAravisNodeBase::isParameterValueEqualTo<double>(
  const rclcpp::ParameterValue&, const double&, const uint&) const;

//==================================================================================================
std::string CameraAravisNodeBase::constructCameraGuidStr(ArvCamera* p_cam)
{
    const char* vendor_name = arv_camera_get_vendor_name(p_cam, nullptr);
    const char* model_name  = arv_camera_get_model_name(p_cam, nullptr);
    const char* device_sn   = arv_camera_get_device_serial_number(p_cam, nullptr);
    const char* device_id   = arv_camera_get_device_id(p_cam, nullptr);

    return (std::string(vendor_name) + "-" +
            std::string(model_name) + "-" +
            std::string((device_sn) ? device_sn : device_id));
}

//==================================================================================================
void CameraAravisNodeBase::handleControlLostSignal(ArvDevice* p_device, gpointer p_user_data)
{
    RCL_UNUSED(p_device);

    CameraAravisNodeBase* p_ca_instance = reinterpret_cast<CameraAravisNodeBase*>(p_user_data);

    if (!p_ca_instance)
        return;

    RCLCPP_FATAL(p_ca_instance->logger_, "Control to aravis device lost.");
    RCLCPP_FATAL(p_ca_instance->logger_, "  GUID: %s", p_ca_instance->guid_.c_str());

    rclcpp::shutdown();
}

}  // end namespace camera_aravis2
