#pragma once

#include <string>

#include "esp_err.h"

#include "IoRtsManager.hpp"
#include "mqtt_client.h"

// forward declaration
namespace IoRts
{
    class IoRtsManager;
}

namespace Helpers
{
    class MqttHelpers
    {
    public:
        /// @brief Construct a new MqttHelpers object
        /// @param manager Pointer to IoRtsManager object
        MqttHelpers(IoRts::IoRtsManager *manager);
        /// @brief Start MQTT client
        /// @return ESP_OK if no error, ESP_ERR_NOT_ALLOWED if MQTT is not enabled in configuration or already started, ...
        esp_err_t StartMqttClient();

        /// @brief Send a discovery message compatible with Home Assistant
        void SendDiscovery();

        /// @brief Send MQTT device status message for IO device
        /// @param deviceId IO device ID
        void SendIoDeviceStatus(const std::string &deviceId);

        const std::string &GetTopicPrefix() { return mTopicPrefix; }

        /// @brief Returns pointer to IoRtsManager instance
        /// @return pointer to IoRtsManager instance
        IoRts::IoRtsManager *GetIoRtsManager() { return mIoRtsManager; }

        /// @brief Returns IO Home passive mode
        /// @return true if IO Home is in passive mode
        bool isIoHomePassive() { return mIsIoHomePassive; }

    private:
        IoRts::IoRtsManager *mIoRtsManager;         // Pointer to IoRtsManager object
        bool mStarted;                              // true if client is started
        bool mIsIoHomePassive;                      // true if IO Home is in passive mode
        std::string mTopicPrefix;                   // Topic prefix, initialized from configuration storage at boot (avoid to read it from storage everytime!)
        std::string mDiscoveryPrefix;               // Discovery prefix, initialized from configuration storage at boot (avoid to read it from storage everytime!)
        esp_mqtt_client_handle_t mMqttClientHandle; // Handle on MQTT client
    };
}