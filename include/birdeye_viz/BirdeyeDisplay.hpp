//
// Created by jonas on 6/30/21.
//

#ifndef VIZ_BIRDEYEDISPLAY_HPP
#define VIZ_BIRDEYEDISPLAY_HPP

#include <OgreHardwarePixelBuffer.h>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <spatz_interfaces/msg/bird_eye_param.hpp>

#include "birdeye_viz/visibility_control.hpp"

namespace birdeye_viz::displays {

    class BIRDEYE_VIZ_PUBLIC BirdeyeDisplay : public rviz_common::_RosTopicDisplay {
        Q_OBJECT

        using ImageMsg = sensor_msgs::msg::Image;
        using ParamMsg = spatz_interfaces::msg::BirdEyeParam;

      public:
        BirdeyeDisplay();
        ~BirdeyeDisplay() override;

      private:
        void onInitialize() override;

        void reset() override;
        void processMessage(const ImageMsg ::ConstSharedPtr &msg);


        Ogre::ManualObject *imageObject = nullptr;
        Ogre::TexturePtr texture;
        Ogre::MaterialPtr material;

        rclcpp::Subscription<ImageMsg>::SharedPtr imageSub;
        rclcpp::Subscription<ParamMsg>::SharedPtr paramSub;
        std::optional<ParamMsg> currentBirdeyeParam;

        int messagesReceived = 0;

        std::string materialName;
        std::string textureName;

        int currentHeight = 0;
        int currentWidth = 0;

        void incomingMessage(const ImageMsg::ConstSharedPtr &msg);

        /**
         * Create new texture with current image size if it has changed
         */
        void createTextures();
        void subscribe();
        void unsubscribe();

        void resetSubscription();
        void updateTopic() override;
        void setTopic(const QString &topic, const QString &datatype) override;
        void onEnable() override;

        void onDisable() override;
    };

} // namespace birdeye_viz::displays


#endif // VIZ_BIRDEYEDISPLAY_HPP
