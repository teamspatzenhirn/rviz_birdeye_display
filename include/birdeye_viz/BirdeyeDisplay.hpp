//
// Created by jonas on 6/30/21.
//

#ifndef VIZ_BIRDEYEDISPLAY_HPP
#define VIZ_BIRDEYEDISPLAY_HPP

#include <spatz_interfaces/msg/bird_eye_param.hpp>

#include "OgreHardwarePixelBuffer.h"
#include "birdeye_viz/visibility_control.hpp"
#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/properties/editable_enum_property.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace birdeye_viz::displays {

    class BIRDEYE_VIZ_PUBLIC BirdeyeDisplay : public rviz_common::_RosTopicDisplay {
        Q_OBJECT

        using ImageMsg = sensor_msgs::msg::Image;
        using ParamMsg = spatz_interfaces::msg::BirdEyeParam;

      public:
        BirdeyeDisplay();
        ~BirdeyeDisplay() override;

        void onInitialize() override;

        void reset() override;

      private:
        void processMessage(const ImageMsg ::ConstSharedPtr &msg);


        Ogre::ManualObject *imageObject;

        Ogre::TexturePtr texture;
        Ogre::MaterialPtr material;

        rclcpp::Subscription<ImageMsg>::SharedPtr imageSub;
        rclcpp::Subscription<ParamMsg>::SharedPtr paramSub;
        ParamMsg currentBirdeyeParam;

        int messages_received_;


      protected:
        void updateTopic() override;

      public:
        void setTopic(const QString &topic, const QString &datatype) override;

      private:
        void subscribe();
        void unsubscribe();
        void resetSubscription();

        void incomingMessage(const ImageMsg::ConstSharedPtr &msg);
    };

} // namespace birdeye_viz::displays


#endif // VIZ_BIRDEYEDISPLAY_HPP
