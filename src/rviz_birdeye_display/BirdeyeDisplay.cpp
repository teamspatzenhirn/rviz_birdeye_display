//
// Created by jonas on 6/30/21.
//

#include "rviz_birdeye_display/BirdeyeDisplay.hpp"

#include <OgreBillboardSet.h>
#include <OgreMaterialManager.h>
#include <OgreMesh.h>
#include <OgreSceneNode.h>
#include <OgreSubMesh.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <opencv2/opencv.hpp>
#include <rviz_rendering/material_manager.hpp>
#include <sensor_msgs/image_encodings.hpp>

constexpr auto RESOURCEGROUP_NAME = "rviz_rendering";

namespace rviz_birdeye_display::displays {

    int cvTypeFromEncoding(const std::string &encoding) {
        using namespace sensor_msgs::image_encodings;
        if (encoding.rfind("32FC", 0) == 0) {
            return CV_32FC(numChannels(encoding));
        }
        if (bitDepth(encoding) == 8) {
            // This case covers all bgr/rgb/bayer 8bit cases. OpenCV does not differentiate between them.
            return CV_8UC(numChannels(encoding));
        }

        throw std::invalid_argument{"OpenCV Type for encoding could not be found"};
    }


    std::string parentTopic(const std::string &topic) {
        return topic.substr(0, topic.find_last_of('/'));
    }

    BirdeyeDisplay::BirdeyeDisplay() {
        std::string message_type = rosidl_generator_traits::name<ImageMsg>();
        topic_property_->setMessageType(QString::fromStdString(message_type));
        topic_property_->setDescription(QString::fromStdString(message_type) + " topic to subscribe to.");

        static int birdeye_count = 0;
        birdeye_count++;
        materialName = "BirdeyeMaterial" + std::to_string(birdeye_count);
        textureName = "BirdeyeTexture" + std::to_string(birdeye_count);
    }

    BirdeyeDisplay::~BirdeyeDisplay() {
        if (initialized()) {
            scene_manager_->destroyManualObject(imageObject);
        }
        unsubscribe();
    }

    void BirdeyeDisplay::createTextures() {
        assert(currentBirdeyeParam.has_value());

        if (currentBirdeyeParam->height == currentHeight and currentBirdeyeParam->width == currentWidth) {
            return;
        }

        texture = Ogre::TextureManager::getSingleton().createManual(
                textureName, RESOURCEGROUP_NAME, Ogre::TEX_TYPE_2D, currentBirdeyeParam->width,
                currentBirdeyeParam->height, 1, 0, Ogre::PF_BYTE_BGRA, Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

        material = rviz_rendering::MaterialManager::createMaterialWithNoLighting(materialName);

        auto rpass = material->getTechniques()[0]->getPasses()[0];
        rpass->createTextureUnitState(textureName);
        rpass->setCullingMode(Ogre::CULL_NONE);
        rpass->setEmissive(Ogre::ColourValue::White);
        rpass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);

        currentHeight = currentBirdeyeParam->height;
        currentWidth = currentBirdeyeParam->width;
    }

    void BirdeyeDisplay::onInitialize() {
        _RosTopicDisplay::onInitialize();

        imageObject = scene_manager_->createManualObject();
        imageObject->setDynamic(true);
        scene_node_->attachObject(imageObject);
    }

    void BirdeyeDisplay::reset() {
        _RosTopicDisplay::reset();
        imageObject->clear();
        messagesReceived = 0;
    }

    void BirdeyeDisplay::processMessage(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {

        if (not currentBirdeyeParam.has_value()) {
            setStatus(rviz_common::properties::StatusProperty::Error, "Params", QString("No BirdEyeParam"));
            return;
        }
        setStatus(rviz_common::properties::StatusProperty::Ok, "Params", QString("OK"));

        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        if (!context_->getFrameManager()->getTransform(currentBirdeyeParam->header.frame_id, msg->header.stamp,
                                                       position, orientation)) {
            setMissingTransformToFixedFrame(currentBirdeyeParam->header.frame_id);
            return;
        }
        setTransformOk();

        scene_node_->setPosition(position);
        scene_node_->setOrientation(orientation);

        createTextures();

        imageObject->clear();
        imageObject->estimateVertexCount(4);
        imageObject->begin(material->getName(), Ogre::RenderOperation::OT_TRIANGLE_FAN, "rviz_rendering");

        auto xOffset = currentBirdeyeParam->offset.x / currentBirdeyeParam->resolution;
        auto yOffset = currentBirdeyeParam->offset.y / currentBirdeyeParam->resolution;
        auto height = currentBirdeyeParam->height / currentBirdeyeParam->resolution;
        auto width = currentBirdeyeParam->width / currentBirdeyeParam->resolution;

        /**
         *        birdeye-height
         *     2------------------1
         *     |                  |
         *     |                  | birdeye-width
         *     |                  |
         *     3------------------0
         *     |---------|     I
         *      y-offset ^     I birdeye-x-offset
         *               |     I
         *   driving dir |     I
         *               |     I
         *             Spatz   I
         */

        // 0
        imageObject->position(xOffset, height - yOffset, 0);
        imageObject->textureCoord(0, 0);

        // 1
        imageObject->position(xOffset + width, height - yOffset, 0);
        imageObject->textureCoord(1, 0);

        // 2
        imageObject->position(xOffset + width, -yOffset, 0);
        imageObject->textureCoord(1, 1);

        // 3
        imageObject->position(xOffset, -yOffset, 0);
        imageObject->textureCoord(0, 1);
        imageObject->end();

        Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture->getBuffer();

        // Lock the pixel buffer and get a pixel box
        pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
        const Ogre::PixelBox &pixelBox = pixelBuffer->getCurrentLock();

        cv::Mat input(msg->height, msg->width, cvTypeFromEncoding(msg->encoding), (void *) msg->data.data(), msg->step);

        cv::Mat textureMat(currentHeight, currentWidth, CV_8UC4, (void *) pixelBox.data);

        if (sensor_msgs::image_encodings::numChannels(msg->encoding) == 1) {
            cv::cvtColor(input, textureMat, cv::COLOR_GRAY2BGRA, 4);
        } else if (msg->encoding.rfind("rgb", 0) == 0) {
            cv::cvtColor(input, textureMat, cv::COLOR_RGB2BGRA, 4);
        } else if (msg->encoding.rfind("bgr", 0) == 0) {
            cv::cvtColor(input, textureMat, cv::COLOR_BGR2BGRA, 4);
        } else {
            throw std::runtime_error{"Unknown encoding: " + msg->encoding};
        }

        // Unlock the pixel buffer
        pixelBuffer->unlock();
    }

    void BirdeyeDisplay::subscribe() {
        if (!isEnabled()) {
            return;
        }

        if (topic_property_->isEmpty()) {
            setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
                      QString("Error subscribing: Empty topic name"));
            return;
        }

        try {
            imageSub = rviz_ros_node_.lock()->get_raw_node()->create_subscription<ImageMsg>(
                    topic_property_->getTopicStd(), qos_profile,
                    [this](ImageMsg::ConstSharedPtr msg) { incomingMessage(msg); });

            paramSub = rviz_ros_node_.lock()->get_raw_node()->create_subscription<ParamMsg>(
                    parentTopic(topic_property_->getTopicStd()) + "/params", qos_profile,
                    [this](ParamMsg ::ConstSharedPtr msg) { this->currentBirdeyeParam = *msg; });

            setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
        } catch (std::exception &e) {
            setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
                      QString("Error subscribing: ") + e.what());
        }
    }

    void BirdeyeDisplay::unsubscribe() {
        imageSub.reset();
        paramSub.reset();
    }

    void BirdeyeDisplay::updateTopic() {
        resetSubscription();
    }

    void BirdeyeDisplay::resetSubscription() {
        unsubscribe();
        reset();
        subscribe();
        context_->queueRender();
    }

    void BirdeyeDisplay::incomingMessage(const ImageMsg::ConstSharedPtr &msg) {
        if (!msg) {
            return;
        }

        ++messagesReceived;
        setStatus(rviz_common::properties::StatusProperty::Ok, "Topic",
                  QString::number(messagesReceived) + " messages received");

        processMessage(msg);
    }

    void BirdeyeDisplay::setTopic(const QString &topic, const QString & /* datatype */) {
        topic_property_->setString(topic);
    }

    void BirdeyeDisplay::onEnable() {
        subscribe();
    }

    void BirdeyeDisplay::onDisable() {
        unsubscribe();
        reset();
    }


} // namespace rviz_birdeye_display::displays

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_birdeye_display::displays::BirdeyeDisplay, rviz_common::Display)