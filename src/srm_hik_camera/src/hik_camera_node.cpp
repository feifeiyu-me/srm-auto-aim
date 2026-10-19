#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include "MvCameraControl.h"

class HikCameraNode : public rclcpp::Node
{
public:
    HikCameraNode() : Node("hik_camera_node")
    {
        rotate_180_ = declare_parameter<bool>("rotate_180", true);
        show_preview_ = declare_parameter<bool>("show_preview", true);
        const double exposure_us =
            declare_parameter<double>("exposure_time_us", 3000.0);
        const double gain_db =
            declare_parameter<double>("gain_db", 0.0);

        publisher_ = create_publisher<sensor_msgs::msg::Image>(
            "/srm_aim/image_raw", rclcpp::SensorDataQoS());

        try {
            MV_CC_DEVICE_INFO_LIST devices = {};
            check(MV_CC_EnumDevices(
                MV_USB_DEVICE | MV_GIGE_DEVICE, &devices),
                "EnumDevices");

            if (devices.nDeviceNum == 0)
                throw std::runtime_error("No camera found");

            check(MV_CC_CreateHandle(&handle_, devices.pDeviceInfo[0]),
                  "CreateHandle");
            check(MV_CC_OpenDevice(handle_), "OpenDevice");
            opened_ = true;

            check(MV_CC_SetEnumValue(handle_, "TriggerMode", 0),
                  "Set TriggerMode");
            const bool exposure_auto =
                declare_parameter<bool>("exposure_auto", false);
            const bool gain_auto =
                declare_parameter<bool>("gain_auto", false);

            check(MV_CC_SetEnumValueByString(
                handle_, "ExposureAuto",
                exposure_auto ? "Continuous" : "Off"),
                "Set ExposureAuto");
            check(MV_CC_SetEnumValueByString(
                handle_, "GainAuto",
                gain_auto ? "Continuous" : "Off"),
                "Set GainAuto");

            if (!exposure_auto)
                set_manual_float("ExposureTime", exposure_us);
            if (!gain_auto)
                set_manual_float("Gain", gain_db);

            RCLCPP_INFO(get_logger(),
                "ExposureAuto=%s GainAuto=%s",
                exposure_auto ? "Continuous" : "Off",
                gain_auto ? "Continuous" : "Off");

            check(MV_CC_StartGrabbing(handle_), "StartGrabbing");
            grabbing_ = true;

            if (show_preview_)
                cv::namedWindow("HikRobot ROS", cv::WINDOW_NORMAL);

            timer_ = create_wall_timer(
                std::chrono::milliseconds(33),
                [this]() { capture(); });

            RCLCPP_INFO(get_logger(),
                "Camera started; rotate_180=%s; publishing /srm_aim/image_raw",
                rotate_180_ ? "true" : "false");
        } catch (...) {
            cleanup();
            throw;
        }
    }

    ~HikCameraNode() override
    {
        cleanup();
    }

private:
    static void check(int ret, const char* operation)
    {
        if (ret != 0) {
            char message[160];
            std::snprintf(message, sizeof(message),
                          "%s failed: 0x%08X", operation,
                          static_cast<unsigned int>(ret));
            throw std::runtime_error(message);
        }
    }

    void set_manual_float(const char* key, double value)
    {
        MVCC_FLOATVALUE limits = {};
        check(MV_CC_GetFloatValue(handle_, key, &limits), key);

        if (!std::isfinite(value) ||
            value < limits.fMin || value > limits.fMax) {
            char message[200];
            std::snprintf(message, sizeof(message),
                "%s=%.3f outside camera range [%.3f, %.3f]",
                key, value, limits.fMin, limits.fMax);
            throw std::runtime_error(message);
        }

        check(MV_CC_SetFloatValue(
            handle_, key, static_cast<float>(value)), key);

        MVCC_FLOATVALUE actual = {};
        check(MV_CC_GetFloatValue(handle_, key, &actual), key);

        RCLCPP_INFO(get_logger(),
            "%s requested=%.3f actual=%.3f range=[%.3f, %.3f]",
            key, value, actual.fCurValue, actual.fMin, actual.fMax);
    }

    void capture()
    {
        MV_FRAME_OUT frame = {};
        int ret = MV_CC_GetImageBuffer(handle_, &frame, 200);

        if (ret != 0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "GetImageBuffer failed: 0x%08X",
                static_cast<unsigned int>(ret));
            return;
        }

        // 此时间戳是主机收到图像的时间，并非硬件曝光时间。
        const auto received_stamp = now();
        cv::Mat bgr;

        try {
            bgr.create(frame.stFrameInfo.nHeight,
                       frame.stFrameInfo.nWidth, CV_8UC3);

            MV_CC_PIXEL_CONVERT_PARAM convert = {};
            convert.nWidth = frame.stFrameInfo.nWidth;
            convert.nHeight = frame.stFrameInfo.nHeight;
            convert.pSrcData = frame.pBufAddr;
            convert.nSrcDataLen = frame.stFrameInfo.nFrameLen;
            convert.enSrcPixelType = frame.stFrameInfo.enPixelType;
            convert.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
            convert.pDstBuffer = bgr.data;
            convert.nDstBufferSize = static_cast<unsigned int>(
                bgr.total() * bgr.elemSize());

            ret = MV_CC_ConvertPixelType(handle_, &convert);
        } catch (...) {
            MV_CC_FreeImageBuffer(handle_, &frame);
            throw;
        }

        const int free_ret = MV_CC_FreeImageBuffer(handle_, &frame);
        check(ret, "ConvertPixelType");
        check(free_ret, "FreeImageBuffer");

        if (rotate_180_)
            cv::rotate(bgr, bgr, cv::ROTATE_180);

        std_msgs::msg::Header header;
        header.stamp = received_stamp;
        header.frame_id = "camera";

        auto message = cv_bridge::CvImage(
            header, "bgr8", bgr).toImageMsg();
        publisher_->publish(*message);

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "Publishing %d x %d BGR images", bgr.cols, bgr.rows);

        if (show_preview_) {
            cv::imshow("HikRobot ROS", bgr);
            const int key = cv::waitKey(1) & 0xff;
            if (key == 27 || key == 'q' || key == 'Q')
                rclcpp::shutdown();
        }
    }

    void cleanup() noexcept
    {
        if (grabbing_) {
            MV_CC_StopGrabbing(handle_);
            grabbing_ = false;
        }
        if (opened_) {
            MV_CC_CloseDevice(handle_);
            opened_ = false;
        }
        if (handle_) {
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
        }
        if (show_preview_) {
            try { cv::destroyAllWindows(); } catch (...) {}
        }
    }

    void* handle_ = nullptr;
    bool opened_ = false;
    bool grabbing_ = false;
    bool rotate_180_ = true;
    bool show_preview_ = true;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    int result = 0;

    try {
        auto node = std::make_shared<HikCameraNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Camera node error: %s\n", e.what());
        result = 1;
    }

    if (rclcpp::ok())
        rclcpp::shutdown();

    return result;
}
