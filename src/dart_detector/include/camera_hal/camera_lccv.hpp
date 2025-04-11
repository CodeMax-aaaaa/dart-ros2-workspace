/**
 * @file camera_lccv.hpp
 * @note lccv相机HAL驱动实现
 */
#ifndef CAMERA_lccv_HPP
#define CAMERA_lccv_HPP

#include <camera_hal/camera_driver.hpp>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <lccv.hpp>
#include <string>

namespace CameraHAL
{
    /**
     * @brief lccv相机驱动
     */

    class CameraDriver_lccv : public CameraDriver
    {
    private:
        lccv::PiCamera camera; ///< LCCV 相机对象
        bool isOpened;         ///< 相机是否已打开
    public:
        CameraDriver_lccv();
        ~CameraDriver_lccv();

        bool open(std::unordered_map<std::string, std::string> &params) override;
        bool write(std::string para_name, std::string para_value) override;
        bool read(cv::Mat &image) override;
        bool close() override;
    };
} // namespace CameraHAL

#endif