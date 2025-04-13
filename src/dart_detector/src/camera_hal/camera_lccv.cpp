/**
 * @file camera_lccv.cpp
 * @note 树莓派LCCV相机HAL驱动实现
 */
#include <camera_hal/camera_lccv.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

namespace CameraHAL
{
    CameraDriver_LCCV::CameraDriver_LCCV() : isOpened(false) {}

    CameraDriver_LCCV::~CameraDriver_LCCV()
    {
        close();
    }

    bool CameraDriver_LCCV::open(std::unordered_map<std::string, std::string> &params)
    {
        // 获取分辨率和帧率参数，设置默认值
        int width = 640, height = 480, framerate = 30;
        if (params.find("Width") != params.end())
        {
            width = std::stoi(params["Width"]);
        }
        if (params.find("Height") != params.end())
        {
            height = std::stoi(params["Height"]);
        }
        if (params.find("Framerate") != params.end())
        {
            framerate = std::stoi(params["Framerate"]);
        }

        // 配置 LCCV 相机参数
        camera.options->video_width = width;
        camera.options->video_height = height;
        camera.options->framerate = framerate;
        camera.options->verbose = true;

        // 启动视频流
        try
        {
            camera.startVideo();
            isOpened = true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to start LCCV camera: " << e.what() << std::endl;
            return false;
        }

        return true;
    }

    bool CameraDriver_LCCV::write(std::string para_name, std::string para_value)
    {
        if (!isOpened)
        {
            std::cerr << "Camera is not opened" << std::endl;
            return false;
        }

        if (para_name == "Width")
        {
            camera.options->video_width = std::stoi(para_value);
        }
        else if (para_name == "Height")
        {
            camera.options->video_height = std::stoi(para_value);
        }
        else if (para_name == "Framerate")
        {
            camera.options->framerate = std::stoi(para_value);
        }
        
        else
        {
            std::cerr << "Unsupported parameter: " << para_name << std::endl;
            return false;
        }

        return true;
    }

    bool CameraDriver_LCCV::read(cv::Mat &image)
    {
        if (!isOpened)
        {
            std::cerr << "Camera is not opened" << std::endl;
            return false;
        }

        // 获取视频帧
        if (!camera.getVideoFrame(image, 1000))
        {
            std::cerr << "Failed to capture video frame" << std::endl;
            return false;
        }

        return true;
    }

    bool CameraDriver_LCCV::close()
    {
        if (!isOpened)
        {
            return true;
        }

        // 停止视频流
        try
        {
            camera.stopVideo();
            isOpened = false;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to stop LCCV camera: " << e.what() << std::endl;
            return false;
        }

        return true;
    }
} // namespace CameraHAL