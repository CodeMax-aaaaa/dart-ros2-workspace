
#include <string>
#include <functional>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <fstream>
#include <cmath>
#include <iostream>

#include "dart_algorithm.hpp"
// GSL多项拟合
#include <gsl/gsl_poly.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_multifit.h>

#include <gsl/gsl_interp.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_errno.h>

// 时间日期 文件名相关
#include <ctime>
#include <cstdio>
#include <chrono>
#include <unistd.h>
#include <sstream>
#include <iomanip>

void gsl_error_handler(const char *reason, const char *file, int line, int gsl_errno)
{
    throw std::runtime_error(std::string(reason) + " in file " + file + " at line " + std::to_string(line));
}

DartAlgorithm::DartDataBase::DartDataBase()
{
    gsl_set_error_handler(gsl_error_handler); // 设置自定义错误处理器
}

void DartAlgorithm::DartDataBase::notifyUpdate()
{
    if (dart_data_update_callback_)
    {
        dart_data_update_callback_(nullptr);
    }
}

DartAlgorithm::DartDataBase::DartDataBase(YAML::Node yaml_node)
{
    loadFromYaml(yaml_node);

    gsl_set_error_handler(gsl_error_handler); // 设置自定义错误处理器
}

DartAlgorithm::DartDataBase::DartDataBase(std::string file_name) : file_name_(file_name)
{
    loadFromFile(file_name_);

    gsl_set_error_handler(gsl_error_handler); // 设置自定义错误处理器
}

void DartAlgorithm::DartDataBase::loadFromYaml(YAML::Node yaml_node)
{
    dart_data_.clear();
    // 若 yaml_node 为空，则直接返回
    if (yaml_node.IsNull() || yaml_node.size() == 0)
    {
        return;
    }

    for (auto it = yaml_node.begin(); it != yaml_node.end(); ++it)
    {
        DartData_t data;
        try
        {
            data.name = it->first.as<std::string>();
            data.target_yaw_angle_offset = it->second["target_yaw_angle_offset"].as<int32_t>();
            data.target_fw_velocity_offset = it->second["target_fw_velocity_offset"].as<int32_t>(); // Unused

            for (const auto &point : it->second["distance_velocity_points"])
            {
                data.distance_velocity_points.push_back({point["distance"].as<double>(), point["velocity"].as<double>()});
            }

            for (const auto &point : it->second["fw_velocity_points"])
            {
                data.fw_velocity_points.push_back({point["fw_velocity"].as<int>(), point["velocity"].as<double>()});
            }

            for (const auto &point : it->second["distance_calibrated_points"])
            {
                data.distance_calibrated_points.push_back({point["distance"].as<double>(), point["velocity"].as<double>()});
            }

            for (const auto &point : it->second["fw_velocity_calibrated_points"])
            {
                data.fw_velocity_calibrated_points.push_back({point["fw_velocity"].as<int>(), point["velocity"].as<double>()});
            }

            dart_data_.push_back(data);

            // 所有point按照x values must be strictly increasing的插值要求排序
            std::sort(data.distance_velocity_points.begin(), data.distance_velocity_points.end(), [](const DistanceVelocityPoint_t &a, const DistanceVelocityPoint_t &b)
                      { return a.distance < b.distance; });

            std::sort(data.fw_velocity_points.begin(), data.fw_velocity_points.end(), [](const FWVelocityPoint_t &a, const FWVelocityPoint_t &b)
                      { return a.velocity < b.velocity; });
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }
    notifyUpdate();
}

void DartAlgorithm::DartDataBase::saveToYaml(YAML::Node &yaml_node)
{
    for (auto &data : dart_data_)
    {
        yaml_node[data.name]["target_yaw_angle_offset"] = data.target_yaw_angle_offset;
        yaml_node[data.name]["target_fw_velocity_offset"] = data.target_fw_velocity_offset;

        // 所有point按照x values must be strictly increasing的插值要求排序
        std::sort(data.distance_velocity_points.begin(), data.distance_velocity_points.end(), [](const DistanceVelocityPoint_t &a, const DistanceVelocityPoint_t &b)
                  { return a.distance < b.distance; });

        std::sort(data.fw_velocity_points.begin(), data.fw_velocity_points.end(), [](const FWVelocityPoint_t &a, const FWVelocityPoint_t &b)
                  { return a.velocity < b.velocity; });

        for (const auto &point : data.distance_velocity_points)
        {
            YAML::Node point_node;
            point_node["distance"] = point.distance;
            point_node["velocity"] = point.velocity;
            yaml_node[data.name]["distance_velocity_points"].push_back(point_node);
        }

        for (const auto &point : data.fw_velocity_points)
        {
            YAML::Node point_node;
            point_node["fw_velocity"] = point.fw_velocity;
            point_node["velocity"] = point.velocity;
            yaml_node[data.name]["fw_velocity_points"].push_back(point_node);
        }

        for (const auto &point : data.distance_calibrated_points)
        {
            YAML::Node point_node;
            point_node["distance"] = point.distance;
            point_node["velocity"] = point.velocity;
            yaml_node[data.name]["distance_calibrated_points"].push_back(point_node);
        }

        for (const auto &point : data.fw_velocity_calibrated_points)
        {
            YAML::Node point_node;
            point_node["fw_velocity"] = point.fw_velocity;
            point_node["velocity"] = point.velocity;
            yaml_node[data.name]["fw_velocity_calibrated_points"].push_back(point_node);
        }
    }
}

void DartAlgorithm::DartDataBase::loadFromFile(std::string file_name)
{
    if (file_name.empty())
    {
        // 建立一个空的数据库
        dart_data_.clear();
        return;
    }
    try
    {
        YAML::Node yaml_node = YAML::LoadFile(file_name);
        loadFromYaml(yaml_node);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        dart_data_.clear();
    }
    file_name_ = file_name;
}

void DartAlgorithm::DartDataBase::saveToFile(std::string file_name)
{
    YAML::Node yaml_node;
    saveToYaml(yaml_node);
    // 将源文件改名备份
    if (file_name_.size() == 0)
    {
        file_name_ = file_name;
    }

    std::stringstream ss;
    auto now = std::time(nullptr);
    ss << file_name_ << "." << std::put_time(std::localtime(&now), "%Y%m%d%H%M%S");
    std::fstream fs(file_name_);
    if (fs.good())
    {
        fs.close();
        rename(file_name_.c_str(), ss.str().c_str());
    }

    std::ofstream fout(file_name);
    fout << yaml_node;
}

void DartAlgorithm::DartDataBase::saveToFile()
{
    if (file_name_.size() == 0)
    {
        throw std::runtime_error("File name is empty");
    }
    YAML::Node yaml_node;
    saveToYaml(yaml_node);
    std::ofstream fout(file_name_);
    fout << yaml_node;
}

void DartAlgorithm::DartDataBase::setDartDataUpdateCallback(std::function<void(void *)> callback)
{
    dart_data_update_callback_ = callback;
}

int polyfit(const double *x, const double *y, int xyLength, int poly_n, std::vector<double> &out_factor, double &out_chisq)
{
    /*
     * x：自变量，视差
     * y：因变量，距离
     * xyLength: x、y长度
     * poly_n：拟合的阶次
     * out_factor：拟合的系数结果，从0阶到poly_n阶的系数
     * out_chisq：拟合曲线与数据点的优值函数最小值 ,χ2 检验
     */

    gsl_matrix *XX = gsl_matrix_alloc(xyLength, poly_n + 1);
    gsl_vector *c = gsl_vector_alloc(poly_n + 1);
    gsl_matrix *cov = gsl_matrix_alloc(poly_n + 1, poly_n + 1);
    gsl_vector *vY = gsl_vector_alloc(xyLength);

    for (size_t i = 0; i < xyLength; i++)
    {
        gsl_matrix_set(XX, i, 0, 1.0);
        gsl_vector_set(vY, i, y[i]);
        for (int j = 1; j <= poly_n; j++)
        {
            gsl_matrix_set(XX, i, j, pow(x[i], j));
        }
    }

    gsl_multifit_linear_workspace *workspace = gsl_multifit_linear_alloc(xyLength, poly_n + 1);
    int r = gsl_multifit_linear(XX, vY, c, cov, &out_chisq, workspace);
    gsl_multifit_linear_free(workspace);
    out_factor.resize(c->size, 0);
    for (size_t i = 0; i < c->size; ++i)
    {
        out_factor[i] = gsl_vector_get(c, i);
    }

    gsl_vector_free(vY);
    gsl_matrix_free(XX);
    gsl_matrix_free(cov);
    gsl_vector_free(c);

    return 0;
}

int DartAlgorithm::DartDataBase::calculateFWVelocity(std::string dart_name, double velocity)
{
    auto it = std::find_if(dart_data_.begin(), dart_data_.end(), [&dart_name](const DartData_t &data)
                           { return data.name == dart_name; });

    if (it != dart_data_.end())
    {
        size_t n = it->fw_velocity_points.size();

        // 多项式阶数
        const int degree = 2;

        double result;
        try
        {
            std::vector<double> result_factor;

            double x[n], y[n];

            for (size_t i = 0; i < n; i++)
            {
                x[i] = it->fw_velocity_points[i].velocity;
                y[i] = it->fw_velocity_points[i].fw_velocity;
            }

            double chisq;

            polyfit(x, y, n, degree, result_factor, chisq);

            result = 0.0;
            for (size_t i = 0; i < result_factor.size(); i++)
            {
                result += result_factor[i] * pow(velocity, i);
            }
            std::cout << "poly fit result: " << result_factor[0] << " " << result_factor[1] << " " << result_factor[2] << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            std::cout << "WARNING: " << dart_name << " fw_velocity_points calculate downgraded to linear regression" << std::endl;
            // 降级成线性回归
            double sum_x = 0.0, sum_y = 0.0;
            double sum_xx = 0.0, sum_xy = 0.0;

            for (const auto &point : it->fw_velocity_points)
            {
                sum_x += point.velocity;
                sum_y += point.fw_velocity;
                sum_xx += point.velocity * point.velocity;
                sum_xy += point.velocity * point.fw_velocity;
            }

            // 计算斜率 (m) 和截距 (b)
            double m_ = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
            double b_ = (sum_y - m_ * sum_x) / n;

            result = (m_ * velocity + b_);
        }

        return int(result);
    }
    return -1; // 没有找到对应的飞镖
}

double DartAlgorithm::DartDataBase::calculateVelocity(std::string dart_name, double distance)
{
    auto it = std::find_if(dart_data_.begin(), dart_data_.end(), [&dart_name](const DartData_t &data)
                           { return data.name == dart_name; });

    if (it != dart_data_.end())
    {
        size_t n = it->distance_velocity_points.size();

        // 多项式阶数
        const int degree = 2;

        double result;

        try
        {
            std::vector<double> result_factor;

            double x[n], y[n];

            for (size_t i = 0; i < n; i++)
            {
                x[i] = it->distance_velocity_points[i].distance;
                y[i] = it->distance_velocity_points[i].velocity;
            }

            double chisq;

            polyfit(x, y, n, degree, result_factor, chisq);

            result = 0.0;
            for (size_t i = 0; i < result_factor.size(); i++)
            {
                result += result_factor[i] * pow(distance, i);
            }
            std::cout << "poly fit result: " << result_factor[0] << " " << result_factor[1] << " " << result_factor[2] << std::endl;
        }

        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            std::cout << "WARNING: " << dart_name << " distance_velocity_points calculate downgraded to linear regression" << std::endl;
            // 降级成线性回归
            double sum_x = 0.0, sum_y = 0.0;
            double sum_xx = 0.0, sum_xy = 0.0;

            for (const auto &point : it->distance_velocity_points)
            {
                sum_x += point.distance;
                sum_y += point.velocity;
                sum_xx += point.distance * point.distance;
                sum_xy += point.distance * point.velocity;
            }

            // 计算斜率 (m) 和截距 (b)
            double m_ = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
            double b_ = (sum_y - m_ * sum_x) / n;

            result = m_ * distance + b_;
        }

        return result;
    }

    return -1.0; // 没有找到对应的飞镖或距离
}

void DartAlgorithm::DartDataBase::calibrateFWVelocity(std::string dart_name, std::vector<FWVelocityPoint_t> fw_velocity_points_input, CalibrationType_t calibration_type)
{
    auto it = std::find_if(dart_data_.begin(), dart_data_.end(), [&dart_name](const DartData_t &data)
                           { return data.name == dart_name; });

    if (it != dart_data_.end())
    {
        switch (calibration_type)
        {
        // case CalibrationType_t::CALIBRATION_TYPE_SPLINE:
        // {
        //     // 对校准表与原表的差值进行三次样条插值，然后将插值结果加到原表上
        //     size_t n = fw_velocity_points_input.size();
        //     double x[n], y[n];
        //     for (size_t i = 0; i < n; i++)
        //     {
        //         x[i] = fw_velocity_points_input[i].velocity;
        //         y[i] = fw_velocity_points_input[i].fw_velocity - calculateFWVelocity(dart_name, fw_velocity_points_input[i].velocity);
        //     }

        //     gsl_interp_accel *acc = gsl_interp_accel_alloc();
        //     gsl_spline *spline = gsl_spline_alloc(gsl_interp_cspline, n);
        //     gsl_spline_init(spline, x, y, n);

        //     // 将插值结果加到calibrated_points上
        //     it->fw_velocity_calibrated_points.clear();
        //     for (auto &point : it->fw_velocity_points)
        //     {
        //         FWVelocityPoint_t calibrated_point;
        //         calibrated_point.velocity = point.velocity;
        //         calibrated_point.fw_velocity = point.fw_velocity + gsl_spline_eval(spline, point.velocity, acc);
        //         it->fw_velocity_calibrated_points.push_back(calibrated_point);
        //     }

        //     gsl_spline_free(spline);
        //     gsl_interp_accel_free(acc);
        //     break;
        // }
        case CalibrationType_t::CALIBRATION_TYPE_LINER:
        {
            // 对误差进行线性回归，然后将回归结果加到原表上
            std::vector<FWVelocityPoint_t> delta_points_;

            for (const auto &point : fw_velocity_points_input)
            {
                FWVelocityPoint_t delta_point;
                delta_point.velocity = point.velocity;
                delta_point.fw_velocity = point.fw_velocity - calculateFWVelocity(dart_name, point.velocity);
                delta_points_.push_back(delta_point);
            }

            size_t n = delta_points_.size();
            if (n == 0)
            {
                break;
            }

            double sum_x = 0.0, sum_y = 0.0;
            double sum_xx = 0.0, sum_xy = 0.0;

            for (const auto &point : delta_points_)
            {
                sum_x += point.velocity;
                sum_y += point.fw_velocity;
                sum_xx += point.velocity * point.velocity;
                sum_xy += point.velocity * point.fw_velocity;
            }

            // 计算斜率 (m) 和截距 (b)
            double m_ = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
            double b_ = (sum_y - m_ * sum_x) / n;

            // 将回归结果加到calibrated_points上
            it->fw_velocity_calibrated_points.clear();
            for (auto &point : it->fw_velocity_points)
            {
                FWVelocityPoint_t calibrated_point;
                calibrated_point.velocity = point.velocity;
                calibrated_point.fw_velocity = point.fw_velocity + m_ * point.velocity + b_;
                it->fw_velocity_calibrated_points.push_back(calibrated_point);
            }
            break;
        }
        case CalibrationType_t::CALIBRATION_TYPE_OFFSET:
        {
            // 对校准表与原表的差值求平均值，然后将平均值加到原表上
            double sum = 0.0;
            for (const auto &point : fw_velocity_points_input)
            {
                sum += point.fw_velocity - calculateFWVelocity(dart_name, point.velocity);
            }

            double offset = sum / fw_velocity_points_input.size();

            // 将平均值加到calibrated_points上
            it->fw_velocity_calibrated_points.clear();
            for (auto &point : it->fw_velocity_points)
            {
                FWVelocityPoint_t calibrated_point;
                calibrated_point.velocity = point.velocity;
                calibrated_point.fw_velocity = point.fw_velocity + offset;
                it->fw_velocity_calibrated_points.push_back(calibrated_point);
            }
            break;
        }
        }
        notifyUpdate();
    }
}

void DartAlgorithm::DartDataBase::calibrateVelocity(std::string dart_name, std::vector<DistanceVelocityPoint_t> distance_velocity_points_input, CalibrationType_t calibration_type)
{
    auto it = std::find_if(dart_data_.begin(), dart_data_.end(), [&dart_name](const DartData_t &data)
                           { return data.name == dart_name; });

    if (it != dart_data_.end())
    {
        switch (calibration_type)
        {
        // case CalibrationType_t::CALIBRATION_TYPE_SPLINE:
        // {
        //     // 对校准表与原表的差值进行三次样条插值，然后将插值结果加到原表上
        //     size_t n = distance_velocity_points_input.size();
        //     double x[n], y[n];
        //     for (size_t i = 0; i < n; i++)
        //     {
        //         x[i] = distance_velocity_points_input[i].distance;
        //         y[i] = distance_velocity_points_input[i].velocity - calculateVelocity(dart_name, distance_velocity_points_input[i].distance);
        //     }

        //     gsl_interp_accel *acc = gsl_interp_accel_alloc();
        //     gsl_spline *spline = gsl_spline_alloc(gsl_interp_cspline, n);
        //     gsl_spline_init(spline, x, y, n);

        //     // 将插值结果加到calibrated_points上
        //     it->distance_calibrated_points.clear();
        //     for (auto &point : it->distance_velocity_points)
        //     {
        //         DistanceVelocityPoint_t calibrated_point;
        //         calibrated_point.distance = point.distance;
        //         calibrated_point.velocity = point.velocity + gsl_spline_eval(spline, point.distance, acc);
        //         it->distance_calibrated_points.push_back(calibrated_point);
        //     }

        //     gsl_spline_free(spline);
        //     gsl_interp_accel_free(acc);
        //     break;
        // }
        case CalibrationType_t::CALIBRATION_TYPE_LINER:
        {
            // 对误差进行线性回归，然后将回归结果加到原表上
            std::vector<DistanceVelocityPoint_t> delta_points_;

            for (const auto &point : distance_velocity_points_input)
            {
                DistanceVelocityPoint_t delta_point;
                delta_point.distance = point.distance;
                delta_point.velocity = point.velocity - calculateVelocity(dart_name, point.distance);
                delta_points_.push_back(delta_point);
            }

            size_t n = delta_points_.size();
            if (n == 0)
            {
                break;
            }

            double sum_x = 0.0, sum_y = 0.0;
            double sum_xx = 0.0, sum_xy = 0.0;

            for (const auto &point : delta_points_)
            {
                sum_x += point.distance;
                sum_y += point.velocity;
                sum_xx += point.distance * point.distance;
                sum_xy += point.distance * point.velocity;
            }

            // 计算斜率 (m) 和截距 (b)
            double m_ = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
            double b_ = (sum_y - m_ * sum_x) / n;

            // 将回归结果加到calibrated_points上
            it->distance_calibrated_points.clear();
            for (auto &point : it->distance_velocity_points)
            {
                DistanceVelocityPoint_t calibrated_point;
                calibrated_point.distance = point.distance;
                calibrated_point.velocity = point.velocity + m_ * point.distance + b_;
                it->distance_calibrated_points.push_back(calibrated_point);
            }
            break;
        }
        case CalibrationType_t::CALIBRATION_TYPE_OFFSET:
        {
            // 对校准表与原表的差值求平均值，然后将平均值加到原表上
            double sum = 0.0;
            for (const auto &point : distance_velocity_points_input)
            {
                sum += point.velocity - calculateVelocity(dart_name, point.distance);
            }

            double offset = sum / distance_velocity_points_input.size();

            // 将平均值加到calibrated_points上
            it->distance_calibrated_points.clear();
            for (auto &point : it->distance_velocity_points)
            {
                DistanceVelocityPoint_t calibrated_point;
                calibrated_point.distance = point.distance;
                calibrated_point.velocity = point.velocity + offset;
                it->distance_calibrated_points.push_back(calibrated_point);
            }
            break;
        }
        }
    }
}

bool DartAlgorithm::DartDataBase::saveCalibration(std::string file_name, std::string dart_name)
{
    auto it = std::find_if(dart_data_.begin(), dart_data_.end(), [&dart_name](const DartData_t &data)
                           { return data.name == dart_name; });

    if (it != dart_data_.end())
    {
        // 将校准结果保存到原vector
        it->distance_velocity_points = it->distance_calibrated_points;
        it->fw_velocity_points = it->fw_velocity_calibrated_points;

        // 调用一次保存
        saveToFile(file_name);

        return true;
    }
    return false;
}

void DartAlgorithm::DartDataBase::setYawOffset(std::string dart_name, int32_t offset)
{
    auto it = std::find_if(dart_data_.begin(), dart_data_.end(), [&dart_name](const DartData_t &data)
                           { return data.name == dart_name; });

    if (it != dart_data_.end())
    {
        it->target_yaw_angle_offset = offset;
        notifyUpdate();
    }
}

bool DartAlgorithm::DartDataBase::setDartName(std::string dart_name, std::string new_dart_name)
{
    auto it = std::find_if(dart_data_.begin(), dart_data_.end(), [&dart_name](const DartData_t &data)
                           { return data.name == dart_name; });

    if (it != dart_data_.end())
    {
        it->name = new_dart_name;
        notifyUpdate();
        return true;
    }

    return false;
}

int32_t DartAlgorithm::DartDataBase::getYawOffset(std::string dart_name)
{
    auto it = std::find_if(dart_data_.begin(), dart_data_.end(), [&dart_name](const DartData_t &data)
                           { return data.name == dart_name; });

    if (it != dart_data_.end())
    {
        return it->target_yaw_angle_offset;
    }

    return 0; // 默认偏移量
}

bool DartAlgorithm::DartDataBase::addDart(std::string dart_name)
{
    // check exist
    auto it = std::find_if(dart_data_.begin(), dart_data_.end(), [&dart_name](const DartData_t &data)
                           { return data.name == dart_name; });
    if (it != dart_data_.end())
    {
        return false;
    }
    DartData_t new_dart;
    new_dart.name = dart_name;
    dart_data_.push_back(new_dart);
    notifyUpdate();
    return true;
}

bool DartAlgorithm::DartDataBase::addDart(DartData_t dart_data)
{
    // check exist
    auto it = std::find_if(dart_data_.begin(), dart_data_.end(), [&dart_data](const DartData_t &data)
                           { return data.name == dart_data.name; });

    if (it != dart_data_.end())
    {
        // 将it里的点与dart_data里的点合并
        for (const auto &point : dart_data.distance_velocity_points)
        {
            it->distance_velocity_points.push_back(point);
        }

        for (const auto &point : dart_data.fw_velocity_points)
        {
            it->fw_velocity_points.push_back(point);
        }

        for (const auto &point : dart_data.distance_calibrated_points)
        {
            it->distance_calibrated_points.push_back(point);
        }

        for (const auto &point : dart_data.fw_velocity_calibrated_points)
        {
            it->fw_velocity_calibrated_points.push_back(point);
        }
        return true;
    }
    dart_data_.push_back(dart_data);
    notifyUpdate();
    return true;
}

bool DartAlgorithm::DartDataBase::removeDart(std::string dart_name)
{
    auto it = std::remove_if(dart_data_.begin(), dart_data_.end(), [&dart_name](const DartData_t &data)
                             { return data.name == dart_name; });

    if (it != dart_data_.end())
    {
        dart_data_.erase(it, dart_data_.end());
        notifyUpdate();
        return true;
    }

    return false;
}

DartAlgorithm::DartData_t DartAlgorithm::DartDataBase::getDartData(std::string dart_name)
{
    auto it = std::find_if(dart_data_.begin(), dart_data_.end(), [&dart_name](const DartData_t &data)
                           { return data.name == dart_name; });

    if (it != dart_data_.end())
    {
        return *it;
    }

    return DartData_t(); // 返回一个默认的 DartData
}

void DartAlgorithm::DartDataBase::addDistanceVelocityPoints(std::string dart_name, std::vector<DistanceVelocityPoint_t> distance_velocity_points)
{
    auto it = std::find_if(dart_data_.begin(), dart_data_.end(), [&dart_name](const DartData_t &data)
                           { return data.name == dart_name; });

    if (it != dart_data_.end())
    {
        for (const auto &point : distance_velocity_points)
        {
            it->distance_velocity_points.push_back(point);
        }
        notifyUpdate();
    }
}

void DartAlgorithm::DartDataBase::addFWVelocityPoints(std::string dart_name, std::vector<FWVelocityPoint_t> fw_velocity_points)
{
    auto it = std::find_if(dart_data_.begin(), dart_data_.end(), [&dart_name](const DartData_t &data)
                           { return data.name == dart_name; });

    if (it != dart_data_.end())
    {
        for (const auto &point : fw_velocity_points)
        {
            it->fw_velocity_points.push_back(point);
        }
        notifyUpdate();
    }
}

bool DartAlgorithm::DartDataBase::empty()
{
    return dart_data_.empty();
}

size_t DartAlgorithm::DartDataBase::size()
{
    return dart_data_.size();
}

bool DartAlgorithm::DartDataBase::contains(std::string dart_name)
{
    auto it = std::find_if(dart_data_.begin(), dart_data_.end(), [&dart_name](const DartData_t &data)
                           { return data.name == dart_name; });

    return it != dart_data_.end();
}

std::vector<std::string> DartAlgorithm::DartDataBase::getDartNames()
{
    std::vector<std::string> names;
    for (const auto &data : dart_data_)
    {
        names.push_back(data.name);
    }
    return names;
}

const double g = 9.8;
const double theta = 32 * M_PI / 180; // Pitch angle 弧度
const double h = 0.60;                // 出射点距离地面的高度 m

// y向上为正  dis为水平距离  单位:m
double calculateV(double dis, double y)
{
    double v = sqrt(g * dis * dis / ((dis * tan(theta) - y) * 2 * pow(cos(theta), 2)));
    return v;
}

// 计算最终落地时飞行的距离（水平） y表示打击目标相对于地面的高度
double DartAlgorithm::calculateDis(double dis, double y)
{
    double v = calculateV(dis, y);
    double vy = v * sin(theta);
    double t = (vy + sqrt(vy * vy + 2 * h * g)) / g;

    std::cout << "t = " << t << std::endl;

    double dis_far = v * cos(theta) * t;
    return dis_far;
}