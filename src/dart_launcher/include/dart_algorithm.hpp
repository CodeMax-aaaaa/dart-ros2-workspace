#ifndef DART_ALGORITHM_H
#define DART_ALGORITHM_H

#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <functional>

namespace DartAlgorithm
{
    typedef struct DistanceVelocityPoint_t
    {
        double distance = 0.0;
        double velocity = 0.0;
    } DistanceVelocityPoint_t;

    typedef struct FWVelocityPoint_t
    {
        int fw_velocity = 0;
        double velocity = 0.0;
    } FWVelocityPoint_t;

    typedef struct DartData_t
    {
        std::string name = "";
        int32_t target_yaw_angle_offset = 0;
        int32_t target_fw_velocity_offset = 0;

        std::vector<DistanceVelocityPoint_t> distance_velocity_points;
        std::vector<FWVelocityPoint_t> fw_velocity_points;
        // 摩擦轮速度校准保留
        std::vector<DistanceVelocityPoint_t> distance_calibrated_points;
        std::vector<FWVelocityPoint_t> fw_velocity_calibrated_points;
    } DartData_t;

    enum CalibrationType_t
    {
        CALIBRATION_TYPE_SPLINE = 0,
        CALIBRATION_TYPE_LINER = 1,
        CALIBRATION_TYPE_OFFSET = 2
    };

    class DartDataBase
    {
    private:
        std::vector<DartData_t> dart_data_;
        std::function<void(void *)> dart_data_update_callback_;
        void notifyUpdate();
        std::string file_name_;

    public:
        DartDataBase(); // do nothing
        DartDataBase(YAML::Node yaml_node);
        // 使用文件名加载
        DartDataBase(std::string file_name);
        void loadFromYaml(YAML::Node yaml_node);
        void saveToYaml(YAML::Node &yaml_node);
        void loadFromFile(std::string file_name);
        void saveToFile(std::string file_name);
        void saveToFile();
        void setDartDataUpdateCallback(std::function<void(void *)> callback);

        bool empty();
        size_t size();
        bool contains(std::string dart_name);

        /**
         * @brief 已知目标出射速度，根据飞镖计算摩擦轮速度
         * @param dart_name 飞镖名
         * @param distance 距离
         */
        int calculateFWVelocity(std::string dart_name, double velocity);
        /**
         * @brief 已知目标距离(斜边距离，单位m)，根据飞镖计算目标出射速度
         * @param dart_name 飞镖名
         * @param distance 距离
         */
        double calculateVelocity(std::string dart_name, double distance);
        /**
         * @brief 线性校准摩擦轮速度-初速度曲线
         * @param dart_name 飞镖名
         * @param distance_velocity_points 距离-速度校准点集
         */
        void calibrateFWVelocity(std::string dart_name, std::vector<FWVelocityPoint_t> fw_velocity_points_input, CalibrationType_t calibration_type);
        /**
         * @brief 线性校准目标出射速度-距离曲线
         * @param dart_name 飞镖名
         * @param fw_velocity_points 摩擦轮速度-目标出射速度校准点集
         */
        void calibrateVelocity(std::string dart_name, std::vector<DistanceVelocityPoint_t> distance_velocity_points_input, CalibrationType_t calibration_type);
        /**
         * @brief 保存校准结果
         * @param file_name 文件名
         * @param dart_name 飞镖名
         * @param type 校准类型
         * @param points 校准点集
         * @return 保存成功返回true
         */
        bool saveCalibration(std::string file_name, std::string dart_name);
        /**
         * @brief 修改Yaw Offset
         * @param dart_name 飞镖名
         * @param offset 偏移量
         */
        void setYawOffset(std::string dart_name, int32_t offset);
        /**
         * @brief 修改名字
         * @param dart_name 飞镖名
         * @param new_dart_name 新名字
         * @return 修改成功返回true
         */
        bool setDartName(std::string dart_name, std::string new_dart_name);
        /**
         * @brief 获取Yaw Offset
         * @param dart_name 飞镖名
         * @return 偏移量
         */
        int32_t getYawOffset(std::string dart_name);
        /**
         * @brief 新建飞镖
         * @param dart_name 飞镖名
         * @return 新建成功返回true
         */
        bool addDart(std::string dart_name);
        bool addDart(DartData_t dart_data);
        void addDistanceVelocityPoints(std::string dart_name, std::vector<DistanceVelocityPoint_t> distance_velocity_points);
        void addFWVelocityPoints(std::string dart_name, std::vector<FWVelocityPoint_t> fw_velocity_points);
        /**
         * @brief 删除飞镖
         * @param dart_name 飞镖名
         * @return 删除成功返回true
         */
        bool removeDart(std::string dart_name);
        /**
         * @brief 获取飞镖数据
         * @param dart_name 飞镖名
         * @return 飞镖数据
         */
        DartData_t getDartData(std::string dart_name);
        /**
         * @brief 获取全部飞镖名称
         * @return 飞镖名称集
         */
        std::vector<std::string> getDartNames();
    };

    /**
     * @brief 计算目标落地距离
     * @param dis 斜边距离
     * @param y 高度差
     */
    double calculateDis(double dis, double y);
};

#endif // DART_ALGORITHM_H