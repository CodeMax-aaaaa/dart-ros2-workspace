#include <dart_algorithm.hpp>
#include <dart_calibration.hpp>
#include <iostream>
#include <string>
#include <sstream>

using namespace DartAlgorithm;
using namespace std;

DartDataBase dart_db;

void clearInput()
{
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

vector<DistanceVelocityPoint_t> getDistanceVelocityPointsFromCin()
{
    vector<DistanceVelocityPoint_t> distance_velocity_points;
    clearInput();
    while (true)
    {
        stringstream ss;
        DistanceVelocityPoint_t point;
        string cin_buffer;
        getline(cin, cin_buffer);
        // EOF
        if (std::cin.eof())
        {
            return distance_velocity_points;
        }

        if (cin_buffer == "q")
        {
            break;
        }
        else
        {
            // 用空格分割字符串
            ss << cin_buffer;
            ss >> point.distance >> point.velocity;
            cout << "distance: " << point.distance << " velocity: " << point.velocity << endl;
            distance_velocity_points.push_back(point);
            ss.clear();
        }
    }

    return distance_velocity_points;
}

vector<FWVelocityPoint_t> getFWVelocityPointsFromCin()
{
    vector<FWVelocityPoint_t> fw_velocity_points;
    clearInput();
    while (true)
    {
        std::string cin_buffer;
        stringstream ss;
        FWVelocityPoint_t point;
        getline(cin, cin_buffer);
        // EOF
        if (std::cin.eof())
        {
            return fw_velocity_points;
        }

        if (cin_buffer == "q")
        {
            break;
        }
        else
        {
            ss << cin_buffer;
            ss >> point.fw_velocity >> point.velocity;
            fw_velocity_points.push_back(point);
        }
    }

    return fw_velocity_points;
}

void addDart()
{
    DartData_t new_dart;
    cout << "Please input dart name: ";
    cin >> new_dart.name;

    // EOF
    if (std::cin.eof())
    {
        return;
    }

    cout << "Please input dart distance-velocity pairs. First Distance next velocity. Enter 'q' to finish.\n";
    new_dart.distance_velocity_points = getDistanceVelocityPointsFromCin();
    // EOF
    if (std::cin.eof())
    {
        return;
    }
    cout << "Please input dart fricwheel velocity-velocity pairs. First fricwheel velocity next velocity. Enter 'q' to finish.\n";
    new_dart.fw_velocity_points = getFWVelocityPointsFromCin();
    // EOF
    if (std::cin.eof())
    {
        return;
    }

    dart_db.addDart(new_dart);
    dart_db.saveToFile(string(YAML_PATH) + string("dart_db.yaml"));
    cout << "Dart data saved" << endl;
}

void addDistanceVelocityPoint(string dart_name)
{
    cout << "Please input dart distance-velocity pairs. First Distance next velocity. Enter 'q' to finish.\n";
    vector<DistanceVelocityPoint_t> distance_velocity_points = getDistanceVelocityPointsFromCin();

    dart_db.addDistanceVelocityPoints(dart_name, distance_velocity_points);
    dart_db.saveToFile(string(YAML_PATH) + string("dart_db.yaml"));
    cout << "Dart data saved" << endl;
}

void addFWVelocityPoint(string dart_name)
{
    cout << "Please input dart fricwheel velocity-velocity pairs. First fricwheel velocity next velocity. Enter 'q' to finish.\n";
    vector<FWVelocityPoint_t> fw_velocity_points = getFWVelocityPointsFromCin();

    dart_db.addFWVelocityPoints(dart_name, fw_velocity_points);
    dart_db.saveToFile(string(YAML_PATH) + string("dart_db.yaml"));
    cout << "Dart data saved" << endl;
}

void setDistanceVelocityPointCalibration(string dart_name, CalibrationType_t calibration_type)
{
    cout << "====DistanceVelocity Calibration====\n";
    cout << "Please input dart distance-velocity pairs. First Distance next velocity. Enter 'q' to finish.\n";
    vector<DistanceVelocityPoint_t> distance_velocity_calibration_points = getDistanceVelocityPointsFromCin();

    dart_db.calibrateVelocity(dart_name, distance_velocity_calibration_points, calibration_type);
    dart_db.saveToFile(string(YAML_PATH) + string("dart_db.yaml"));
    cout << "Calibration completed." << endl;
}

void setFWVelocityPointCalibration(string dart_name, CalibrationType_t calibration_type)
{
    cout << "====FWVelocity Calibration====\n";
    cout << "Please input dart fricwheel velocity-velocity pairs. First fricwheel velocity next velocity. Enter 'q' to finish.\n";
    vector<FWVelocityPoint_t> fw_velocity_calibration_points = getFWVelocityPointsFromCin();

    dart_db.calibrateFWVelocity(dart_name, fw_velocity_calibration_points, calibration_type);
    dart_db.saveToFile(string(YAML_PATH) + string("dart_db.yaml"));
    cout << "Calibration completed." << endl;
}

void deleteDart(string dart_name)
{
    dart_db.removeDart(dart_name);
    dart_db.saveToFile(string(YAML_PATH) + string("dart_db.yaml"));
    cout << "Dart deleted." << endl;
}

void calculateVelocity(string dart_name, double distance)
{
    double velocity = dart_db.calculateVelocity(dart_name, distance);
    cout << "Calculated velocity: " << velocity << endl;
}

void calculateFWVelocityFromVelocity(string dart_name, double velocity)
{
    cout << "Calculated velocity: " << dart_db.calculateFWVelocity(dart_name, velocity) << endl;
}

void calculateFWVelocityFromDistance(string dart_name, double distance)
{
    cout << "Calculated velocity: " << dart_db.calculateFWVelocity(dart_name, dart_db.calculateVelocity(dart_name, distance)) << endl;
}

void dumpDartData(string dart_name)
{
    DartData_t dart_data = dart_db.getDartData(dart_name);
    cout << "Dart name: " << dart_data.name << endl;
    cout << "Yaw offset: " << dart_data.target_yaw_angle_offset << endl;
    cout << "Distance-Velocity points:\n";
    for (auto &i : dart_data.distance_velocity_points)
    {
        cout << i.distance << " " << i.velocity << endl;
    }
    cout << "Fricwheel Velocity-Velocity points:\n";
    for (auto &i : dart_data.fw_velocity_points)
    {
        cout << i.fw_velocity << " " << i.velocity << endl;
    }

    cout << "Calibrated Distance-Velocity points:\n";
    for (auto &i : dart_data.distance_calibrated_points)
    {
        cout << i.distance << " " << i.velocity << endl;
    }
    cout << "Calibrated Fricwheel Velocity-Velocity points:\n";
    for (auto &i : dart_data.fw_velocity_calibrated_points)
    {
        cout << i.fw_velocity << " " << i.velocity << endl;
    }
}

void dartMenu(string index)
{
    while (true)
    {
        std::cout << "\nDart Menu for " << index << ":\n";
        std::cout << "1. Delete dart\n";
        std::cout << "2. Set yaw offset\n";
        std::cout << "3. Add distance-velocity point\n";
        std::cout << "4. Add fricwheel velocity-velocity point\n";
        std::cout << "5. Calibrate distance-velocity points\n";
        std::cout << "6. Calibrate fricwheel velocity-velocity points\n";
        std::cout << "7. Calculate velocity\n";
        std::cout << "8. Calculate fricwheel velocity from velocity\n";
        std::cout << "9. Calculate fricwheel velocity from distance\n";
        std::cout << "10. Dump dart data\n";
        std::cout << "11. Calculate target distance from position\n";
        std::cout << "12. Save Calibration Data\n";
        std::cout << "13. Exit\n";
        std::cout << "Enter your choice: ";
        int choice;
        std::cin >> choice;

        if (std::cin.fail())
        {
            // EOF
            if (std::cin.eof())
            {
                return;
            }

            clearInput();
            std::cout << "Invalid input. Please enter a number.\n";
            continue;
        }

        switch (choice)
        {
        case 1:
            deleteDart(index);
            return;
        case 2:
        {
            cout << "Please input offset: ";
            int offset;
            cin >> offset;
            dart_db.setYawOffset(index, offset);
            dart_db.saveToFile(string(YAML_PATH) + string("dart_db.yaml"));
            cout << "Calibration completed." << endl;
            break;
        }
        case 3:
            addDistanceVelocityPoint(index);
            break;
        case 4:
            addFWVelocityPoint(index);
            break;
        case 5:
            setDistanceVelocityPointCalibration(index, CALIBRATION_TYPE_OFFSET);
            break;
        case 6:
            setFWVelocityPointCalibration(index, CALIBRATION_TYPE_OFFSET);
            break;
        case 7:
        {
            cout << "Please input distance: ";
            double distance;
            cin >> distance;
            calculateVelocity(index, distance);
            break;
        }
        case 8:
        {
            cout << "Please input velocity: ";
            double velocity;
            cin >> velocity;
            calculateFWVelocityFromVelocity(index, velocity);
            break;
        }
        case 9:
        {
            cout << "Please input distance: ";
            double distance;
            cin >> distance;
            calculateFWVelocityFromDistance(index, distance);
            break;
        }
        case 10:
            dumpDartData(index);
            break;

        case 11:
        {
            cout << "Please input position in x and y: ";
            double x, y;
            cin >> x >> y;
            cout << "Calculated distance: " << calculateDis(x, y) << endl;
            break;
        }
        case 12:
        {
            dart_db.saveCalibration(string(YAML_PATH) + string("dart_db.yaml"), index);
            cout << "Calibration data saved." << endl;
            break;
        }
        case 13:
            return;
        }
    }
}

void mainMenu()
{
    while (true)
    {
        std::cout << "\nMain Menu:\n";
        std::cout << "1. Select dart\n";
        std::cout << "2. Add new dart\n";
        std::cout << "3. Exit\n";
        std::cout << "Enter your choice: ";
        int choice;
        std::cin >> choice;

        if (std::cin.fail())
        {
            // EOF
            if (std::cin.eof())
            {
                return;
            }
            clearInput();
            std::cout << "Invalid input. Please enter a number.\n";
            continue;
        }

        switch (choice)
        {
        case 1:
            if (dart_db.empty())
            {
                std::cout << "No darts available. Please add a new dart first.\n";
            }
            else
            {
                std::cout << "Available darts:\n";
                for (auto &i : dart_db.getDartNames())
                {
                    std::cout << i << std::endl;
                }

                std::cout << "Select dart name: ";
                string dart_choice;
                std::cin >> dart_choice;
                if (dart_db.contains(dart_choice))
                {
                    dartMenu(dart_choice);
                }
                else
                {
                    std::cout << "Dart not found. Please try again.\n";
                }
            }
            break;
        case 2:
            addDart();
            break;
        case 3:
            std::cout << "Exiting...\n";
            return;
        default:
            std::cout << "Invalid choice. Please try again.\n";
        }
    }
}

int main()
{
    // 读取配置文件
    dart_db.loadFromFile(string(YAML_PATH) + string("dart_db.yaml"));
    // 主菜单
    mainMenu();
    return 0;
}