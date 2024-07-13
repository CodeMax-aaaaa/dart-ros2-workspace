/**
 * @file Main.cpp
 * @author Simon Cahill (simonc@online.de)
 * @brief Contains the implementation of a test application using this library.
 * @version 0.1
 * @date 2020-07-02
 *
 * @copyright Copyright (c) 2020 Simon Cahill
 *
 *  Copyright 2020 Simon Cahill
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include <CanDriver.hpp>
#include <exceptions/CanException.hpp>
#include <exceptions/CanInitException.hpp>
#include <exceptions/InvalidSocketException.hpp>

using sockcanpp::CanDriver;
using sockcanpp::CanId;
using sockcanpp::CanMessage;
using sockcanpp::exceptions::CanException;
using sockcanpp::exceptions::CanInitException;
using sockcanpp::exceptions::InvalidSocketException;

using std::cerr;
using std::cout;
using std::endl;
using std::string;
// using std::chrono;

void printHelp(string);

void int32ToCanData(int32_t value, char *data) {
  data[0] = (value >> 24) & 0xFF;
  data[1] = (value >> 16) & 0xFF;
  data[2] = (value >> 8) & 0xFF;
  data[3] = value & 0xFF;
}

int main(int32_t argCount, char **argValues) {
  string canInterface;

  if (canInterface == "")
    canInterface = "can0";

  CanDriver *canDriver;
  try {
    canDriver = new CanDriver(canInterface, CAN_RAW);
  } catch (CanInitException &ex) {
    cerr << "An error occurred while initialising CanDriver: " << ex.what()
         << endl;
    delete canDriver;
    return -1;
  }


  // 开启一条线程，每隔100ms向0x301发布一条0000f77000000000的消息
  std::thread t1(
      [canDriver] {
        char data[8] = {0x00, 0x00, 0xbb, 0x70, 0x00, 0x00, 0x00, 0x00};
        int32_t speed = 3000;
        bool rotation = true;
        while (true) {
          // 将速度写入到can message中，注意can message的数据是大端模式，而int32_t是小端模式
          int32ToCanData(speed, data);
          CanMessage msg_s(0x301, data, 8);
          std::cout << "send message: ";
          for (auto byte : msg_s.getFrameData()) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int>(byte) << ' ';
          }
          cout << endl;
          canDriver->sendMessage(msg_s, true);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          if (rotation)
            speed += 1;
          else
            speed -= 1;
          if (abs(speed) == 54000) {
            rotation = !rotation;
            if (rotation)
              speed += 1;
            else
              speed -= 1;
          }
        }
      }
  );

// 开启thread t1
  t1.
      detach();

  while (true) {
    auto canMessages = canDriver->readMessage();

    auto msg = canMessages;
    if ((int32_t) msg.
        getCanId()
        == 0x901) {
      cout << "CAN ID: " << (int32_t) msg.
          getCanId()
           << " CAN data: ";
      for (
        auto byte
          : msg.
          getFrameData()
          ) {
        std::cout << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast
                      <int>(byte)
                  << ' ';
      }
      cout <<
           endl;
    }
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}