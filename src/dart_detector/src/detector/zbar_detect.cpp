#include "detector/zbar_detect.h"

QRCodeDetector::QRCodeDetector()
{
    // 初始化zbar扫描器，仅启用二维码识别
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
}

QRCodeDetector::~QRCodeDetector() {}

void QRCodeDetector::setBinaryThreshold(double threshold)
{
    binary_threshold = threshold;
}

std::vector<std::string> QRCodeDetector::detect(const cv::Mat &inputImage)
{
    std::vector<std::string> results;

    if (inputImage.empty())
    {
        return results; // 返回空结果
    }

    // 转换为灰度图
    cv::Mat gray;
    cv::cvtColor(inputImage, gray, cv::COLOR_BGR2GRAY);

    // 二值化处理
    cv::Mat binary;
    cv::threshold(gray, binary, binary_threshold, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // 形态学开运算去除噪点
    cv::Mat morph;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binary, morph, cv::MORPH_OPEN, element);

    // 将处理后的图像转换为zbar所需的格式
    zbar::Image imagez(morph.cols, morph.rows, "Y800", morph.data, morph.cols * morph.rows);
    scanner.scan(imagez);

    // 遍历检测到的二维码
    for (zbar::Image::SymbolIterator symbol = imagez.symbol_begin(); symbol != imagez.symbol_end(); ++symbol)
    {
        if (symbol->get_type() == zbar::ZBAR_QRCODE)
        {
            results.push_back(symbol->get_data());
        }
    }

    return results;
}