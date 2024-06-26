#include <chrono>
#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

std::shared_ptr<dai::Pipeline> createPipeline() {
    // Start defining a pipeline
    auto pipeline = std::make_shared<dai::Pipeline>();
    
    auto camMono_A = pipeline->create<dai::node::MonoCamera>();
    // camMono_A->setPreviewSize(300, 300);
    camMono_A->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camMono_A->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
    // camMono_A->setInterleaved(false);
    auto xoutMono_A = pipeline->create<dai::node::XLinkOut>();
    xoutMono_A->setStreamName("Mono_A");
    camMono_A->out.link(xoutMono_A->input);

    auto camRgb_B = pipeline->create<dai::node::ColorCamera>();
    camRgb_B->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    camRgb_B->setResolution(dai::ColorCameraProperties::SensorResolution::THE_800_P);
    // camRgb_B->setInterleaved(false);
    auto xoutRgb_B = pipeline->create<dai::node::XLinkOut>();
    xoutRgb_B->setStreamName("Rgb_B");
    camRgb_B->video.link(xoutRgb_B->input);

    auto camRgb_C = pipeline->create<dai::node::ColorCamera>();
    camRgb_C->setBoardSocket(dai::CameraBoardSocket::CAM_C);
    camRgb_C->setResolution(dai::ColorCameraProperties::SensorResolution::THE_800_P);
    // camRgb_C->setInterleaved(false);
    auto xoutRgb_C = pipeline->create<dai::node::XLinkOut>();
    xoutRgb_C->setStreamName("Rgb_C");
    camRgb_C->video.link(xoutRgb_C->input);

    auto camMono_D = pipeline->create<dai::node::MonoCamera>();
    camMono_D->setBoardSocket(dai::CameraBoardSocket::CAM_D);
    camMono_D->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
    // camMono_D->setInterleaved(false);
    auto xoutMono_D = pipeline->create<dai::node::XLinkOut>();
    xoutMono_D->setStreamName("Mono_D");
    camMono_D->out.link(xoutMono_D->input);

    return pipeline;
}

int main(int argc, char** argv) {
    auto deviceInfoVec = dai::Device::getAllAvailableDevices();
    const auto usbSpeed = dai::UsbSpeed::SUPER;
    auto openVinoVersion = dai::OpenVINO::Version::VERSION_2021_4;

    std::map<std::string, std::shared_ptr<dai::DataOutputQueue>> qCamMap;
    std::vector<std::shared_ptr<dai::Device>> devices;

    for(auto& deviceInfo : deviceInfoVec) {
        auto device = std::make_shared<dai::Device>(openVinoVersion, deviceInfo, usbSpeed);
        devices.push_back(device);
        std::cout << "===Connected to " << deviceInfo.getMxId() << std::endl;
        auto mxId = device->getMxId();
        auto cameras = device->getConnectedCameras();
        auto usbSpeed = device->getUsbSpeed();
        auto eepromData = device->readCalibration2().getEepromData();
        std::cout << "   >>> MXID:" << mxId << std::endl;
        std::cout << "   >>> Num of cameras:" << cameras.size() << std::endl;
        std::cout << "   >>> USB speed:" << usbSpeed << std::endl;
        if(eepromData.boardName != "") {
            std::cout << "   >>> Board name:" << eepromData.boardName << std::endl;
        }
        if(eepromData.productName != "") {
            std::cout << "   >>> Product name:" << eepromData.productName << std::endl;
        }
        // device->config.board.gpio[6] = dai::BoardConfig::GPIO(dai::BoardConfig::GPIO::OUTPUT,dai::BoardConfig::GPIO::Level::HIGH);

        auto pipeline = createPipeline();
        device->startPipeline(*pipeline);

        auto qMono_A = device->getOutputQueue("Mono_A", 2, false);
        qCamMap.insert({"cam_a", qMono_A});

        auto qRgb_B = device->getOutputQueue("Rgb_B", 2, false);
        qCamMap.insert({"cam_b", qRgb_B});

        auto qRgb_C = device->getOutputQueue("Rgb_C", 2, false);
        qCamMap.insert({"cam_c", qRgb_C});

        auto qMono_D = device->getOutputQueue("Mono_D", 2, false);
        qCamMap.insert({"cam_d", qMono_D});
        
    }
    while(true) {
        for(auto& element : qCamMap) {
            auto qCam = element.second;
            auto streamName = element.first;
            auto inCam = qCam->tryGet<dai::ImgFrame>();
            if(inCam != nullptr) {
                cv::imshow(streamName, inCam->getCvFrame());
            }
        }
        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
