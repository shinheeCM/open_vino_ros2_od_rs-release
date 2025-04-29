#!/bin/bash
sudo mkdir /opt/intel

# for ubuntu 22.04
curl -L https://storage.openvinotoolkit.org/repositories/openvino/packages/2025.1/linux/openvino_toolkit_ubuntu22_2025.1.0.18503.6fec06580ab_x86_64.tgz --output openvino_2025.1.0.tgz
tar -xf openvino_2025.1.0.tgz
sudo mv openvino_toolkit_ubuntu22_2025.1.0.18503.6fec06580ab_x86_64 /opt/intel/openvino_2025.1.0


cd /opt/intel/openvino_2025.1.0
sudo -E ./install_dependencies/install_openvino_dependencies.sh

# for python3 api
cd /opt/intel/openvino_2025.1.0
python3 -m pip install -r ./python/requirements.txt

echo 'source /opt/intel/openvino_2025/setupvars.sh' >> ~/.bashrc

source ~/.bashrc


pip install ultralytics

cd ~/ObjectDetection/open_vino/YOLOv8-OpenVINO-CPP-Inference
mkdir build
cd build
cmake ..
make


cd ~/ObjectDetection/open_vino/YOLOv8-OpenVINO-LIVE-Inference
mkdir build
cd build
cmake ..
make