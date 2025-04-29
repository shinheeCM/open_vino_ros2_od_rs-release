# ObjectDetection
Shinhee ObjectDetection

```
    wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
    sudo apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB

    # Ubuntu 22
    echo "deb https://apt.repos.intel.com/openvino/2024 ubuntu22 main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2024.list

    # Step 4: Update the list of packages via the update command
    sudo apt update

    # Step 5: Verify that the APT repository is properly set up. Run the apt-cache command to see a list of all available OpenVINO packages and components
    sudo apt-cache search openvino

    # Step 6: Install OpenVINO Runtime
    sudo apt install openvino-2024.6.0

```