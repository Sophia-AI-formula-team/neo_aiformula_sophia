# Road Detection
This package provides a road detector for AIFormula.
The active YOLOP runtime code is vendored inside this package at
`road_detector/yolop`; do not use a separate top-level YOLOP checkout.

![image](https://github.com/user-attachments/assets/078b5c81-7e08-4ad7-9dc2-9ec09a537de5)

## pytorch install

### requirement packages
* requiement install 

    ```
    sudo apt-get -y install \
    autoconf bc build-essential g++-8 gcc-8 clang-8 lld-8 gettext-base gfortran-8 iputils-ping libbz2-dev libc++-dev libcgal-dev libffi-dev libfreetype6-dev libhdf5-dev libjpeg-dev liblzma-dev libncurses5-dev libncursesw5-dev libpng-dev libreadline-dev libssl-dev libsqlite3-dev libxml2-dev libxslt-dev locales moreutils openssl python-openssl rsync scons python3-pip libopenblas-dev
    ```

### pytorch install
* Specifying the Torch Version
Note: you choice pytorch version link 1.2


    ```
    export TORCH_INSTALL=https://developer.download.nvidia.com/compute/redist/jp/v502/pytorch/torch-1.13.0a0+d0d6b1f2.nv22.10-cp38-cp38-linux_aarch64.whl
    export "LD_LIBRARY_PATH=/usr/lib/llvm-8/lib:$LD_LIBRARY_PATH"
    ```

* install

    ```
    python3 -m pip install --upgrade protobuf
    python3 -m pip install --no-cache $TORCH_INSTALL
    ```

### torchvision

This package no longer requires torchvision at runtime; image tensor conversion and normalization are implemented directly in `road_detector.py`.
### Source

#### 1.NVIDIA distribute wheel file
https://developer.download.nvidia.com/compute/redist/jp/

#### 2. orin nvidia version
https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048
#### 3. torch and torchvision version
https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-pytorch


## Running Example:
Road detection is launched through the `launchers` package:
```
ros2 launch launchers auto_yolop_launch.py
```
