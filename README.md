# Online Photometric Calibration

## 简介

这是一个独立包，用于处理 rosbag 文件。针对热成像 bag 类型数据集的光度校准——读取 bag 文件版本。

## 使用说明

首先根据不同的 bag 数据集获得 `correspondence.txt` 文件，然后可以使用该包获得光度校准后的数据集。

目前的版本是 NTU thermal 数据集。

## 编译和运行

```bash
mkdir build
cd build
cmake ..
make -j8
./main
```

## 相关资源

- **原版代码**：请阅读原作者开源网址，需要获取对应的 txt 文件
  - https://github.com/mpdmanash/thermal_photometric_calibration

- **实时校准实现**：或参考下列代码实现实时校准
  - https://github.com/jpl-x/x_multi_agent/tree/main

## 配置说明

使用此包得到校准后的 bag 文件，如何设置话题（NTU数据集）请阅读 `/home/xiajiawei/TS-LIVO/src/config/NTU4DradLM.yaml`

本开源数据集提供一个correspondence.txt和校准后的NTU数据集
