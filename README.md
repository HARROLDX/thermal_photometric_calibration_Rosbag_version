# Online Photometric Calibration
针对热成像bag类型数据集的光度校准---读取bag文件版本
首先根据不同的bag数据集使用match包获得 correspendence.txt文件
目前的版本是NTU thermal数据集
在main文件中需要修改读取bag文件和txt文件的路径  话题  消息类型
mkdir build
cd build
cmake ..
make -j8
./main

