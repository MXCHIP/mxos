MXOS
====

MXCHIP Open Stack

### 特性
* 为嵌入式设备设计的物联网开发软件平台
* 基于实时操作系统设计
* 支持各种微控制器平台
* 无线网络接入方案：Wi-Fi，蓝牙，LoRa，蜂窝网络
* 内置云服务接入中间件和各种示例代码
* 优秀的低功耗控制功能
* 为物联网产品量产设计的应用程序框架
* 为应用开发提供的大量编程工具

### 目录结构
* board：板级定制代码
* include：MXOS核心功能接口
* libraries：MXOS软件中间件，应用支持代码等
* makefiles：MiCoder 编译工具链文件
* MXOS：MXOS核心库,二进制文件和相关系统代码
* platform：基于不同开发环境，硬件平台的特性文件
* resources：模组功能所需要的资源文件，如射频固件等
* sub_build：booloader 和 JTAG 下载的 RAM code
* template：工程模板文件

Notice: Internal use only

MXOS
====
MXCHIP Open Stack

### Features
* Software development platform designed for embedded devices
* Based on a real time operation system
* Support abundant MCUs
* Wi-Fi, bluetooth connectivity total solution, LoRa, Cellular network
* Build-in protocols for cloud service
* State-of-the-art low power management
* Application framework for I.O.T product
* Rich tools and mobile APP to accelerate development

### Contents:
* board: BSP file, Hardware resources and configurations on different boards.
* include: MXOS core APIs.
* libraries: Open source software libraries.
* makefiles：MiCoder compile toolchain files.
* MXOS: MXOS core libraries, binary file and system codes.
* platform: codes based in different IDE and hardware.
* resources：resources, like RF firmware, needed by modules.
* sub_build：booloader and RAM code for JTAG flasher.
* template：template files for new project.
