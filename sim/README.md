# PC 模拟器编译与运行说明

本目录 (`sim/`) 包含让项目在 PC 上运行的所有必要文件。

## 文件说明

| 文件 | 说明 |
|---|---|
| `CMakeLists.txt` | PC 端 CMake 构建脚本 |
| `lv_conf.h` | 转发头文件（`#include "lv_conf_pc.h"`） |
| `lv_conf_pc.h` | PC 端 LVGL 配置（32 位色深、标准 malloc、SDL2） |
| `main_pc.cpp` | PC 端程序入口，替代嵌入式 `src/main.cpp` |

## 环境依赖

### macOS

```bash
brew install cmake sdl2
```

### Ubuntu / Debian

```bash
sudo apt install cmake libsdl2-dev build-essential
```

## 构建步骤

在项目根目录执行：

```bash
# 配置（只需执行一次）
cmake -B build_sim sim/

# 编译
cmake --build build_sim -j$(nproc)

# 运行
./build_sim/sim
```

## 快捷键

| 按键 | 功能 |
|---|---|
| `R` | 切换屏幕旋转（0° / 90° / 180° / 270°） |
| `T` | 切换深色 / 浅色主题 |
| `Q` / `ESC` | 退出 |
| 鼠标左键 | 触摸交互 |

## 与嵌入式端的差异

| 功能 | 嵌入式端 | PC 端 |
|---|---|---|
| 显示输出 | CO5300 AMOLED (QSPI) | SDL2 窗口 |
| 触摸输入 | CST9217 (I2C) | 鼠标左键 |
| 时钟 | PCF85063 RTC | 系统时间 (`time.h`) |
| 文件系统 | SD 卡 (`/sdcard`) | 本地 `./assets/` |
| 内存分配 | 自定义 PSRAM 分配器 | 标准 `malloc` |
| 音频 | ES8311 I2S | 跳过（stub） |
| PMU / 电源键 | AXP2101 | 键盘 R 键模拟 |
| 颜色深度 | RGB565 (16-bit) | XRGB8888 (32-bit) |

## 资源文件

构建后，字体和图片会自动复制到 `build_sim/assets/`：
- `build_sim/assets/fonts/`：LVGL binfont 二进制字体文件
- `build_sim/assets/images/`：PNG 图标资源

如果手动修改了资源文件，重新运行 `cmake --build build_sim` 即可自动同步。

## 清理

```bash
rm -rf build_sim
```
