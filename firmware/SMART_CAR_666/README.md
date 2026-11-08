# SMART CAR 666 固件

目标主控为 STM32H743VIT6。VS Code 和 Keil 共用 `Core/` 与 `Drivers/`。电机、编码器、PID 和串口代码在 `Core/APP/`。

## 编译

- **VS Code**　打开本目录，用 STM32 扩展和 CMake Tools 选 `Debug` 或 `Release` 预设再编译。命令行编译需要将 CMake、Ninja 和 Arm GCC 加入 `PATH`，然后执行以下命令。

  ```sh
  cmake --preset Debug
  cmake --build --preset Debug
  ```

  固件在 `build/Debug/SMART_CAR_666.elf`。

- **Keil**　打开 `MDK-ARM/SMART CAR 666.uvprojx` 并编译。工程使用 Arm Compiler 5.06 update 7 和 `Keil.STM32H7xx_DFP` 4.1.3。

## 更新硬件配置

用 STM32CubeMX 打开 `SMART_CAR_666.ioc`，固件包版本为 STM32CubeH7 V1.13.0。重新生成后核对 `STM32H743xx_FLASH.ld` 和 `.map`，确保 USART3 的 `Serial_TxBuffer` 仍在 DMA1 可访问的 AXI SRAM。新增自写 `.c` 时，在 CMake 和 Keil 中都加入该文件。

首次上板验证时先断开电机供电。现有程序启动约 300 ms 后会输出非零 PWM，USART3 遥测使用 9600 波特率。
