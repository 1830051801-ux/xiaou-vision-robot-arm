# Third-Party Notices

The repository combines original XiaoU application and control code with
vendor and open-source components. The following paths are retained with their
upstream notices and must not be relicensed as project-original code.

| Component | Location | Notice or license |
| --- | --- | --- |
| ARM CMSIS | stm32_keil/BasicSetting_DaRanRobot/Drivers/CMSIS/ | Apache License 2.0 in the component LICENSE.txt |
| STM32F4 HAL | stm32_keil/BasicSetting_DaRanRobot/Drivers/STM32F4xx_HAL_Driver/ | BSD-3-Clause notice in the component LICENSE.txt; vendor copyright headers also apply |
| STM32 device files | stm32_keil/BasicSetting_DaRanRobot/Drivers/CMSIS/Device/ST/STM32F4xx/ | See the component LICENSE.txt and source headers |
| RT-Thread sources | stm32_keil/BasicSetting_DaRanRobot/RTT_RTOS/ | Apache-2.0 SPDX headers are retained where present; verify the upstream version before redistribution |
| Python dependencies | raspberry_pi/requirements*.txt | Each dependency keeps its own upstream license and notice |
| ROS 2 / MoveIt dependencies | raspberry_pi/ros2_ws/ | System packages are not redistributed by this repository |

Before a public release, audit every bundled model, mesh, font, image, video
and compressed deployment archive. Keep a source URL, version, license and
redistribution decision in the release record. Do not publish internal
company data or product-specific factory material.
