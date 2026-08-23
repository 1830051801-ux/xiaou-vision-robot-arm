# 公开发布范围

这个文件把“能够放进 GitHub 的源码”与本机运行目录、部署包和内部数据分开。它是发布检查清单，不代表工作区中的每个文件都适合提交。

## 可以保留在源码提交中

- raspberry_pi/robot_ai、tools、tests 和可复现的离线仿真代码；
- STM32F407/Keil 源码、工程文件和随附的第三方许可证；
- ROS 2 描述、接口、规划预览和不启动硬件的回放工具；
- 去除本地绝对路径、真实设备参数和内部产品信息的验证证据；
- 架构、协议、验证边界、许可证和第三方组件说明。

## 提交前逐项审查

- ONNX、INT8 模型和策略权重：确认来源、训练数据、大小和再分发权限；
- demo.mp4、图片和网格：确认没有工厂现场、内部产品或个人信息；
- runtime 下的验证 JSON：确认没有绝对路径、串口名、IP、密钥、真实标定值；
- STM32 驱动和 RT-Thread：保留上游版权头和许可证，不把第三方代码写成原创；
- 大于 10 MiB 的模型、视频和压缩包：优先使用 GitHub Release 或 Git LFS。

## 禁止发布

- 荣耀、富士康或其他公司内部的照片、日志、方案、模型、源码和产品标识；
- raspberry_pi/config.env、真实 API key、SSH 配置、设备 IP 和本机路径；
- 未经测量的零位、关节限位、桌面高度、TCP 参数和生产线标定文件；
- 把离线仿真、协议回放或数值收敛写成真实机械臂验收结果；
- 能够直接把普通用户加入免密 sudo 或绕过系统权限的脚本。

## 发布前检查

~~~powershell
py -3.13 -B scripts\verify_release.py
py -3.13 -B -m unittest discover -s raspberry_pi\tests -q
git diff --check
git ls-files | rg '(^|/)(config\.env|.*\.pyc|MDK-ARM/Objects|MDK-ARM/Listings|runtime/deployment)(/|$)'
~~~

真实上机必须另建现场记录，包含设备身份、固件哈希、标定版本、急停检查、反馈新鲜度和操作者确认。GitHub 提交本身不等于硬件验收。
