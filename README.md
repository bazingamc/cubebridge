# cubebridge

ESP32 / ESP32-S3 蓝牙双角色网关：BLE Central 连接实体智能魔方（魔域 MHC / 魔域 WCU_MY3 / 奇艺 QY-QYSC），把转动、状态与电量实时转换成 GAN Gen2 协议，再以 BLE Peripheral 身份作为虚拟魔方供支持GAN协议的App 连接。

## 功能

- BLE 双角色：Central 连接实体魔方 + Peripheral 被手机 App 连接（NimBLE）
- 实体魔方转动 / 状态 / 电量实时转发（打乱还原全同步），源事件携带真实时间戳
- 支持实体魔方：魔域 MHC（明文协议）、魔域 WCU_MY3（AES-CBC MAC 派生密钥）、奇艺 QY-QYSC（AES-ECB + CRC16）
- BOOT 按键：短按（<1s）强制重扫；长按（≥3s）清除配对（红色三短闪确认）
- LED 状态机：蓝慢闪=扫描 黄快闪=连接 紫2Hz=同步 绿微光短闪=运行（ESP32 单色板为常亮） 橙3Hz=重连
- UART 只读；开启 `CONFIG_VCUBE_TEST_INPUT` 后可用 UART 命令 / 6 键 GPIO 本地离线测试（不转发 BLE）

## 硬件

| 项目 | ESP32 | ESP32-S3 |
| --- | --- | --- |
| 六路测试按键 GPIO（仅开启测试输入时） | 32、33、25、26、27、14 | 4、5、6、7、15、16 |
| LED | GPIO2 单色 | GPIO48 WS2812 RGB |
| BOOT 按键 | GPIO0 | GPIO0 |
| Flash | 4MB | 16MB |

## 编译与烧录

需要 ESP-IDF 6.0.1（ESP32 与 ESP32-S3 均支持）。

```powershell
idf.py set-target esp32s3   # 或 esp32
idf.py build flash monitor
```

分区表：NVS `0x9000`、PHY `0xf000`、factory 应用 `0x10000`（1MiB），无 OTA。

## 串口命令（115200，仅 `CONFIG_VCUBE_TEST_INPUT` 开启时可用）

```
U R F D L B      单步顺转     U' R2 F' ...   标准记号
scramble         随机打乱 20 步
bat <0-100>      设置电量
state / ?        打印当前面贴状态
help             帮助
```

测试输入仅操作本地状态引擎，实体魔方在线时自动禁用，绝不转发到 BLE。

## 使用须知与免责声明

- 本项目与 GANCUBE / 深圳市魔萝卜科技有限公司及魔方星球 App 无任何关联，未获得任何官方认证。
- 项目将非 GAN 魔方的状态转换为 GAN 协议，属于协议逆向研究。魔方星球隐私条款明确不正当行为可能导致服务终止；联机对战、排名与奖励功能存在账号风险，请自行评估。
- 仅供个人学习研究使用，请遵守所在地区法律及目标服务的用户协议。

## 许可证

本项目以 [GPL-3.0](LICENSE) 发布。第三方参考与移植来源见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

部分魔方协议常量（根密钥 / IV / UUID / 位布局）为公开逆向事实，散见于多个开源项目。
