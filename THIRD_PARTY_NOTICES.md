# 第三方声明

本项目中由作者原创的代码以 GPL-3.0 发布（见 [LICENSE](LICENSE)）。下列第三方项目仅作为协议与算法的**参考来源**，未直接复制其代码文件；各项目归其原作者所有。

## 参考的开源项目

| 项目 | 用途 | 许可证 |
| --- | --- | --- |
| [csTimer](https://github.com/cs0x7f/cstimer)（`mathlib.js`、`gancube.js`、`moyucube.js`、`moyu32cube.js`、`qiyicube.js`） | CubieCube 模型基准；GAN / 魔域 MHC / 魔域 MY3 / 奇艺协议解析布局；时间换算依据 | GPL-3.0 |
| [gan-web-bluetooth](https://github.com/afedotov/gan-web-bluetooth) | GAN 协议事实：RESET 固定报文等 | MIT |
| [smartcube-web-bluetooth](https://github.com/poliva/smartcube-web-bluetooth) | 多品牌智能魔方 Web Bluetooth 库（协议调研参考） | MIT |

本项目整体以 GPL-3.0 发布，与上表 GPL-3.0 的 csTimer 兼容；MIT 许可的引用项目可再授权并入 GPL-3.0 项目，均无冲突。

## 其他来源

- 魔域 MHC / WCU_MY3、奇艺 QY-QYSC 驱动逻辑与 GAN AES 密钥派生、加解密逻辑基于作者私有 Flutter 项目移植（作者保留其版权，此处按 GPL-3.0 授权发布）。
- 各品牌协议常量（AES 根密钥 / IV、服务 UUID、opcode、帧位布局）为公开的逆向工程事实数据。

## 平台组件

固件基于以下开源平台组件构建，其许可证各自适用：

- [ESP-IDF](https://github.com/espressif/esp-idf)：Apache-2.0（含 NimBLE 主机协议栈、mbedtls、FreeRTOS 等）
