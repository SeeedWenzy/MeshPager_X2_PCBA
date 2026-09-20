# 无线共存测试例程

本例程用于产品新增的两组测试需求，单个固件同时保持以下功能运行：

- LoRa 持续接收；
- Wi-Fi 常驻 AP，可连接多个设备并进行 iPerf 测速；
- NimBLE 可连接 GATT 服务，手机可进行读、写和通知收发。

## 构建与烧录

在 ESP-IDF 终端执行：

```bash
cd examples/test_wireless_coexistence
idf.py set-target esp32s3
idf.py build
idf.py -p <串口> flash monitor
```

本例程的 `sdkconfig.defaults` 按 16 MB flash 配置，适用于已知 test PCBA。请根据实际板卡确认 flash 容量后再烧录。

## Wi-Fi / iPerf

启动后串口会打印：

```text
AP started: SSID=wireless_coex_ap ...
AP IPv4=192.168.4.1
```

电脑或手机连接 `wireless_coex_ap`（开放网络），然后使用与 ESP-IDF iPerf 兼容的 iPerf 2 客户端：

```bash
iperf -c 192.168.4.1 -p 5001 -t 60
iperf -c 192.168.4.1 -p 5001 -u -b 20M -t 60
```

设备端通过控制台命令启动 iPerf server：

```text
iperf -s
iperf -s -u
iperf --abort
```

本例程对 iPerf 组件做了定制：服务端启动后**持续监听**，不会因默认测试时长（30 s）到期而自动退出，一个客户端测完断开后会自动等待下一个客户端重新连接。服务端模式下 `-t` 参数被忽略，只有执行 `iperf --abort` 才会整体停止服务端任务。也就是说，从 `iperf -s` 开始直到输入 `iperf --abort`，设备一直处于 iPerf 测试模式。

如果要让电脑作为 server、设备作为 client，可使用：

```text
iperf -c 192.168.4.2 -t 60
```

其中 `192.168.4.2` 替换为电脑在 AP 网段取得的地址。iPerf 2 与 iPerf 3 的控制协议不完全相同，优先使用 iPerf 2 客户端。

## BLE

用 nRF Connect、LightBlue 或同类工具搜索并连接 `wireless_coex`。连接后找到自定义服务中的 characteristic：

- READ：读取最近一次数据；
- WRITE：写入测试数据；
- NOTIFY/INDICATE：订阅后接收设备回显。

写入数据后，设备会回显通知并在串口打印写入计数。LoRa 接收和 Wi-Fi AP 不会被停止。

## LoRa

LoRa 参数与现有 `test_fullfeatured` 接收测试保持一致：

- 频率：915 MHz；
- SF7、BW 125 kHz、CR 4/5；
- explicit header、CRC、sync word `0x12`。

串口出现 `LoRa RX packet len=...` 表示收到数据包。需要使用相同参数的 LoRa 对端持续发包。

## 无线功率命令

固件启动控制台后，使用以下命令设置发射功率：

```text
rf_power wifi <dBm>
rf_power ble <dBm>
```

Wi-Fi 支持小数点后两位为 `0`、`25`、`50` 或 `75` 的 dBm 输入，例如：

```text
rf_power wifi 17.25
rf_power wifi 17.5
rf_power ble 3
```

Wi-Fi 使用 ESP-IDF 的 0.25 dBm 输入单位，当前接口范围为 `2`～`20` dBm；芯片实际支持的功率档位会将部分输入映射到相邻的有效值，串口会同时打印请求值和 read-back 的 effective 值。启动时本例程没有主动设置 Wi-Fi 功率，实际默认值由 ESP-IDF PHY 初始化数据和国家/地区配置决定。

BLE 使用 ESP32-S3 NimBLE 控制器公开的离散档位：`-12`、`-9`、`-6`、`-3`、`0`、`3`、`6`、`9` dBm；启动时本例程没有主动设置 BLE 功率，ESP-IDF 文档定义的默认档位为 `+3` dBm。当前公共 API 不能提供 1 dB 或 0.25 dB 的真实 BLE 功率步进，输入非上述档位会被拒绝，不会静默四舍五入。若需要更细的 BLE 功率，需要支持更细档位的控制器/SDK，或使用外部射频衰减和校准方案。功率设置应在距离测试前记录，并保持 Wi-Fi、BLE 两端配置一致。


## 两个应用场景测试方法

以下两个场景使用同一个固件完成测试。两种场景都要求 LoRa 始终处于接收状态，并且在测试过程中不能因为 Wi-Fi 或 BLE 业务导致 LoRa 接收停止、设备重启或任务看门狗异常。

### 测试前准备

#### 1. 测试设备

- 被测设备：烧录本例程的 MeshPager X2；
- LoRa 对端：能够按照本例程参数持续发送 LoRa 数据包的设备；
- Wi-Fi 测试终端：电脑或手机，安装 iPerf 2 客户端；
- BLE 测试手机：安装 nRF Connect、LightBlue 或其他 BLE GATT 调试工具；
- 卷尺或测距工具，用于确认 10 m、20 m、30 m 测试距离。

#### 2. 启动设备

烧录并启动设备后，打开串口控制台，确认能够看到以下状态：

```text
AP started: SSID=wireless_coex_ap ...
BLE advertising: wireless_coex
LoRa RX started: 915000000 Hz, SF7, BW125, sync=0x12
Wireless coexistence test is running
```

如需固定无线发射功率，应在正式测试前设置并记录：

```text
rf_power wifi 20
rf_power ble 9
```

Wi-Fi 支持 `2`～`20` dBm、0.25 dBm 步进输入；实际生效值以串口 read-back 为准。BLE 支持 `-12`、`-9`、`-6`、`-3`、`0`、`3`、`6`、`9` dBm。两个应用场景应使用相同的功率配置，避免功率变化影响测试结果。

#### 3. 确认 LoRa 接收

让 LoRa 对端持续发送数据包，确认串口持续出现类似日志：

```text
COEX_LORA: LoRa RX packet len=...
```

如果没有 LoRa 收包日志，应先检查 LoRa 对端的频率、同步字、扩频因子、带宽、编码率和天线连接，不能在 LoRa 未正常接收时开始 Wi-Fi 或 BLE 距离测试。

---

### 场景一：LoRa 接收 + 蓝牙连接手机 + Wi-Fi AP + iPerf 测速

#### 测试目的

验证以下功能可以同时正常工作：

1. LoRa 持续接收；
2. 手机可以连接 BLE；
3. Wi-Fi 开启 AP，测试终端可以正常连接；
4. Wi-Fi AP 链路可以进行 iPerf TCP/UDP 测速；
5. Wi-Fi 测速期间 BLE 连接和 LoRa 接收不被影响。

#### 操作步骤

1. 启动被测设备，设置并记录 Wi-Fi、BLE 发射功率。
2. 启动 LoRa 对端持续发包，确认设备已经能够稳定收到 LoRa 数据。
3. 使用电脑或手机搜索并连接 Wi-Fi：

   ```text
   SSID: wireless_coex_ap
   ```

4. 确认测试终端通过 DHCP 获取到 `192.168.4.x` 网段地址，并能够访问设备 AP 地址。设备 AP 地址通常为：

   ```text
   192.168.4.1
   ```

5. 在设备串口控制台启动 iPerf server（启动后持续运行，不会自动退出）：

   ```text
   iperf -s
   ```

6. 在 Wi-Fi 测试终端执行 TCP 测速：

   ```bash
   iperf -c 192.168.4.1 -p 5001 -t 60 -i 1
   ```

7. 切换到 UDP 测速：先停止当前 TCP server，再以 UDP 模式启动 server，然后在测试终端执行 UDP 测速：

   ```text
   iperf --abort
   iperf -s -u
   ```

   ```bash
   iperf -c 192.168.4.1 -p 5001 -u -b 20M -t 60 -i 1
   ```

8. UDP 测速结束后，使用 `iperf --abort` 停止服务端，释放 socket：

   ```text
   iperf --abort
   ```

9. 在 iPerf 测试期间，使用手机连接 BLE 设备 `wireless_coex`，并保持 BLE 连接。
10. 使用 BLE 工具执行 Read、Write 和 Notify 测试，确认手机端仍可收发数据。
11. 保持 LoRa 对端持续发包，确认 iPerf、BLE 测试期间仍持续出现 LoRa 收包日志。
12. 在无遮挡环境下分别测试 `0 m`、`10 m`、`20 m`、`30 m`。每个距离至少持续 60 s，记录 Wi-Fi 吞吐、连接状态、LoRa 收包和 BLE 状态。

#### 场景一通过判据

- Wi-Fi 终端能够正常连接 `wireless_coex_ap` 并获得 IP；
- TCP iPerf 能够建立连接并持续输出带宽结果；
- UDP iPerf 能够完成测试，无持续性断流；
- iPerf 测试过程中设备不重启、不触发 Task Watchdog；
- BLE 手机连接保持正常，Write 后能够收到对应 Notify 回显；
- LoRa 对端持续发包时，设备仍能持续输出 `LoRa RX packet len=...`；
- 在无遮挡 20～30 m 位置，Wi-Fi 能够保持正常通信并完成产品要求的测速；
- 若 30 m 处出现断连、严重丢包或吞吐明显异常，应记录为测试结果，不得只依据 0 m 测试判定通过。

---

### 场景二：LoRa 接收 + Wi-Fi AP 已有设备连接 + 蓝牙连接手机并收发通信

#### 测试目的

验证 Wi-Fi AP 已经有其他设备连接并保持业务运行时，手机仍能够正常连接 BLE，并进行双向数据通信，同时 LoRa 继续接收。

#### BLE 测试操作

1. 启动被测设备并确认 LoRa RX 正常。
2. 使用第一台手机或电脑连接 Wi-Fi：

   ```text
   SSID: wireless_coex_ap
   ```

3. 保持该 Wi-Fi 设备在线。可以保持网络连接，也可以同时运行低速 iPerf 测试。运行前需先在设备串口启动 iPerf server（启动后持续运行，测完用 `iperf --abort` 停止）：

   ```text
   iperf -s
   ```

   ```bash
   iperf -c 192.168.4.1 -p 5001 -t 300 -i 1
   ```

4. 使用第二台手机搜索 BLE 设备：

   ```text
   wireless_coex
   ```

5. 连接后找到自定义 GATT Service 和 Characteristic：

   ```text
   Service UUID:
   9a8d3b6a-7b12-4d80-9ea1-214400000001

   Characteristic UUID:
   9a8d3b6a-7b12-4d80-9ea1-214400000002
   ```

6. 确认该 Characteristic 具有以下属性：

   ```text
   READ | WRITE | NOTIFY | INDICATE
   ```

7. 先执行 Read，设备默认数据应为 ASCII 字符串：

   ```text
   read
   ```

8. 打开 Notify 订阅，然后向 Characteristic 写入测试数据，例如：

   ```text
   01 02 03 04 05
   ```

   设备串口应出现：

   ```text
   COEX_GATT: write len=5 count=...
   ```

9. 手机应通过 Notify 收到相同的数据：

   ```text
   01 02 03 04 05
   ```

10. 重复发送不同长度的数据，例如：

    ```text
    AA
    11 22 33 44
    00 01 02 03 04 05 06 07 08 09
    ```

11. 每组数据建议连续发送 100～1000 次，记录 Write 次数、Notify 次数、断连次数和重连时间。
12. 测试过程中保持第一台设备连接 Wi-Fi，并保持 LoRa 对端持续发送数据。
13. 在无遮挡环境下，将 BLE 手机移动到 `10 m`、`20 m`、`30 m` 位置分别测试，每个距离持续至少 3～5 分钟。

#### 场景二通过判据

- 第一台设备能够持续连接 Wi-Fi AP；
- Wi-Fi AP 已有 station 连接时，BLE 手机仍能发现并连接 `wireless_coex`；
- BLE Characteristic 可以正常 Read；
- 手机 Write 后，设备串口能够打印 Write 计数；
- 手机能够收到设备通过 Notify 回传的数据，内容与 Write 数据一致；
- 连续收发过程中无异常断连，或断连后能够自动恢复广播并重新连接；
- Wi-Fi 设备在线、BLE 手机收发期间，LoRa 仍能持续收到数据包；
- 在无遮挡 20～30 m 位置，BLE 能够保持正常连接和双向通信。

---

### 测试记录表

#### 场景一：Wi-Fi + iPerf

| 距离 | Wi-Fi 功率 | BLE 功率 | TCP 吞吐 | UDP 吞吐/丢包 | Wi-Fi 是否断连 | BLE 是否断连 | LoRa 是否持续收包 | 结果 |
|---|---:|---:|---:|---:|---|---|---|---|
| 0 m |  |  |  |  |  |  |  |  |
| 10 m |  |  |  |  |  |  |  |  |
| 20 m |  |  |  |  |  |  |  |  |
| 30 m |  |  |  |  |  |  |  |  |

#### 场景二：Wi-Fi AP + BLE 收发

| 距离 | Wi-Fi 功率 | BLE 功率 | Wi-Fi station | BLE Write 次数 | BLE Notify 次数 | BLE 断连次数 | LoRa 收包 | 结果 |
|---|---:|---:|---:|---:|---:|---:|---|---|
| 0 m |  |  |  |  |  |  |  |  |
| 10 m |  |  |  |  |  |  |  |  |
| 20 m |  |  |  |  |  |  |  |  |
| 30 m |  |  |  |  |  |  |  |  |

> 20～30 m 通信距离必须通过实际无遮挡环境测试确认。测试时应同时记录 Wi-Fi、BLE、LoRa 的客观日志和异常现象，不能仅依据设备能够被发现或短时间连接来判定通过。
