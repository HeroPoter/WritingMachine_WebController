# ESP32 写字机 Web 控制器

基于 ESP32-WROOM-32 与 FastAccelStepper 的写字机控制固件，通过 Wi-Fi 网页控制三轴（X / 双 Y / Z），支持 NVS 参数保存与 HTTP 接口调用。

## 硬件接线

| 轴 | STEP | DIR |
| --- | --- | --- |
| X | 13 | 15 |
| Y 左 | 26 | 25 |
| Y 右 | 14 | 27 |
| Z | 33 | 32 |

所有 A4988 的 `EN` 并联到 GPIO 21（低电平使能）。

## 编译与烧录

1. Arduino IDE 安装 ESP32 开发板支持（测试版本 3.3.10）与 `FastAccelStepper` 库（2.0.1）。
2. 开发板选择 `ESP32 Dev Module`，Flash 4 MB，分区 `Default 4MB with spiffs`，上传速度 921600。
3. 打开 `WritingMachine_WebController.ino` 编译上传。

上电后位置默认清零为 (0, 0, 0)。若已保存 Wi-Fi 凭据会自动连接，串口打印 IP；否则开启热点 `WritingMachine`（密码 `12345678`），连接后浏览器访问 `192.168.4.1` 配网。

## 网页功能

- 三轴速度、加速度、行程、停顿参数，点击表头启用/停用对应轴
- 往返（循环）/ 单次 / 停止 / 清零 / 保存
- 参数与轴启用状态写入 NVS，断电后恢复
- 各轴按自己的停顿值独立往复

## HTTP 接口

### 定点往返 `/moveto`

```
GET /moveto?x=1000&y=1000&z=500
            &xv=150000&xac=150000&xsl=100
            &yv=150000&yac=150000&ysl=100
            &zv=150000&zac=150000&zsl=100
```

- `x` / `y` / `z`：目标位置（步），任意组合，未给出的轴不参与
- `xv` / `yv` / `zv`：速度（Hz）
- `xac` / `yac` / `zac`：加速度（Hz/s）
- `xsl` / `ysl` / `zsl`：该轴到位后的停顿（ms）
- `pause`：可选，统一覆盖三轴停顿
- `save=1`：可选，把速度/加速度/停顿写入 NVS

接口立即返回，各轴走到目标后按自己的停顿各自返回原点 (0, 0, 0)。`/move` 为兼容别名。

### 状态 `/status`

```json
{"running":false,"api":false,"stage":{"x":4,"y":4,"z":4},"x":0,"y":0,"z":0}
```

`stage`：0 去程、1 到位停顿、2 回程、3 原点停顿、4 完成；`api` 表示当前是否为接口触发的任务。

### 其他

- `GET /start?axes=xy&mode=loop|once`：按表格参数启动
- `GET /stop`：停止
- `GET /home`：位置计数清零
- `GET /save?...`：保存参数

## 换算

示例机器为 1000 步/cm，即 10000 步 = 10 cm。不同丝杆/细分请用表格里的行程自行标定。
