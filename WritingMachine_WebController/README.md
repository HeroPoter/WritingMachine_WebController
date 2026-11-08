# ESP32 写字机 Web 控制器

基于 ESP32-WROOM-32 与 FastAccelStepper 的写字机控制固件，通过 Wi-Fi 网页控制三轴（X / 双 Y / Z），支持 NVS 参数保存与 HTTP 接口调用。

## 目录结构

- `WritingMachine_WebController/`：Wi-Fi 网页控制固件，本项目主程序
- `xyz/`：串口测试程序，上电即按固定参数往复，用于验证电机接线与方向

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
- 最大行程 (mm)：API 与手动运行的安全上限，0 表示不限制；固件按 `STEPS_PER_MM`（默认 100 步/mm）换算成步

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

### 点击 `/click`

```
GET /click?x=10000&y=15000[&z=500]
```

- `x` / `y`：目标绝对坐标（步），X/Y 走到该处停住，不返回原点；未给出的轴不移动
- `z`：Z 轴点击行程（步），缺省 500

X/Y 到位后 Z 轴立即做一次 `z` 步的下压抬起往返，实现"移动到指定位置并点击"。接口立即返回，进度查 `/status`（`click`、`clickPhase`）。

### 点击（相对）`/clickto`

```
GET /clickto?x=1000&y=1500[&z=500]
```

`x` / `y` 为**相对当前坐标的偏移**（可为负），X/Y 移动到"当前 + 偏移"处停住，到位后 Z 轴做 `z` 步单击（缺省 500），其余同 `/click`。

### 任务互斥

运动类指令（`/moveto`、`/move`、`/click`、`/clickto`、`/start`）在上一条任务成功完成后才接受新指令；任务执行中会返回 HTTP `409`：`{"ok":false,"msg":"busy, ..."}`。`/stop`、`/home`、`/status` 始终可用，可随时中止或查询。

另外，目标位置（`/clickto` 为当前+偏移后的结果）超出对应轴的最大行程时，直接拒绝执行并返回 HTTP `400`：`{"ok":false,"msg":"target exceeds max travel"}`；手动运行 `/start` 超行程时返回纯文本 `TRAVEL_LIMIT_x`（x/y/z 为超限的轴）。

### 状态 `/status`

```json
{"running":false,"api":false,"stage":{"x":4,"y":4,"z":4},"click":false,"clickPhase":0,"x":0,"y":0,"z":0}
```

`stage`：0 去程、1 到位停顿、2 回程、3 原点停顿、4 完成；`api` 表示当前是否为接口触发的任务；`click`/`clickPhase` 为 `/click` 状态（0 下发 XY、1 等 XY 到位、2 Z 下压、3 Z 抬起）。

### 其他

- `GET /start?axes=xy&mode=loop|once`：按表格参数启动
- `GET /stop`：停止
- `GET /home`：位置计数清零
- `GET /save?...`：保存参数

## 换算

示例机器为 1000 步/cm，即 10000 步 = 10 cm，对应源码宏 `STEPS_PER_MM = 100`（mm↔步 换算，最大行程即按此换算）。不同丝杆/细分请修改该宏，并用表格里的行程自行标定。
