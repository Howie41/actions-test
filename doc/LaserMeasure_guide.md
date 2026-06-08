# LaserMeasure 接入说明

## 当前约定

- 模块类名：`LaserMeasure`
- 串口：
  - 激光 1 -> `UART7`
  - 激光 2 -> `UART8`
- 当前地址策略：
  - 两个模块都先用 `0x80`
  - 因为两个模块分别挂在两个独立串口上，所以地址相同不会冲突
- 上位机配置：
  - 波特率：`9600`
  - 数据位：`8`
  - 停止位：`1`
  - 校验位：`None`
  - 频率：`10Hz`

## 为什么地址都可以先用 `0x80`

模块地址只有在“同一条串口总线里区分多个模块”时才必须不同。

当前硬件接法是：

- `UART7` 只连模块 1
- `UART8` 只连模块 2

所以：

- 模块 1 回包只会进 `UART7`
- 模块 2 回包只会进 `UART8`

即使两者地址都为 `0x80`，也不会发生协议冲突。

## 上位机软件要不要用

建议使用，作用有两个：

1. 验证每个模块本身是好的，能稳定出数。
2. 先把频率设成 `10Hz`。

当前阶段不强制用上位机改地址，因为我们暂时不需要把两个模块放在同一串口上。

## 当前代码文件

- 头文件：[Module/LaserMeasure/LaserMeasure.hpp](D:\32CUBEMXCODE\ROBOCON26_R2\ROBOCON26_R2\Module\LaserMeasure\LaserMeasure.hpp)
- 实现：[Module/LaserMeasure/LaserMeasure.cpp](D:\32CUBEMXCODE\ROBOCON26_R2\ROBOCON26_R2\Module\LaserMeasure\LaserMeasure.cpp)

## `LaserMeasure.hpp` 设计说明

### 类的职责

`LaserMeasure` 当前只负责三件事：

1. 记住它绑定的是哪个串口、哪个模块地址。
2. 发送单次测量命令。
3. 解析串口回包并保存最新结果。

当前不负责：

- 连续测量模式切换
- 广播测量
- 读取缓存
- 改地址
- 设置频率

这些能力后面可以再加，第一版先把最小闭环跑通。

### 常量

```cpp
static constexpr uint8_t kDefaultAddress = 0x80;
static constexpr std::size_t kMaxFrameSize = 16;
```

作用：

- `kDefaultAddress`
  - 模块默认地址
  - 现在两个模块都先按这个地址走
- `kMaxFrameSize`
  - 用于限制当前允许解析的最大帧长
  - 第一版协议帧很短，16 字节已经足够

### `MeasureResult`

```cpp
struct MeasureResult {
  bool valid{false};
  bool is_error{false};
  int32_t distance_mm{0};
  uint32_t frame_count{0};
  char error_text[8]{};
};
```

作用：

- `valid`
  - 当前这份结果能不能用
- `is_error`
  - 这次不是正常距离，而是模块返回的错误码
- `distance_mm`
  - 最新距离，统一用毫米保存
- `frame_count`
  - 成功解析出的帧计数
- `error_text`
  - 保存 `ERR-15` 这种错误字符串

为什么不只存一个距离值：

因为模块可能返回错误码。如果只存距离，就无法区分“测量失败”和“测到 0”。

### 构造函数

```cpp
explicit LaserMeasure(UartPort &uart_port, uint8_t address = kDefaultAddress)
    : uart_port_(uart_port), address_(address) {}
```

作用：

- 构造对象时绑定一个串口
- 同时绑定一个地址

举例：

```cpp
LaserMeasure laser1(uart7_port, 0x80);
LaserMeasure laser2(uart8_port, 0x80);
```

说明：

- `laser1` 使用 `uart7_port`
- `laser2` 使用 `uart8_port`
- 两者地址都为 `0x80`

### 对外接口

#### `init()`

```cpp
HAL_StatusTypeDef init();
```

作用：

- 初始化内部状态
- 当前版本只做结果清零

#### `triggerSingleMeasure()`

```cpp
HAL_StatusTypeDef triggerSingleMeasure();
```

作用：

- 发送单次测量命令

协议命令格式：

```text
ADDR 06 02 CS
```

#### `processFrame()`

```cpp
bool processFrame(const uint8_t *data, std::size_t len);
```

作用：

- 处理一整帧串口数据
- 判断这帧是否合法
- 判断是距离还是错误码
- 解析并保存到 `latest_result_`

为什么是“整帧”而不是“逐字节”：

因为当前工程串口层是 `UartPort + DMA + Idle`，更适合按数据块来处理。

#### `latestResult()`

```cpp
const MeasureResult &latestResult() const { return latest_result_; }
```

作用：

- 获取最新测距结果

这里用到了几个 C++ 语法点：

- 返回 `const MeasureResult &`
  - 不拷贝结构体
  - 外部只能读，不能改
- 函数后面的 `const`
  - 表示这个函数本身不会修改对象状态

#### `address()`

```cpp
uint8_t address() const { return address_; }
```

作用：

- 获取当前模块地址

因为 `uint8_t` 很小，所以直接按值返回即可。

### 私有函数

#### `checksum()`

```cpp
static uint8_t checksum(const uint8_t *data, std::size_t len);
```

作用：

- 按协议规则计算校验字节

协议规则：

- 前面所有字节求和
- 取反加 1

#### `parseDistancePayload()`

作用：

- 把正常测距返回的 ASCII 字符串转成毫米整数

例如：

```text
123.456
```

会被转换为：

```text
123456 mm
```

#### `parseErrorPayload()`

作用：

- 解析模块返回的错误字符串，例如 `ERR-15`

#### `clearResultValidity()`

作用：

- 清空当前结果的有效标志
- 清掉旧的距离和错误字符串

## `LaserMeasure.cpp` 设计说明

### `init()`

```cpp
HAL_StatusTypeDef LaserMeasure::init() {
  latest_result_ = {};
  return HAL_OK;
}
```

当前版本只负责清零结果，不主动发任何配置命令。

这么做的好处是：

- 初始化逻辑简单
- 出问题时好定位

### `triggerSingleMeasure()`

```cpp
HAL_StatusTypeDef LaserMeasure::triggerSingleMeasure() {
  clearResultValidity();

  uint8_t cmd[4] = {address_, 0x06, 0x02, 0x00};
  cmd[3] = checksum(cmd, 3);

  return uart_port_.write(cmd, sizeof(cmd), 10);
}
```

作用：

- 发送单次测量命令

为什么发命令前先 `clearResultValidity()`：

- 避免上层误把旧结果当成新结果

为什么用同步发送：

- 命令只有 4 字节
- 第一版先求稳，逻辑更简单

### `processFrame()`

主要处理流程：

1. 检查空指针和帧长
2. 检查地址
3. 检查回包类型是否为单次测量回包 `0x82`
4. 校验校验和
5. 抽出 payload
6. 判断是正常距离还是错误码
7. 解析成功后更新 `latest_result_`

### 正常距离解析

当前代码不使用 `sscanf` 或 `atof`，而是手工解析 ASCII：

- 小数点前部分按“米”累加
- 小数点后前三位转成毫米
- 第四位用于四舍五入

这么做的原因：

- 嵌入式里更轻量
- 行为更可控
- 结果天然就是毫米整数

### 错误码解析

如果 payload 以 `ERR` 开头，则认为是模块错误返回，而不是串口坏包。

区别如下：

- 串口坏包：`processFrame()` 返回 `false`
- 模块错误帧：解析成功，`latest_result_.valid = true`，同时 `is_error = true`

这样上层就能区分：

- “我没收到合法数据”
- “我收到的是模块给出的错误状态”

## 当前实现的边界

当前代码已经支持：

- 单次测量命令发送
- 单次测量回包解析
- 错误码识别
- 最新结果保存

当前代码还没有支持：

- 连续测量模式
- 广播测量
- 读取缓存
- 改地址命令
- 设置频率命令
- 激光开关控制

这是故意保持简单，不是遗漏。

## 当前开发顺序

建议按下面顺序继续：

1. 完成 `LaserMeasure` 模块
2. 在 `APP/robot_com/com_config.cpp` 里接入 `uart7_port` 和 `uart8_port`
3. 实例化两个 `LaserMeasure`
4. 增加接收任务或统一的 `laserMeasureTask`
5. 先让一个模块稳定返回数据
6. 再同时接两只模块

## 当前建议

- 先保持两个模块地址都为 `0x80`
- 上位机先把频率设成 `10Hz`
- 第一版代码先按“单次测量”闭环跑通

如果后面发现模块被上位机设成了上电连续测量，再根据实际行为调整驱动策略。
