
# 硬件通信协议文档  
**版本：1.2**  
**修订日期：2023-10-12**  

---

## 1. 协议概述  
本协议用于规范上位机（主设备）与下位机（从设备）之间的通信交互。上位机主动发送指令，下位机执行指令并反馈执行状态。协议通过固定格式的命令和状态响应机制实现双向通信，支持实时状态查询和错误处理。 如使用串口通信波特率115200,起始位 数据： 8位数据位，1位起始位，无校验。

---

## 2. 命令格式  
每条命令由以下部分组成，顺序固定，格式为：  
`节点名称@功能代码:参数1,参数2&校验码结束符`  

| 字段           | 说明                                                                 | 示例                     |
|----------------|----------------------------------------------------------------------|--------------------------|
| **节点名称**   | 设备标识，格式为`节点名+编号`（如 `pay01`、`node1`）                  | `node1`                  |
| **功能代码**   | 功能标识，由字母和数字组成（如 `f1000x` 表示导轨初始化）              | `f1000x`                 |
| **参数**       | 功能参数，格式为 `键:值`，多个参数用逗号分隔                          | `s:1, t:100.00`          |
| **CRC16校验码**| 校验字段，用于验证数据完整性，固定长度为4字符，前缀为`0x`             | `0x8765`                 |
| **结束符**     | 固定为 `#`，表示命令结束                                             | `#`                      |

**分隔符规则**：  
- 参数与校验码之间使用 `&` 分隔（如 `参数&校验码#`）。  
- 校验码前缀 `0x` 表示十六进制格式（如 `0xfe12`）。  

**完整示例**：  
```plaintext
node1@f1000x:9999.99&0x8765#
```
（功能：导轨初始化，目标坐标 `9999.99`，校验码 `0x8765`）



## 3. 通信流程  
### 3.1 指令下发  
1. **上位机发送命令**：  
   格式：`节点@功能代码参数&校验码#`  
   示例：  
   ```plaintext
   node1@fd001s:1,t:100.00&0xfe12#
   ```  

2. **下位机响应**：  
   - **立即响应**：替换结束符 `#` 为 `~`，表示已接收并开始执行。  
     示例：  
     ```plaintext
     node1@fd001s:1,t:100.00&0xfe12~
     ```  
   - **执行失败**：返回 `?` 结尾的命令，表示CRC校验错误。  
     示例：  
     ```plaintext
     node1@fd001s:1,t:100.00&0xfe12?
     ```  
   - **校验失败**：返回 `E` 开始的命令，表示错误。  
     示例：  
     ```plaintext
     E001  数据包错误
     ```  
     

### 3.2 状态查询  
- **上位机轮询**：重复发送原指令（保持 `#` 结束符），查询执行状态。  
- **下位机返回最终状态**：  
  - **执行成功**：替换结束符为 `!`。  
    示例：  
    ```plaintext
    node1@fd001s:1,t:100.00&0xfe12!
    ```  
  - **执行失败**：替换结束符为 `?`，并附带错误码（可选）。  
    示例：  
    ```plaintext
    node1@fd001s:1,t:100.00&0xfe12?
    E002
    ```  

---

## 4. 错误处理  
| 错误类型         | 处理方式                                                                 |
|------------------|--------------------------------------------------------------------------|
| **CRC校验失败**  | 下位机立即返回 `E001` ，上位机需重发指令。                            |
| **参数不合法**   | 下位机返回 `?` 结束符，上位机需检查参数格式后重试。                      |
| **执行超时**     | 上位机可设定超时阈值（默认5秒），超时后标记为失败并重发指令（最多3次）。 |

---

## 5. 示例命令表（完整版）  

| 功能描述                | 示例命令                                      | 参数说明                            | 成功响应           |
|-------------------------|-----------------------------------------------|-------------------------------------|--------------------|
| 1. 硬件初始化           | `node1@f0000&0x4589#`                        | 无参数                             | `...0x4589!`       |
| 2. 导轨初始化           | `node1@f1000x:9999.99&0x8765#`               | 目标坐标：`9999.99`                | `...0x8765!`       |
| 3. 导轨运动至指定坐标   | `node1@f1000x:1876.00&0x8765#`               | 目标坐标：`1876.00`                | `...0x8765!`       |
| 4. GPIO DO1开启（100ms）| `node1@fd001s:1,t:100.00&0xfe12#`            | `s:1`（开启），`t:100.00`（时间）  | `...0xfe12!`       |
| 5. GPIO AO2关闭         | `node1@fa002s:0&0xa2c1#`                     | `s:0`（关闭）                      | `...0xa2c1!`       |
| 6. 打开第一路料盒（100ms）| `node1@fm001t:100.00&0xd2fc#`               | `t:100.00`（时间）                 | `...0xd2fc!`       |
| 7. 打开第二路水泵（100ms）| `node1@fw002t:100.00&0xa2c5#`               | `t:100.00`（时间）                 | `...0xa2c5!`       |
| 8. 锅炉加热设置（89℃）  | `node1@fn001t:89.00&0xa2c5#`                | `t:89.00`（温度）                  | `...0xa2c5!`       |
| 9. 读取传感器数值       | `node1@fs001&0xa2c5#`                        | 无参数                             | `...0xa2c5!`       |
| 10. 多参数示例（GPIO控制）| `node1@fd003s:1,t:200.00,d:50&0xabcd#`     | `s:1`（开启），`t:200.00`（时间），`d:50`（占空比） | `...0xabcd!` |

---

## 6. 附录  
### 6.1 CRC16校验码计算规则  
- **算法**：Modbus CRC16（多项式 `0x8005`，初始值 `0xFFFF`）。  
- **计算范围**：从节点名称开始到参数结束的所有字符（不包含 `&` 和校验码）。  

**示例计算步骤**：  
1. 提取命令中的参数部分（如 `node1@f1000x:9999.99`）。  
2. 计算其CRC16校验值，转换为十六进制格式（如 `0x8765`）。  

### 6.2 特殊功能代码表  
| 功能代码   | 说明                   | 参数格式              |
|------------|------------------------|-----------------------|
| `f0000`    | 硬件初始化             | 无参数                |
| `f1000`   | 导轨初始化             | 目标坐标（浮点数）    |
| `fd001`    | GPIO数字输出控制       | `s:状态（0/1）`       |
| `fa002`    | GPIO模拟输出控制       | `s:状态（0/1）`       |
| `fm001`    | 料盒控制               | `t:时间（ms）`        |
| `fw002`    | 水泵控制               | `t:时间（ms）`        |
| `fn001`    | 锅炉温度设置           | `t:温度值（浮点数）`  |
| `fs001`    | 传感器数据读取         | 无参数                |


---

## 修订记录  
| 日期       | 版本  | 修订内容                     |  
|------------|-------|------------------------------|  
| 2023-10-01 | 1.0   | 初版发布                     |  
| 2023-10-05 | 1.1   | 补充错误处理逻辑与完整示例命令表 |  
| 2023-10-12 | 1.2   | 修正校验码分隔符为 `&`，统一格式 |  

---  

