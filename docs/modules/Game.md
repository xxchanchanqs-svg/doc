# CARLA 游戏模块说明


- [引擎模块](#engine_module)







# 引擎模块  <span id="engine_module"></span>
主要函数

代码结构

贡献代码

## 简介 <span id="introduction"></span>

本项目的 [CarlaEngine.cpp](https://github.com/OpenHUTB/hutb/blob/hutb/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Game/CarlaEngine.cpp) 负责管理 Carla 的核心引擎功能，如固定时间步长、RPC 服务器线程数等。

## 环境依赖 <span id="env_dependence"></span>
本模块依赖以下组件： Unreal Engine 、CARLA 源码 、ROS2（可选） 、多 GPU 支持库

## 核心功能 <span id="core_function"></span>

CarlaEngine.cpp 主要提供以下核心功能：

*　时间步长控制：允许用户获取和设置固定的物理模拟时间步长。
多线程支持：提供对 RPC 服务器线程数的管理。

＊　场景管理：管理 CARLA 场景生命周期，支持多 GPU 计算。
与 ROS2 交互：通过 carla::ros2::ROS2 进行通信。

## 主要函数
```shell
FCarlaEngine_GetFixedDeltaSeconds()
static TOptional<double> FCarlaEngine_GetFixedDeltaSeconds();
功能： 获取 CARLA 的固定时间步长。 2. FCarlaEngine_GetNumberOfThreadsForRPCServer()

static uint32 FCarlaEngine_GetNumberOfThreadsForRPCServer();
功能： 获取用于 RPC 服务器的线程数量。

FCarlaEngine_SetFixedDeltaSeconds(TOptional FixedDeltaSeconds)
static void FCarlaEngine_SetFixedDeltaSeconds(TOptional<double> FixedDeltaSeconds);
功能：设置固定的物理模拟时间步长。

if (bIsRunning)

if (bIsRunning)
功能：确保引擎正在运行后再执行特定操作。
```


## 代码结构
本模块的代码结构如下：

CARLA
├── Unreal
│   ├── CarlaUE4
│   │   ├── Plugins
│   │   │   ├── Carla
│   │   │   │   ├── Source
│   │   │   │   │   ├── Carla
│   │   │   │   │   │   ├── Game
│   │   │   │   │   │   │   ├── CarlaEngine.cpp  # 主要文件
│   │   │   │   │   │   │   ├── CarlaEngine.h
│   │   │   │   │   │   │   ├── CarlaEpisode.h
│   │   │   │   │   │   │   ├── CarlaStatics.h


# 游戏模块 <span id="game_module"></span>

---

## 目录  
1. [模块概述](#📝模块概述)  
2. [核心功能详解](#📚核心功能详解)  
3. [模块间关系](#📑模块间关系)  
4. [类与方法详解](#📜类与方法详解)  
5. [模块调用逻辑](#📝模块调用逻辑)  
6. [注意事项](#❗注意事项) 
---

## 📝模块概述
`Game`模块是 HUTB 项目在引擎中的核心逻辑模块，负责管理人车仿真场景的完整生命周期。其核心功能包括：
- **场景管理**：加载 OpenDRIVE 地图、动态设置天气及交通规则；
- **角色控制**：生成车辆、行人、传感器等实体，并绑定物理行为（如车辆移动组件）；
- **传感器交互**：通过摄像头、雷达、激光雷达等传感器实时采集数据，并支持异步传输到服务端或 ROS2；
- **录制与回放**：记录车辆位置、传感器数据，并支持按帧回放；
- **ROS2 集成**：通过帧同步机制与 ROS2 通信，发布传感器消息（需启用`WITH_ROS2`宏）；
- **语义分割**：为生成的角色分配语义标签（如`CustomDepthStencilValue`），支持自动驾驶算法训练与可视化。

该模块通过 **游戏模式基类** [`CarlaGameModeBase`](https://github.com/OpenHUTB/hutb/blob/hutb/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Game/CarlaGameModeBase.h) 、**剧情** `CarlaEpisode` 等核心类协调引擎与 HUTB 服务端的交互，是连接游戏逻辑与人车模拟的关键桥梁。

---
## 📚核心功能详解
### 2.1 场景管理
#### **地图加载**
- **OpenDRIVE 解析**：通过 `carla::opendrive::OpenDriveParser` 解析 `.xodr` 文件生成道路网络。
  ```cpp
  void CarlaEpisode::LoadMap(const FString& MapName) {
    carla::opendrive::OpenDriveParser parser(MapName);
    for (const auto& road : parser.GetRoads()) {
      CreateRoadActor(road); // 生成道路Actor
    }
  }
  ```
- **动态天气设置**：通过`carla::rpc::WeatherParameters`动态调整雾、雨、云等天气参数。
  ```cpp
  void CarlaEpisode::SetWeather(const FWeatherParameters& Weather) {
    carla::rpc::WeatherParameters carlaWeather;
    carlaWeather.cloudiness = Weather.Cloudiness;
    carlaWeather.wetness = Weather.Wetness;
    // 设置虚幻引擎天气系统
    World->SetWeather(carlaWeather);
  }
  ```
#### **交通规则初始化**
- **信号灯管理**：加载交通信号灯、道路标志等实体。
  ```cpp
  void CarlaEpisode::InitializeTrafficLights() {
    for (auto* Light : TrafficLightList) {
      Light->SetState(carla::rpc::TrafficLightState::Green); // 初始化为绿灯
    }
  }
  ```
### 2.2 角色控制
#### **实体生成**
- **Actor 生成逻辑**：通过`CarlaGameModeBase`的`SpawnActor`方法生成车辆、行人、传感器。
  ```cpp
  AActor* CarlaGameModeBase::SpawnActor(TSubclassOf<AActor> Class) {
    return GetWorld()->SpawnActor(Class, &Transform); // 在指定位置生成Actor
  }
  ```
#### **行为管理**
- **车辆物理模拟**：绑定车辆移动组件（`UWheeledVehicleMovementComponent`）实现物理模拟。
  ```cpp
  void ACarlaVehicle::InitializePhysics() {
    UWheeledVehicleMovementComponent* MovementComponent = FindComponentByClass<UWheeledVehicleMovementComponent>();
    MovementComponent->SetMaxSpeed(100.0f); // 设置最大速度
  }
  ```
### 2.3 传感器交互
#### **数据采集**
- **摄像头传感器**：实时采集图像数据。
  ```cpp
  void ACarlaCamera::Tick(float DeltaTime) {
    if (bIsActive) {
      RenderImage(); // 渲染当前帧图像
      SendDataToServer(); // 发送数据到服务端
    }
  }
  ```
- **激光雷达传感器**：采集点云数据。
  ```cpp
  void ACarlaLidar::UpdateLidarData() {
    TArray<FVector> PointCloud = GetPointCloud(); // 获取点云数据
    SendPointCloud(PointCloud); // 发送点云到服务端
  }
  ```
#### **数据处理**
- **异步数据传输**：通过`AsyncDataStreamImpl`异步传输传感器数据到服务端。
  ```cpp
  void AsyncDataStreamImpl::SendSensorData(const TArray<uint8>& Data) {
    Async(EAsyncExecution::Thread, [Data]() {
      // 处理数据并发送到服务端
      SendToServer(Data);
    });
  }
  ```
### 2.4 录制与回放
#### **录制功能**
- **录制逻辑**：通过`ACarlaRecorder::StartRecording()`记录车辆位置、传感器数据。
  ```cpp
  void ACarlaRecorder::StartRecording() {
    FileWriter.Open("recording.log");
    FileWriter.Write("Frame,VehiclePosition,X,Y,Z\n");
  }
  ```
#### **回放功能**
- **回放逻辑**：通过`CarlaReplayer::PlayBack()`按帧回放录制数据。
  ```cpp
  void CarlaReplayer::PlayBack() {
    while (HasMoreFrames()) {
      ReadNextFrame(); // 读取下一帧数据
      ApplyFrameData(); // 应用到车辆和传感器
    }
  }
  ```
### 2.5 ROS2 集成
#### **帧同步**
- **帧号同步**：通过`FCarlaEngine::FrameCounter`同步仿真帧与 ROS2 话题时间戳。
  ```cpp
  void FCarlaEngine::OnTick() {
    FrameCounter += 1;
    #if defined(WITH_ROS2)
    carla::ros2::ROS2::GetInstance()->SetFrame(FrameCounter);
    #endif
  }
  ```
#### **消息发布**
- **传感器数据转换**：将传感器数据转换为 ROS2 消息（如`sensor_msgs::Image`）。
  ```cpp
  void ACarlaCamera::PublishToROS2() {
    sensor_msgs::Image msg;
    ConvertToROSImage(ImageData, msg); // 转换为ROS2图像消息
    Publisher->Publish(msg); // 发布到ROS2话题
  }
  ```
### 2.6 语义分割
#### **标签系统**
- **语义标签分配**：通过`UTaggerDelegate`为生成的角色分配语义标签（如`CustomDepthStencilValue`）。
  ```cpp
  void UTaggerDelegate::OnActorSpawned(AActor* Actor) {
    Actor->SetCustomDepthStencilValue(1); // 分配语义标签
  }
  ```
#### **颜色映射**
- **HUD 可视化**：在`ACarlaHUD`中绘制语义分割颜色映射图。
  ```cpp
  void ACarlaHUD::DrawSemanticSegmentation() {
    for (auto* Actor : SemanticActors) {
      DrawColoredBox(Actor->GetActorLocation(), Actor->GetSemanticColor()); // 绘制语义颜色框
    }
  }
  ```
## 📑模块间关系
### **3.1 调用关系图**

<p align="center">
 <img src="https://raw.githubusercontent.com/Elotkowski/KD-LoRA/refs/heads/main/00.png?token=GHSAT0AAAAAADCITNBNRCKN5E4U2VJ3HKIQ2A3GUJQ"  style="max-width: 100%; height: auto;"/>
</p>

### **3.2 模块依赖**
以下为 `Game` 模块与其他模块的完整依赖关系（数字表示依赖强度）：

| 模块名称       | 与 Game 模块的关系数 | 主要功能说明                     |
|----------------|----------------------|----------------------------------|
| **LibCarla**   | 51                   | 核心逻辑交互，包含 RPC、地图加载等 |
| **Recorder**   | 25                   | 录制与回放功能                   |
| **Settings**   | 9                    | 配置加载与参数管理               |
| **Sensor**     | 6                    | 传感器数据采集与处理             |
| **Util**       | 6                    | 工具函数（异步传输、数据处理）   |
| **Traffic**    | 5                    | 交通规则与信号灯控制             |
| **Lights**     | 4                    | 交通信号灯管理                   |
| **Actor**      | 4                    | 角色生成与交互                   |
| **MapGen**     | 4                    | 地图生成与 OpenDRIVE 解析        |
| **Server**     | 3                    | 服务端通信接口                   |
| **Vehicle**    | 3                    | 车辆物理模拟与控制               |
| **Weather**    | 2                    | 动态天气参数设置                 |
| **OpenDrive**  | 1                    | OpenDRIVE 地图文件解析           |

详细模块关系参考：[ :octocat: ](https://openhutb.github.io/carla_cpp/dir_b708e75f0564cefaa95a07ef1c60fa1d.html)
## 📜类与方法详解
### **4.1 ACarlaGameModeBase**
- **功能**：游戏模式基类，负责场景初始化和全局逻辑控制。
- **关键方法**：
  - **`InitGame(...)`**  
    初始化核心模块（`CarlaEpisode`、`CarlaRecorder`）并加载地图。
    ```cpp
    void ACarlaGameModeBase::InitGame(...) {
      Episode = CreateDefaultSubobject<UCarlaEpisode>("Episode");
      Recorder = CreateDefaultSubobject<ACarlaRecorder>("Recorder");
      Episode->LoadMap(SelectedMap); // 加载指定地图
    }
    ```

  - **`Tick(float DeltaTime)`**  
    每帧调用，更新场景状态和传感器数据。
    ```cpp
    void ACarlaGameModeBase::Tick(float DeltaTime) {
      Episode->Update(); // 更新车辆/传感器状态
      Recorder->CheckRecord(); // 检查是否需要录制
    }
    ```

### **4.2 CarlaEpisode**
- **功能**：管理场景生命周期（地图、天气、交通规则）。
- **关键方法**：
  - **`LoadMap(const FString& MapName)`**  
    解析 OpenDRIVE 文件并生成道路网络。
    ```cpp
    void CarlaEpisode::LoadMap(const FString& MapName) {
      carla::opendrive::OpenDriveParser parser(MapName);
      for (const auto& road : parser.GetRoads()) {
        CreateRoadActor(road); // 生成道路Actor
      }
    }
    ```

  - **`SetWeather(const FWeatherParameters& Weather)`**  
    应用动态天气参数（云量、湿度等）。
    ```cpp
    void CarlaEpisode::SetWeather(const FWeatherParameters& Weather) {
      carla::rpc::WeatherParameters carlaWeather;
      carlaWeather.cloudiness = Weather.Cloudiness;
      World->SetWeather(carlaWeather);
    }
    ```

### **4.3 ACarlaRecorder**
- **功能**：记录和回放模拟过程数据。
- **关键方法**：
  - **`StartRecording()`**  
    开始记录车辆位置、传感器数据到文件。
    ```cpp
    void ACarlaRecorder::StartRecording() {
      FileWriter.Open("recording.log");
      FileWriter.Write("Frame,VehiclePosition,X,Y,Z\n");
    }
    ```

  - **`PlayBack()`**  
    按帧回放录制数据。
    ```cpp
    void ACarlaRecorder::PlayBack() {
      while (HasMoreFrames()) {
        ReadNextFrame(); // 读取下一帧数据
        ApplyFrameData(); // 应用到车辆和传感器
      }
    }
    ```

### **4.4 UTaggerDelegate**
- **功能**：语义分割标签管理器，为角色分配语义ID。
- **关键方法**：
  - **`OnActorSpawned(AActor* Actor)`**  
    为生成的角色分配 `CustomDepthStencilValue`。
    ```cpp
    void UTaggerDelegate::OnActorSpawned(AActor* Actor) {
      Actor->SetCustomDepthStencilValue(1); // 分配语义标签
    }
    ```

### **4.5 FCarlaEngine**
- **功能**：引擎核心逻辑，提供帧计数器和ROS2集成。
- **关键方法**：
  - **`OnTick()`**  
    递增帧计数器并同步到ROS2。
    ```cpp
    void FCarlaEngine::OnTick() {
      FrameCounter += 1;
      #if defined(WITH_ROS2)
      carla::ros2::ROS2::GetInstance()->SetFrame(FrameCounter);
      #endif
    }
    ```

### **4.6 CarlaSensor**
- **功能**：传感器基类，定义数据采集与传输接口。
- **关键方法**：
  - **`Update(float DeltaTime)`**  
    采集传感器数据并异步发送到服务端。
    ```cpp
    void CarlaSensor::Update(float DeltaTime) {
      TArray<uint8> Data = Capture(); // 采集数据
      AsyncDataStreamImpl::Send(Data); // 异步传输
    }
    ```

  - **`PublishToROS2()`**  
    将数据转换为ROS2消息并发布。
    ```cpp
    void CarlaSensor::PublishToROS2() {
      sensor_msgs::Image msg;
      ConvertToROSImage(Data, msg); // 转换为ROS2图像消息
      Publisher->Publish(msg); // 发布到ROS2话题
    }
    ```

---


## 📝模块调用逻辑

本模块的代码结构如下：

	Game
    ├── LibCarla (51)                # 核心交互
    │   ├── LoadMap()                # 地图加载
    │   ├── SetWeather()             # 天气设置
    │   └── VehicleControl()         # 车辆控制
    ├── Recorder (25)                # 录制与回放
    │   ├── StartRecording()         # 开始录制
    │   └── PlayBack()               # 回放
    ├── Sensor (6)                   # 传感器
    │   ├── CaptureData()            # 数据采集
    │   └── SendToServer()           # 数据传输
    ├── Settings (9)                 # 配置管理
    ├── Traffic (5)                  # 交通规则
    │   ├── Lights (4)               # 信号灯控制
    ├── Util (6)                     # 工具函数
    └── Weather (2)                  # 动态天气

## ❗注意事项
### **6.1 性能优化**
- **高频调用函数**：  
  - `FCarlaEngine::OnTick()` 和 `CarlaGameModeBase::Tick()` 需避免阻塞操作，建议将传感器数据处理移至多线程（如 `AsyncDataStreamImpl`）。  
  - 高权重模块（如 `LibCarla`、`Recorder`）的调用需控制频率，防止帧率下降。

- **内存管理**：  
  - 复用 `HUDString` 和 `HUDLine` 结构体实例，减少动态内存分配。  
  - 避免频繁创建/销毁 `AsyncDataStreamImpl` 实例，建议复用连接。

---

### **6.2 资源释放**
- **文件句柄**：  
  - `CarlaRecorder::StopRecording()` 必须手动调用 `FileWriter.Close()` 释放文件资源。  
  - 回放结束后需清理 `CarlaReplayer` 缓存的帧数据。

- **线程安全**：  
  - 多线程访问 `FrameCounter` 时需加锁（`std::mutex`），防止竞态条件。  
  - `AsyncDataStreamImpl::Send()` 需确保线程间数据同步。

---

### **6.3 版本兼容性**
- **ROS2 宏启用**：  
  - 使用 ROS2 功能前需确认 `WITH_ROS2` 宏已启用，否则编译会报错。  
  - 依赖的 ROS2 消息类型需与 CARLA 服务端版本匹配。

- **UE4/UE5 差异**：  
  - `UWorld` 和 `AActor` 在 UE5 中的接口可能变化，需适配 `CarlaEpisode` 和 `Sensor` 的调用逻辑。  
  - `AsyncDataStreamImpl` 在 UE5 中需替换为 `FAsyncTask` 实现。

---

### **6.4 常见问题**
- **传感器未响应**：  
  - 检查 `CarlaEpisode` 是否初始化成功。  
  - 确认 `CarlaGameInstance` 已绑定到 `PlayerVehicle` 并激活传感器。

- **语义分割失败**：  
  - 确保 `UTaggerDelegate::SetSemanticSegmentationEnabled(true)` 已调用。  
  - 验证 `CustomDepthStencilValue` 是否在渲染管线中正确配置。

- **录制文件损坏**：  
  - 回放前检查 `recording.log` 文件格式是否完整。  
  - 确保录制时 `CarlaRecorder::StartRecording()` 已正确调用。

---

### **6.5 开发建议**
- **模块依赖控制**：  
  - 避免直接依赖高权重模块（如 `LibCarla`），优先通过接口抽象解耦。  
  - 使用 `Settings` 模块动态配置模块启用状态，提升灵活性。

- **调试工具**：  
  - 启用 `CarlaHUD` 的调试模式，实时显示帧率、内存占用等关键指标。  
  - 利用 `AsyncDataStreamImpl` 的日志功能追踪传感器数据传输异常。
