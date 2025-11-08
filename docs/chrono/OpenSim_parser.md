# Chrono OpenSim 解析器

## 1.介绍

## 2.背景

## 3.实现和使用

### 3.1 刚体

### 3.2 支持的关节类型

### 3.3 力和执行器元件

目前，`Simbody` 力元素尚未被解析和解释（例如，`PrescribedForce`、`SpringGeneralizedForce` 和 `BushingForce`）。

OpenSim 还包含“理想”执行器，它们通过其最佳力设置（即增益）施加与输入控制（即激励）成正比的纯力或扭矩。这些力和扭矩可以施加在单个物体上，也可以施加在两个物体之间。`ChParserOpenSim` 目前可以解析和解释以下内容：

- 点执行器**PointActuator**：在指定点沿指定方向对给定物体施加力；施加点和力的方向均可在绝对参考系或物体局部参考系中提供。

- 扭矩执行器**TorqueActuator**：对两个物体绕指定轴施加大小相等、方向相反的扭矩；扭矩方向可以在绝对坐标系或物体局部参考系中给出。

解析器将这些执行器转换为 Chrono loads(负载)，Chrono 负载是对可施加于各种 Chrono 建模元素（包括刚体）的力和扭矩的概括。具体来说，`PointActuator` 被转换为 `ChLoadBodyForce`，而 `TorqueActuator` 被转换为 `ChLoadBodyBodyTorque`。Chrono 负载被收集到所谓的负载容器(load containers)中，类型为 `ChLoadContainer`。从 OpenSim 执行器解析出的所有负载都收集到单个负载容器中（一个 `ChSystem` 可以包含任意数量的此类容器）。

在 OpenSim 中，激励函数可以通过多种方式指定：可以直接在 `.osim` 文件中指定，也可以在单独的**控制**文件（XML 格式）中指定，并在 `.osim` 文件中引用该文件。对于前者，OpenSim 还提供了一个简单的语法，允许用户为这些激励指定各种数学函数。为了简化解析过程并提供更大的灵活性，我们选择允许用户通过 Chrono 的 `ChFunction` 机制在 Chrono 中原生指定自己的激励函数。`ChFunction` 基类定义了一个单变量的抽象标量函数 (y = f(x))，并提供了一组相对丰富的具体派生类（包括多项式、正弦、分段线性等）；此外，用户还可以通过定义自己的派生 `ChFunction` 类来扩展此集合（详情请参阅 Chrono API 参考文档）。

默认情况下，遵循 OpenSim 的设计理念，所有转换后的执行器均不被激励（换句话说，它们的激励函数为常数 0）。用户也可以选择在解析过程之前调用 `parser.ActivateActuators(true)`，使所有 Chrono 执行器负载都具有完全激励（即，激励函数为常数 1）。解析完成后，可以通过 `ChParserOpenSim::SetExcitationFunction()` 设置特定执行器的激励，如清单 1 所示。清单 1 中，执行器的名称通过其在 .osim 文件中的名称来引用（也可以通过 3.7 节中描述的报告对象访问）。解析器目前会忽略上述未列出的执行器类型，但用户可以通过 `ChParserOpenSim::Report::GetBody()` 找到相关的执行器，并在它们之间创建自定义力。

```cpp
auto excitation = std::make_shared<ChFunction_Ramp>(0, 1);
parser.SetExcitationFunction( "grav" , excitation );
```


### 3.4 可视化

用户可以控制生成的 Chrono 体的可视化方式。以下选项可用：

- **none** 不创建任何可视化资源（这是默认设置）

- **primitives** 使用 Chrono 的基本可视化资源，为每个身体创建一个可视化模型。该模型由一个以身体参考系原点为中心的球体和连接身体参考系与所有相邻关节位置（内侧关节和所有外侧关节）的圆柱体组成。参见下图中的示例。

    ![](../img/chrono/OpenSim_model_simulation.jpg)

- **mesh** 使用可视化网格，假定网格以 Wavefront `obj` 文件的形式提供，并在输入 osim 文件中的 `<geometry_file>` 标签下指定。参见下图中的示例。

    ![](../img/chrono/human_OpenSim.jpg)



### 3.5 碰撞和接触

默认情况下，生成的 Chrono 实体不会创建碰撞几何体。如果启用此功能，则会根据基本圆柱体形状创建碰撞模型，类似于 `PRIMITIVES` 模式下的可视化形状。由于这意味着相邻实体的碰撞形状会在连接关节处重叠，因此实体会根据其在树状层次结构中的深度，被分配到交替的碰撞族中。实体与其父实体之间不会生成碰撞；允许与任何其他实体（包括同级实体）发生碰撞。

Chrono 支持两种接触处理方法：**非光滑、基于互补性的**(non-smooth, complementarity-based, NSC)方法和**光滑、基于惩罚的**(SMC, smooth, penalty-based)方法。具体方法取决于包含 ChSystem 的类型。ChParserOpenSim 类中的可选方法允许指定接触材料属性（参见 [chrono api](http://api.chrono.projectchrono.org/index.html) ）。

### 3.6 样例使用

示例 2 展示了如何解析 osim 文件以填充现有的 Chrono 系统。使用方法如下：

```cpp
ChSystemSMC my_system;
std::stringfilename = GetChronoDataFile("opensim/skeleton.osim");
ChParserOpenSimparser;
parser.SetVisualizationType (ChParserOpenSim::VisType::PRIMITIVES);
parser.EnableCollision();
parser.SetVerbose(true);
parser.Parse(my_system, filename);
```

```cpp
/// 报告包含有关从文件中解析的对象的信息
class ChApi Report {
public :
/// 从 OpenSim 读取关节数据的信息。
struct JointInfo {
std::string type;               ///< 关节类型如osim文件中所示
std::shared_ptr<ChLink> joint;  ///< Chrono 连杆 (关节)       
bool standin;                   ///< 关节替换成球形关节？
};

/// 关于使用 OpenSim 创建的自定义负载的信息。
struct ForceInfo {
std::string type;                 ///< 负载类型如osim文件中所示
std::sharedptr<ChLoadBase> load ; ///< Chrono 加载对象
};

std::unordered_map<std::string, std::shared_ptr<ChBodyAuxRef>> bodies;
///< 身体信息列表
std::unordered_map<std::string, JointInfo> joints;
///< 关节信息列表
std::unordered_map<std::string, ForceInfo> forces;
///< 力列表的信息
```

1. 创建解析器对象；
2. 设置解析选项；
3. 调用解析方法之一。

清单 2 中的前两行创建了 `ChSystem` 来保存生成的模型，并指定了要解析的文件名。第三行创建了解析器对象；第四行将其设置为使用图元进行可视化（详见下文）。接下来的三行在解析器中设置标志，以确定可视化、接触和调试输出。默认行为是不显示可视化图像、不进行碰撞检测，也不输出详细信息。最后一行将 `filename` 指定的文件解析到 `my_system` 中。还有一个替代函数 `ChParserOpenSim::Parse`，它接受文件名和 `ChMaterialSurface::ContactMethod` 参数，以创建一个具有此接触类型的合适系统，将文件解析到该系统中，并返回指向新系统的指针。


### 3.7 报告

`ChParserOpenSim::Report` 类为用户提供了一个接口，用于访问从 `.osim` 文件解析出的刚体、关节和力。解析过程中会创建一个报告对象，用于以元素名称哈希的映射表形式存储 Chrono 刚体、关节和载荷列表。相关数据结构如清单 3 所示。`ChParserOpenSim::Report` 提供了用于打印报告以及按名称访问刚体、关节和载荷的方法。可以通过 `ChParserOpenSim::GetReport()` 访问解析器的报告。

### 3.8 当前限制

* 初始速度假定为零。
* 未识别的关节（包括自定义关节）被放置在正确的初始位置，但被替换为球形关节，从而导致不同的运动学。
* 自定义关节的移动器坐标系 (FXM(q)) 设置为恒等变换，从而产生不同的初始姿态。

## 4. 将来可能的扩展



### 4.1 ChParserOpenSim 文档



