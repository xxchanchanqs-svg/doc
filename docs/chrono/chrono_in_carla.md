# Carla 中的 Chrono 实现

Chrono 需要在虚幻引擎端启用 RTTI（RTTI（Run-Time Type Information），从而支持动态类型识别和异常处理等功能。
文件 [`Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Carla.Build.cs`](https://github.com/OpenHUTB/carla/blob/1b453d00cbf5e0afa00f3e993799c36c9286d75c/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Carla.Build.cs#L214) 中增加`bUseRTTI = true;`。

!!! 笔记
    每个.Build.cs文件声明派生自ModuleRules基类的类，并设置属性控制器从构造函数进行编译的方式。由虚幻编译工具编译，并被构造来确定整体编译环境。使用C#语法。


## 提交历史

在[commit页面](https://github.com/OpenHUTB/hutb/commits/hutb/?author=Axel1092&after=3225f8cfb1920f4ebf8a98a658385b379a04541b+174)按用户名[`Axel1092`](https://github.com/Axel1092)过滤。


* [Overloaded all UBaseCarlaMovementComponent functions in UChronoMovementComponent]()
* [Added JSON ingestion for chrono vehicle definition]()
* [Review fixes.]()
* [Set chrono version to 6.0.0.]()
* [Skipping 0 delta times.]()
* [Small fix for Windows compilation without chrono.]()
* [Fixed fall through bug for chrono vehicle.]()
* [Small fix to windows build]()
* [Added missing .lib from chrono to Build.cs]()
* [Updated chrono install for windows.]()
* [Added release compile mode for chrono library.]()
* [Added substep algorithm for chrono. Added initial UE4 terrain detection with raycast.]()
* [Added chrono compilation support for windows.](https://github.com/OpenHUTB/hutb/commit/14a634aa29f6ad66df3584a441adf00d9e019fa3)
* [Created base chrono implementation](https://github.com/OpenHUTB/hutb/commit/2117794dd7989b276317743b99430468628f5baa)
* [Added chrono library](https://github.com/OpenHUTB/hutb/commit/7c3367325fad6310202d40490081f946b5f363c9)



## 参考

* [Unreal 插件](https://www.jianshu.com/p/e41a810b10ca)