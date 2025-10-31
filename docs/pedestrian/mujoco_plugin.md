# Mujoco 的插件



## 调试

### 调试模块加载

* 使用 VS 2022 打开 CarlaUE.sln；
* 在 [MuJoCoUE.cpp](https://github.com/OpenHUTB/mujoco_plugin/blob/main/Plugins/MuJoCoUE/Source/MuJoCoUE/Private/MuJoCoUE.cpp) 中的模块初始化函数 `StartupModule()`内打断点；
* 点击`调试`菜单中的启动`调试`；
* 启动引擎编辑器的过程中会在断点处停止。


### 调试协同仿真

* 在 [MuJoCoSimulation.cpp](https://github.com/OpenHUTB/mujoco_plugin/blob/main/Plugins/MuJoCoUE/Source/MuJoCoUE/Private/MuJoCoSimulation.cpp) 中的场景开始函数`BeginPlay()`内打断点；
* 在引擎编辑器中点击`运行`后会在断点处停止。
