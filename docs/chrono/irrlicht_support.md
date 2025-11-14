# irrlicht 支持

添加可视化组件的支持。

## 单独编译 chrono：

安装[blaze](https://bitbucket.org/blaze-lib/blaze) 3.8。



先在`install_chrono.bat`中启用
```shell
-DENABLE_MODULE_MULTICORE=ON^
-DENABLE_MODULE_IRRLICHT=ON^
```


```shell
# 重命名 D:\hutb\Build\chrono-install
call D:\hutb\Util\InstallersWin\install_chrono.bat ^
--build-dir "D:\hutb\Build\" ^
--generator "Visual Studio 17 2022"
```
