# PyChrono

## [安装](https://api.projectchrono.org/pychrono_installation.html)

1. 初始化虚拟环境
```shell
conda config --add channels https://conda.anaconda.org/conda-forge
conda create -n chrono python=3.10
conda activate chrono
```

2. 安装必要的依赖

```shell
# 使用Pardiso直接稀疏线性求解器进行PyChrono演示、Numpy以及 PythonOCC 时所需
conda install -c conda-forge mkl=2020
# Chrono::Sensor 模块
conda install -c conda-forge numpy=1.24.0
# 运行时的可视化
conda install -c conda-forge irrlicht=1.8.5
# Chrono::Cascade 支持
conda install -c conda-forge pythonocc-core=7.9.0
# 图形数据
conda install conda-forge::gnuplot
# 需要英伟达图形驱动 515.xx+
conda install -c nvidia/label/cuda-12.3.0 cuda-toolkit
conda install -c conda-forge glfw
```

3. [下载](https://anaconda.org/projectchrono/pychrono/files) 对应的版本，比如：
```shell
wget https://anaconda.org/projectchrono/pychrono/7.0.0/download/win-64/pychrono-7.0.0-py310_2455.tar.bz2
```

4. 安装
```shell
conda install <pychrono_package>.tar.bz2
```

