@echo off
chcp 65001 >nul
echo ============================================
echo   智能小车仿真控制中心 - 构建脚本
echo ============================================
echo.

REM 检查 Python
python --version >nul 2>&1
if errorlevel 1 (
    echo [错误] 未找到 Python，请先安装 Python 3.8+
    pause
    exit /b 1
)

REM 安装依赖
echo [1/3] 安装依赖...
pip install -r requirements.txt
if errorlevel 1 (
    echo [错误] 依赖安装失败
    pause
    exit /b 1
)

echo.
echo [2/3] 构建 EXE...
pyinstaller --noconfirm --onedir --windowed ^
    --name "SmartCarSimCenter" ^
    --add-data "wsl_bridge;wsl_bridge" ^
    --hidden-import websocket ^
    --hidden-import yaml ^
    --hidden-import PyQt5.sip ^
    main_window.py

if errorlevel 1 (
    echo [错误] 构建失败
    pause
    exit /b 1
)

echo.
echo [3/3] 构建完成!
echo.
echo EXE 位置: dist\SmartCarSimCenter\SmartCarSimCenter.exe
echo.
echo 使用方法:
echo   1. 确保 WSL2 已安装并配置好 ROS2 环境
echo   2. 确保 v2x_ws 工作空间已构建
echo   3. 运行 SmartCarSimCenter.exe
echo.
pause
