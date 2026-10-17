@echo off
chcp 65001 >nul
echo 启动智能小车仿真控制中心...

REM 检查依赖
python -c "import PyQt5" >nul 2>&1
if errorlevel 1 (
    echo 安装依赖...
    pip install PyQt5 websocket-client PyYAML
)

REM 启动 GUI
python main_window.py
