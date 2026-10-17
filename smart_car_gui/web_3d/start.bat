@echo off
chcp 65001 >nul
echo ═══════════════════════════════════════════
echo   智能小车 3D Web 可视化
echo ═══════════════════════════════════════════
echo.

REM 检查 Python
python --version >nul 2>&1
if errorlevel 1 (
    echo [错误] 未找到 Python，请先安装 Python 3.8+
    pause
    exit /b 1
)

REM 检查依赖
echo [1/3] 检查依赖...
python -c "import websockets" 2>nul
if errorlevel 1 (
    echo [提示] 正在安装 websockets...
    python -m pip install websockets -q
)

python -c "import aiohttp" 2>nul
if errorlevel 1 (
    echo [提示] 正在安装 aiohttp...
    python -m pip install aiohttp -q
)

echo [2/3] 依赖检查完成
echo.

REM 获取端口号
set PORT=8080
if not "%1"=="" set PORT=%1

echo [3/3] 启动服务器...
echo.
echo ═══════════════════════════════════════════
echo   HTTP 地址: http://localhost:%PORT%
echo   按 Ctrl+C 停止服务器
echo ═══════════════════════════════════════════
echo.

REM 启动服务器
python "%~dp0server.py" %PORT%

pause
