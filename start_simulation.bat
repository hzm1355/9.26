@echo off
echo ============================================================
echo    ROS2 Smart Car Simulation - Windows Launcher
echo ============================================================
echo.

REM Check WSL
wsl --status >nul 2>&1
if errorlevel 1 (
    echo [ERROR] WSL not detected. Please install WSL2 first.
    echo See: https://docs.microsoft.com/windows/wsl/install
    pause
    exit /b 1
)

REM Check command
if "%1"=="" (
    echo Usage: start_simulation.bat [command] [args]
    echo.
    echo Commands:
    echo   start              Start simulation
    echo   start --nav2       Start with Nav2
    echo   start --no-gui     Headless mode
    echo   status             Show status
    echo   config             Show config
    echo   fault ^<car^> ^<type^>  Inject fault
    echo   record start/stop  Record trajectory
    echo   playback ^<file^>    Play back trajectory
    echo   info               Show help
    echo.
    echo Examples:
    echo   start_simulation.bat start
    echo   start_simulation.bat start --nav2
    pause
    exit /b 0
)

echo [START] Running in WSL: python3 smart_car_app.py %*
echo.

REM Execute in WSL with proper command passing
wsl bash -c "cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim && source /opt/ros/humble/setup.bash && source install/setup.bash 2>/dev/null; python3 smart_car_app.py %*"

if errorlevel 1 (
    echo.
    echo [ERROR] Command failed
    pause
)
