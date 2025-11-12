@echo off
setlocal enabledelayedexpansion

REM ============================================================
REM Check if Docker is running
REM ============================================================
docker info >nul 2>&1
if errorlevel 1 (
    echo Error: Docker Desktop is not running or not accessible.
    echo Please start Docker Desktop and try again.
    exit /b 1
)

REM ============================================================
REM Define the Summit image
REM ============================================================
set SUMMIT_IMAGE=summit:latest

REM ============================================================
REM Check if Docker image exists, build if missing
REM ============================================================
docker image inspect %SUMMIT_IMAGE% >nul 2>&1
if errorlevel 1 (
    echo Docker image %SUMMIT_IMAGE% not found.
    echo Building Docker image %SUMMIT_IMAGE%...
    docker build -t %SUMMIT_IMAGE% -f docker/Dockerfile .
    if errorlevel 1 (
        echo Failed to build Docker image %SUMMIT_IMAGE%. Exiting.
        exit /b 1
    )
) else (
    echo Docker image %SUMMIT_IMAGE% found.
)

REM ============================================================
REM Run Docker container
REM NOTE: Adjust the -v path to match your Windows folder
REM ============================================================
docker run -it --rm ^
    -e DISPLAY=host.docker.internal:0.0 ^
    -e QT_X11_NO_MITSHM=1 ^
    -v C:/Users/pierr/Documents/P7/SafeReinforcementLearning:/home/ros_workspace ^
    --name summit ^
    summit:latest ^
    bash --login -c "source /opt/ros/melodic/setup.bash && cd /home/ros_workspace && bash"

set DOCKER_EXIT_STATUS=%ERRORLEVEL%

REM ============================================================
REM Check exit status
REM ============================================================
if %DOCKER_EXIT_STATUS% neq 0 (
    echo Docker run failed with exit status %DOCKER_EXIT_STATUS%. Exiting.
    exit /b %DOCKER_EXIT_STATUS%
)

endlocal
