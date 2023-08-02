@echo off

REM Get the absolute path to the directory containing this script
set SCRIPT_DIR=%~dp0

REM Replace backslashes with forward slashes in the path
set SCRIPT_DIR=%SCRIPT_DIR:\=/%

REM Build the Docker image from the Dockerfile in the current directory
docker build -t sbmpo:ros2 -f "%SCRIPT_DIR%/docker/Dockerfile.sbmpo-ros2" "%SCRIPT_DIR%"

REM Start the Docker container with the current directory mounted, and automatically remove the container when it is stopped or exited
docker run --rm -it ^
    --mount type=bind,source="%SCRIPT_DIR%",target=/ros2_ws/src/ sbmpo:ros2
