@echo off
REM AVBD Demo 3D - Windows Build Script
REM This script automates the complete build process for Windows

echo === AVBD Demo 3D - Windows Build Script ===

REM Initialize submodules
echo Initializing git submodules...
git submodule update --init --recursive
if %errorlevel% neq 0 (
    echo Error: Failed to initialize submodules
    pause
    exit /b 1
)

REM Clone and bootstrap vcpkg if it doesn't exist
if not exist vcpkg (
    echo Cloning vcpkg...
    git clone https://github.com/microsoft/vcpkg.git
    if %errorlevel% neq 0 (
        echo Error: Failed to clone vcpkg
        pause
        exit /b 1
    )
)

echo Bootstrapping vcpkg...
call vcpkg\bootstrap-vcpkg.bat
if %errorlevel% neq 0 (
    echo Error: Failed to bootstrap vcpkg
    pause
    exit /b 1
)

REM Set VCPKG_ROOT environment variable
set VCPKG_ROOT=%cd%\vcpkg

REM Install dependencies
echo Installing dependencies via vcpkg...
vcpkg\vcpkg.exe install opengl glad
if %errorlevel% neq 0 (
    echo Error: Failed to install dependencies
    pause
    exit /b 1
)

REM Create build directory
echo Creating build directory...
if exist build rmdir /s /q build
mkdir build
cd build

REM Configure and build
echo Configuring with CMake...
cmake .. -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake
if %errorlevel% neq 0 (
    echo Error: CMake configuration failed
    pause
    exit /b 1
)

echo Building...
cmake --build . --config Release --verbose
if %errorlevel% neq 0 (
    echo Error: Build failed
    pause
    exit /b 1
)

echo === Build Complete ===
echo Executable: %cd%\Release\avbd_demo3d.exe
echo Run with: build\Release\avbd_demo3d.exe

REM Package a release-ready zip: exe + SDL2.dll (fixes issue #8 - "SDL2.dll not found")
REM The pre-release failed because only the .exe was uploaded without its SDL2 runtime.
echo Packaging Windows release zip...
set PKGDIR=%cd%\package
if exist "%PKGDIR%" rmdir /s /q "%PKGDIR%"
mkdir "%PKGDIR%"
copy /y Release\avbd_demo3d.exe "%PKGDIR%\" >nul
copy /y Release\SDL2.dll "%PKGDIR%\" >nul
if not exist "%PKGDIR%\SDL2.dll" (
    echo WARNING: SDL2.dll was not found next to the executable. The zip will be missing it.
) else (
    echo SDL2.dll bundled into the package.
)
powershell -NoProfile -Command "Compress-Archive -Force -Path '%PKGDIR%\*' -DestinationPath '%cd%\avbd-demo3d-windows.zip'"
echo Release package: %cd%\avbd-demo3d-windows.zip
echo Upload this zip (not just the .exe) as the Windows pre-release asset.

pause