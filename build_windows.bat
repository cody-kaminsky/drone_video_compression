@echo off
REM ============================================================
REM  Build h265_decoder on Windows using vcpkg + MSVC (cl.exe)
REM  OR using MSYS2 / MinGW-w64 (gcc)
REM ============================================================

REM --- Option A: MSYS2 / MinGW-w64 (recommended, easiest) ----
REM 1. Install MSYS2 from https://www.msys2.org/
REM 2. In MSYS2 MINGW64 shell run:
REM      pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-ffmpeg
REM 3. Then in that same shell:
REM      gcc -O2 -o h265_decoder.exe h265_decoder.c ^
REM          $(pkg-config --cflags --libs libavcodec libavformat libavutil libswscale)

REM --- Option B: vcpkg + cl.exe (MSVC) -----------------------
REM 1. Install vcpkg:  https://vcpkg.io/en/getting-started
REM 2. Run:  vcpkg install ffmpeg:x64-windows
REM 3. Set VCPKG_ROOT to your vcpkg directory, then run this script.

IF "%VCPKG_ROOT%"=="" (
    echo ERROR: VCPKG_ROOT is not set. Please set it to your vcpkg directory.
    exit /b 1
)

SET FFMPEG_INC=%VCPKG_ROOT%\installed\x64-windows\include
SET FFMPEG_LIB=%VCPKG_ROOT%\installed\x64-windows\lib

cl.exe /O2 /W3 ^
    /I"%FFMPEG_INC%" ^
    h265_decoder.c ^
    /link ^
    /LIBPATH:"%FFMPEG_LIB%" ^
    avcodec.lib avformat.lib avutil.lib swscale.lib ^
    /OUT:h265_decoder.exe

IF %ERRORLEVEL% EQU 0 (
    echo Build succeeded: h265_decoder.exe
) ELSE (
    echo Build failed.
    exit /b 1
)
