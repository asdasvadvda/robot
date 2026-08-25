@echo off
rem ============================================================
rem  ESKF unified build script (ASCII comments only, bat files
rem  are decoded in the system codepage, Chinese UTF-8 breaks it)
rem  Usage:  build.cmd source.cpp  ->  source.exe
rem  Similar to Keil CLI:  UV4.exe -b project.uvprojx
rem ============================================================
cd /d "%~dp0"
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
cl /nologo /std:c++20 /EHsc /O2 /utf-8 /I third_party\eigen %*
