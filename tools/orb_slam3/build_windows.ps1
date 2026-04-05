#Requires -Version 5.1
<#
  One-shot build: vcpkg deps (if missing), ORB-SLAM3 (patched vendor copy), orbslam3_stereo_live.
  Prereqs: VS 2022 Build Tools (Desktop C++), CMake on PATH (winget: Kitware.CMake).

  Repo layout:
    tools/vendor/vcpkg
    tools/vendor/ORB_SLAM3
#>
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$VcpkgRoot = Join-Path $Root "tools\vendor\vcpkg"
$OrbRoot = Join-Path $Root "tools\vendor\ORB_SLAM3"
$VsDevCmd = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat"
if (-not (Test-Path $VsDevCmd)) {
  Write-Error "VsDevCmd.bat not found at $VsDevCmd. Install VS 2022 Build Tools with C++ workload."
}

$Toolchain = Join-Path $VcpkgRoot "scripts\buildsystems\vcpkg.cmake"
if (-not (Test-Path $Toolchain)) {
  Write-Error "vcpkg toolchain missing. Run bootstrap in tools\vendor\vcpkg first."
}

$VocabTxt = Join-Path $OrbRoot "Vocabulary\ORBvoc.txt"
if (-not (Test-Path $VocabTxt)) {
  $TarGz = Join-Path $OrbRoot "Vocabulary\ORBvoc.txt.tar.gz"
  if (Test-Path $TarGz) {
    Write-Host "Extracting ORB vocabulary..."
    Push-Location (Join-Path $OrbRoot "Vocabulary")
    try { tar -xf ORBvoc.txt.tar.gz } finally { Pop-Location }
  }
}

$OrbBuild = Join-Path $OrbRoot "build_win"
$LiveBuild = Join-Path $PSScriptRoot "build_win"

$cmd = @"
call "$VsDevCmd" -arch=x64 &&
set "VCPKG_ROOT=$VcpkgRoot" &&
cd /d "$OrbRoot" &&
if not exist "$OrbBuild" mkdir "$OrbBuild" &&
cd /d "$OrbBuild" &&
cmake -G "Visual Studio 17 2022" -A x64 ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DCMAKE_TOOLCHAIN_FILE=$Toolchain ^
  -DVCPKG_TARGET_TRIPLET=x64-windows ^
  .. &&
cmake --build . --config Release --parallel &&
cd /d "$PSScriptRoot" &&
if not exist "$LiveBuild" mkdir "$LiveBuild" &&
cd /d "$LiveBuild" &&
cmake -G "Visual Studio 17 2022" -A x64 ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DCMAKE_TOOLCHAIN_FILE=$Toolchain ^
  -DVCPKG_TARGET_TRIPLET=x64-windows ^
  -DORB_SLAM3_ROOT="$OrbRoot" ^
  .. &&
cmake --build . --config Release --parallel &&
echo BUILD_OK
"@

cmd /c $cmd
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
Write-Host "orbslam3_stereo_live.exe -> $LiveBuild\Release\orbslam3_stereo_live.exe"
Write-Host "Run live (sets PATH for Boost/OpenSSL/GLEW): .\run_live_windows.ps1"
