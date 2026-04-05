#Requires -Version 5.1
param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]] $Passthrough
)
<#
  Windows launcher: PATH for ORB_SLAM3.dll transitive deps (Boost, OpenSSL, GLEW from vcpkg / Pangolin prefix).
  Mirrors run_live_wsl.sh: export YAML from camchain, then orbslam3_stereo_live.

  Env overrides (optional):
    CAMCHAIN, SETTINGS_YAML, RECTIFY_YAML - paths
    ORB_LIVE_LEFT, ORB_LIVE_RIGHT - camera indices (default 2 / 0)
    ORB_LIVE_SWAP=1 - swap left/right for this run
    WIDTH, HEIGHT, FPS, FOURCC, VIEWER, SHOW
    EXPORT_ONLY=1 - only run export_orbslam3_config.py

  vcpkg in a path with spaces: install Pangolin/OpenCV from a subst drive (e.g. subst T: repo_root).
#>
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$ToolsDir = $PSScriptRoot
$ReleaseDir = Join-Path $ToolsDir "build_win\Release"
$Exe = Join-Path $ReleaseDir "orbslam3_stereo_live.exe"
$VcpkgBin = Join-Path $Root "tools\vendor\vcpkg\installed\x64-windows\bin"
$PangolinBin = Join-Path $ToolsDir "vcpkg_installed\x64-windows\bin"
$OrbRoot = Join-Path $Root "tools\vendor\ORB_SLAM3"
$VocabDefault = Join-Path $OrbRoot "Vocabulary\ORBvoc.txt"

$Camchain = if ($env:CAMCHAIN) { $env:CAMCHAIN } else { Join-Path $Root "kalibr_data\my_calib_75mm-camchain.yaml" }
$SettingsYaml = if ($env:SETTINGS_YAML) { $env:SETTINGS_YAML } else { Join-Path $Root "kalibr_data\my_calib_75mm-orbslam3-stereo-rectified.yaml" }
$RectifyYaml = if ($env:RECTIFY_YAML) { $env:RECTIFY_YAML } else { Join-Path $Root "kalibr_data\my_calib_75mm-orbslam3-rectify-input.yaml" }

$left = if ($env:ORB_LIVE_LEFT) { $env:ORB_LIVE_LEFT } else { "2" }
$right = if ($env:ORB_LIVE_RIGHT) { $env:ORB_LIVE_RIGHT } else { "0" }
if ($env:ORB_LIVE_SWAP -eq "1") {
    $t = $left; $left = $right; $right = $t
}

$width = if ($env:WIDTH) { $env:WIDTH } else { "800" }
$height = if ($env:HEIGHT) { $env:HEIGHT } else { "600" }
$fps = if ($env:FPS) { $env:FPS } else { "20" }
$fourcc = if ($env:FOURCC) { $env:FOURCC } else { "MJPG" }
$viewer = if ($null -ne $env:VIEWER) { $env:VIEWER } else { "1" }
$show = if ($null -ne $env:SHOW) { $env:SHOW } else { "1" }
$autoLevels = if ($null -ne $env:ORB_LIVE_AUTO_LEVELS) { $env:ORB_LIVE_AUTO_LEVELS } else { "0" }
$autoTail = if ($env:ORB_LIVE_AUTO_TAIL) { $env:ORB_LIVE_AUTO_TAIL } else { "0.02" }
$uvcAuto = if ($null -ne $env:ORB_LIVE_UVC_AUTO) { $env:ORB_LIVE_UVC_AUTO } else { "0" }

$vocab = if ($env:ORB_SLAM3_VOCAB) { $env:ORB_SLAM3_VOCAB } else { $VocabDefault }

if (-not (Test-Path $Exe)) {
    Write-Error "Missing $Exe - run tools/orb_slam3/build_windows.ps1 first."
}
foreach ($d in @($ReleaseDir, $VcpkgBin, $PangolinBin)) {
    if (Test-Path $d) {
        $env:PATH = "$d;$env:PATH"
    }
}

$exportArgs = @(
    (Join-Path $ToolsDir "export_orbslam3_config.py"),
    "--camchain", $Camchain,
    "--output-settings", $SettingsYaml,
    "--output-rectify", $RectifyYaml,
    "--fps", $fps
)
if ($env:CAPTURE_W -and $env:CAPTURE_H) {
    $exportArgs += @("--capture-width", $env:CAPTURE_W, "--capture-height", $env:CAPTURE_H)
}

Write-Host "Stereo: left=$left right=$right"
& python @exportArgs
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

if ($env:EXPORT_ONLY -eq "1") {
    Write-Host "EXPORT_ONLY=1 - skipping live binary."
    exit 0
}

if (-not (Test-Path $vocab)) {
    Write-Error "Vocabulary not found: $vocab (set ORB_SLAM3_VOCAB or extract ORBvoc.txt under ORB_SLAM3/Vocabulary)"
}

& $Exe `
    --vocab $vocab `
    --settings $SettingsYaml `
    --rectify $RectifyYaml `
    --device-left $left `
    --device-right $right `
    --width $width `
    --height $height `
    --fps $fps `
    --fourcc $fourcc `
    --auto-levels $autoLevels `
    --auto-levels-tail $autoTail `
    --uvc-auto $uvcAuto `
    --viewer $viewer `
    --show $show `
    @Passthrough

exit $LASTEXITCODE
