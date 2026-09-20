param(
    [string]$PiIp = "100.127.160.77",
    [string]$User = "pi",
    [string]$RemoteDir = "~/raspi_robot_ai"
)

$ErrorActionPreference = "Stop"

$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $Root

$Files = @(
    "robot_ai/emotion_state.py",
    "robot_ai/xiaou_gui_demo.py",
    "robot_ai/07_cloud_chat.py",
    "robot_ai/vision_targeting.py",
    "robot_ai/preflight_check.py",
    "config.demo.env",
    "docs/six_axis_preflight.md",
    "tests/test_workspace_homography.py",
    "scripts/check_project.sh",
    "scripts/run_demo_all.sh"
)

$Target = "${User}@${PiIp}"

Write-Host "[upload] target: ${Target}:$RemoteDir"
Write-Host "[upload] files:"
$Files | ForEach-Object { Write-Host "  - $_" }

ssh $Target "cd $RemoteDir && mkdir -p robot_ai docs runtime tests scripts codex_pickup_package"

foreach ($file in $Files) {
    if (!(Test-Path -Path $file)) {
        throw "Missing local file: $file"
    }
    $remotePath = "$RemoteDir/$file"
    Write-Host "[upload] $file -> $remotePath"
    scp "$file" "${Target}:$remotePath"
}

$Calibration = "codex_pickup_package/workspace_homography.yaml"
ssh $Target "test -f $RemoteDir/$Calibration"
if ($LASTEXITCODE -ne 0) {
    Write-Host "[upload] calibration missing on Pi; uploading saved 9-point result"
    scp $Calibration "${Target}:$RemoteDir/$Calibration"
} else {
    Write-Host "[keep] Pi calibration already exists; not overwritten"
}

Write-Host "[verify] compile python files on Pi"
ssh $Target "cd $RemoteDir && chmod +x scripts/check_project.sh scripts/run_demo_all.sh && source .venv/bin/activate && python -m compileall -q robot_ai tests && python -m unittest discover -s tests -v && python robot_ai/preflight_check.py"

Write-Host "[done] XiaoU perception and six-axis planning files uploaded; offline tests passed."
