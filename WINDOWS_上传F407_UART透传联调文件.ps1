param(
    [string]$PiIp = "172.20.10.3",
    [string]$User = "pi",
    [string]$RemoteDir = "/home/pi/raspi_robot_ai",
    [switch]$SkipChecks
)

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$Target = $User + "@" + $PiIp

$Files = @(
    "robot_ai/arm_control/uart_protocol.py",
    "robot_ai/arm_control/task_planner.py",
    "robot_ai/arm_control/safety.py",
    "robot_ai/arm_control/config/hardware_calibration.json",
    "robot_ai/arm_control/config/object_grasp_profiles.json",
    "robot_ai/vision_targeting.py",
    "robot_ai/preflight_check.py",
    "codex_pickup_package/create_workspace_homography.py",
    "docs/f407_uart_passthrough_integration_20260807.md",
    "tests/test_uart_protocol.py",
    "tests/test_task_planner.py",
    "tests/test_arm_control.py"
)

Set-Location $ProjectRoot
Write-Host ("[upload] F407 UART protocol files -> " + $Target + ":" + $RemoteDir)
Write-Host "[safety] This script only copies source/tests and runs offline Python checks."
Write-Host "[safety] It does not open /dev/serial0, enable CAN, send motion, or change hardware gates."

ssh $Target ("mkdir -p '" + $RemoteDir + "/robot_ai/arm_control/config' '" + $RemoteDir + "/codex_pickup_package' '" + $RemoteDir + "/docs' '" + $RemoteDir + "/tests'")
if ($LASTEXITCODE -ne 0) {
    throw "Cannot create target directories on $Target"
}

foreach ($file in $Files) {
    $localPath = Join-Path $ProjectRoot $file
    if (!(Test-Path -LiteralPath $localPath)) {
        throw "Missing local file: $file"
    }
    $remotePath = $RemoteDir + "/" + ($file -replace "\\", "/")
    Write-Host ("[upload] " + $file)
    scp $localPath ($Target + ":" + $remotePath)
    if ($LASTEXITCODE -ne 0) {
        throw "Upload failed: $file"
    }
}

if (!$SkipChecks) {
    Write-Host "[verify] compile and unit-test only; no serial device is opened"
    ssh $Target ("cd '" + $RemoteDir + "' && python3 -m compileall -q robot_ai tests && python3 -m unittest discover -s tests -v")
    if ($LASTEXITCODE -ne 0) {
        throw "Pi offline verification failed"
    }
}

Write-Host "[done] Upload completed. Run the safe PING manually only after F407 is built/flashed with the actuator gate still locked."
