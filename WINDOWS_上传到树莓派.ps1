param(
  [Parameter(Mandatory=$true)]
  [string]$PiIp,

  [string]$User = "pi"
)

$Project = $PSScriptRoot
$Target = "${User}@${PiIp}:~/"

Write-Host "Uploading project to $Target"
scp -r $Project $Target

Write-Host ""
Write-Host "Upload done. Next:"
Write-Host "ssh $User@$PiIp"
Write-Host "cd ~/raspi_robot_ai"
Write-Host "chmod +x scripts/*.sh"
Write-Host "bash scripts/setup_pi.sh"
