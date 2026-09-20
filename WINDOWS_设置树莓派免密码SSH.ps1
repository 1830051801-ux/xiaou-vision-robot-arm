param(
  [Parameter(Mandatory=$true)]
  [string]$PiIp,

  [string]$User = "pi"
)

Write-Host "Passwordless SSH setup has been disabled."
Write-Host "Use normal SSH password login instead:"
Write-Host "ssh ${User}@${PiIp}"
Write-Host ""
Write-Host "Project upload scripts will still work; just enter the Raspberry Pi password when prompted."
exit 0
