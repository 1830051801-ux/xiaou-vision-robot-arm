param(
  [Parameter(Mandatory=$true)]
  [string]$PiIp,

  [string]$User = "pi"
)

$ErrorActionPreference = "Stop"
$Project = $PSScriptRoot
$Remote = "${User}@${PiIp}"
$RemoteProject = "~/raspi_robot_ai"

Write-Host "Uploading XiaoU face display files to $Remote"

ssh $Remote "mkdir -p $RemoteProject/robot_ai $RemoteProject/scripts"
scp "$Project\robot_ai\face_display.py" "${Remote}:$RemoteProject/robot_ai/face_display.py"
scp "$Project\robot_ai\device_runtime.py" "${Remote}:$RemoteProject/robot_ai/device_runtime.py"
scp "$Project\robot_ai\face_state.py" "${Remote}:$RemoteProject/robot_ai/face_state.py"
scp "$Project\robot_ai\emotion_state.py" "${Remote}:$RemoteProject/robot_ai/emotion_state.py"
scp "$Project\robot_ai\dialog_emote_bridge.py" "${Remote}:$RemoteProject/robot_ai/dialog_emote_bridge.py"
scp "$Project\robot_ai\07_cloud_chat.py" "${Remote}:$RemoteProject/robot_ai/07_cloud_chat.py"
scp "$Project\robot_ai\08_voice_ai_chat.py" "${Remote}:$RemoteProject/robot_ai/08_voice_ai_chat.py"
scp "$Project\robot_ai\10_voice_robot_assistant.py" "${Remote}:$RemoteProject/robot_ai/10_voice_robot_assistant.py"
scp "$Project\robot_ai\11_xiaou_integrated_test.py" "${Remote}:$RemoteProject/robot_ai/11_xiaou_integrated_test.py"
scp "$Project\robot_ai\xiaou_demo_main.py" "${Remote}:$RemoteProject/robot_ai/xiaou_demo_main.py"
scp "$Project\robot_ai\vision_targeting.py" "${Remote}:$RemoteProject/robot_ai/vision_targeting.py"
scp "$Project\config.demo.env" "${Remote}:$RemoteProject/config.demo.env"
scp -r "$Project\robot_ai\emote_assets" "${Remote}:$RemoteProject/robot_ai/emote_assets"
scp -r "$Project\codex_pickup_package" "${Remote}:$RemoteProject/codex_pickup_package"
scp -r "$Project\codex_emote_ai_package" "${Remote}:$RemoteProject/codex_emote_ai_package"
scp -r "$Project\codex_deskpet_package" "${Remote}:$RemoteProject/codex_deskpet_package"
scp "$Project\scripts\run_face_screen.sh" "${Remote}:$RemoteProject/scripts/run_face_screen.sh"
scp "$Project\scripts\export_face_debug_frame.sh" "${Remote}:$RemoteProject/scripts/export_face_debug_frame.sh"
scp "$Project\scripts\run_visual_debug.sh" "${Remote}:$RemoteProject/scripts/run_visual_debug.sh"
scp "$Project\scripts\run_integrated_test.sh" "${Remote}:$RemoteProject/scripts/run_integrated_test.sh"
scp "$Project\scripts\run_xiaou_with_face.sh" "${Remote}:$RemoteProject/scripts/run_xiaou_with_face.sh"
scp "$Project\scripts\run_deskpet.sh" "${Remote}:$RemoteProject/scripts/run_deskpet.sh"
scp "$Project\scripts\run_face_greet.sh" "${Remote}:$RemoteProject/scripts/run_face_greet.sh"
scp "$Project\scripts\run_scheduler.sh" "${Remote}:$RemoteProject/scripts/run_scheduler.sh"
scp "$Project\scripts\run_demo_all.sh" "${Remote}:$RemoteProject/scripts/run_demo_all.sh"
scp "$Project\scripts\install_demo_user_service.sh" "${Remote}:$RemoteProject/scripts/install_demo_user_service.sh"

ssh $Remote "cd $RemoteProject && chmod +x scripts/*.sh && sudo apt install -y python3-tk python3-pil python3-pil.imagetk"

Write-Host ""
Write-Host "Done. On Raspberry Pi run:"
Write-Host "cd ~/raspi_robot_ai"
Write-Host "bash scripts/run_face_screen.sh"
Write-Host "bash scripts/run_visual_debug.sh"
Write-Host "bash scripts/run_integrated_test.sh"
Write-Host "bash scripts/run_xiaou_with_face.sh"
Write-Host "bash scripts/run_deskpet.sh"
Write-Host "bash scripts/run_demo_all.sh"
