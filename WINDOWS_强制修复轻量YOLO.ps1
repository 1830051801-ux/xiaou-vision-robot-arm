param(
  [Parameter(Mandatory=$true)]
  [string]$PiIp,

  [string]$User = "pi"
)

$ErrorActionPreference = "Stop"
$Project = $PSScriptRoot
$Remote = "${User}@${PiIp}"
$RemoteProject = "~/raspi_robot_ai"

Write-Host "Fixing light YOLO files on Raspberry Pi: $Remote"

ssh $Remote "mkdir -p $RemoteProject/robot_ai $RemoteProject/scripts $RemoteProject/models"

scp "$Project\requirements.txt" "${Remote}:$RemoteProject/requirements.txt"
scp "$Project\config.env.example" "${Remote}:$RemoteProject/config.env.example"
scp "$Project\scripts\setup_pi.sh" "${Remote}:$RemoteProject/scripts/setup_pi.sh"
scp "$Project\robot_ai\02_yolo_detect.py" "${Remote}:$RemoteProject/robot_ai/02_yolo_detect.py"
scp "$Project\robot_ai\05_robot_assistant.py" "${Remote}:$RemoteProject/robot_ai/05_robot_assistant.py"
scp "$Project\robot_ai\07_cloud_chat.py" "${Remote}:$RemoteProject/robot_ai/07_cloud_chat.py"
scp "$Project\robot_ai\08_voice_ai_chat.py" "${Remote}:$RemoteProject/robot_ai/08_voice_ai_chat.py"
scp "$Project\robot_ai\09_baidu_asr_test.py" "${Remote}:$RemoteProject/robot_ai/09_baidu_asr_test.py"
scp "$Project\robot_ai\10_voice_robot_assistant.py" "${Remote}:$RemoteProject/robot_ai/10_voice_robot_assistant.py"
scp "$Project\robot_ai\baidu_asr.py" "${Remote}:$RemoteProject/robot_ai/baidu_asr.py"
scp "$Project\robot_ai\robot_ai_07_import.py" "${Remote}:$RemoteProject/robot_ai/robot_ai_07_import.py"
scp "$Project\robot_ai\vision_targeting.py" "${Remote}:$RemoteProject/robot_ai/vision_targeting.py"
scp "$Project\robot_ai\face_state.py" "${Remote}:$RemoteProject/robot_ai/face_state.py"
scp "$Project\robot_ai\emotion_state.py" "${Remote}:$RemoteProject/robot_ai/emotion_state.py"
scp "$Project\robot_ai\face_display.py" "${Remote}:$RemoteProject/robot_ai/face_display.py"
scp -r "$Project\robot_ai\emote_assets" "${Remote}:$RemoteProject/robot_ai/emote_assets"
scp "$Project\robot_ai\yolo_opencv.py" "${Remote}:$RemoteProject/robot_ai/yolo_opencv.py"
scp "$Project\scripts\run_cloud_chat.sh" "${Remote}:$RemoteProject/scripts/run_cloud_chat.sh"
scp "$Project\scripts\run_cloud_chat_voice.sh" "${Remote}:$RemoteProject/scripts/run_cloud_chat_voice.sh"
scp "$Project\scripts\run_voice_ai_chat.sh" "${Remote}:$RemoteProject/scripts/run_voice_ai_chat.sh"
scp "$Project\scripts\run_baidu_asr_test.sh" "${Remote}:$RemoteProject/scripts/run_baidu_asr_test.sh"
scp "$Project\scripts\run_voice_robot_assistant.sh" "${Remote}:$RemoteProject/scripts/run_voice_robot_assistant.sh"
scp "$Project\scripts\run_xiaou.sh" "${Remote}:$RemoteProject/scripts/run_xiaou.sh"
scp "$Project\scripts\run_face_screen.sh" "${Remote}:$RemoteProject/scripts/run_face_screen.sh"
scp "$Project\scripts\config_baidu_asr.sh" "${Remote}:$RemoteProject/scripts/config_baidu_asr.sh"
scp "$Project\scripts\config_robot_safety_defaults.sh" "${Remote}:$RemoteProject/scripts/config_robot_safety_defaults.sh"
scp "$Project\scripts\setup_pi5_fan_always_on.sh" "${Remote}:$RemoteProject/scripts/setup_pi5_fan_always_on.sh"
scp "$Project\scripts\install_pi5_fan_force_on_service.sh" "${Remote}:$RemoteProject/scripts/install_pi5_fan_force_on_service.sh"
scp "$Project\scripts\enable_passwordless_sudo.sh" "${Remote}:$RemoteProject/scripts/enable_passwordless_sudo.sh"
scp "$Project\scripts\run_menu.sh" "${Remote}:$RemoteProject/scripts/run_menu.sh"
scp "$Project\models\yolov5n.onnx" "${Remote}:$RemoteProject/models/yolov5n.onnx"
scp "$Project\models\yolov5s.onnx" "${Remote}:$RemoteProject/models/yolov5s.onnx"

ssh $Remote "cd $RemoteProject && rm -rf robot_ai/__pycache__ && chmod +x scripts/*.sh && bash scripts/config_robot_safety_defaults.sh && sed -i 's/YOLO_MODEL=.*/YOLO_MODEL=yolov5s.onnx/' config.env && sed -i 's/YOLO_IMAGE_SIZE=.*/YOLO_IMAGE_SIZE=640/' config.env && sed -i 's/YOLO_CONF=.*/YOLO_CONF=0.45/' config.env && sed -i 's/TARGET_OBJECTS=.*/TARGET_OBJECTS=cup,bottle,cell phone,book,keyboard,mouse/' config.env && grep -q '^BAIDU_ASR_API_KEY=' config.env || printf '\\nBAIDU_ASR_API_KEY=replace_with_baidu_api_key\\nBAIDU_ASR_SECRET_KEY=replace_with_baidu_secret_key\\nBAIDU_ASR_DEV_PID=1537\\nBAIDU_ASR_RATE=16000\\n' >> config.env && grep -q '^TTS_ENGINE=' config.env || printf '\\nTTS_ENGINE=local\\nTTS_VOICE=zh-CN-XiaoxiaoNeural\\nTTS_RATE=+0%%\\n' >> config.env && sed -i 's/TTS_ENGINE=.*/TTS_ENGINE=local/' config.env && echo '--- requirements.txt ---' && cat requirements.txt && echo '--- ultralytics check ---' && grep -R 'from ultralytics\|import ultralytics\|torch' -n requirements.txt robot_ai || true && echo '--- config ---' && grep 'YOLO_MODEL\\|YOLO_IMAGE_SIZE\\|YOLO_CONF\\|TARGET_OBJECTS\\|BAIDU_ASR\\|TTS_ENGINE\\|SERIAL_PROTOCOL\\|WORKSPACE\\|Z_SAFE\\|Z_GRAB' config.env && echo '--- model ---' && ls -lh models/yolov5s.onnx models/yolov5n.onnx"

Write-Host ""
Write-Host "Done. On Raspberry Pi run:"
Write-Host "cd ~/raspi_robot_ai"
Write-Host "bash scripts/run_yolo_test.sh"
