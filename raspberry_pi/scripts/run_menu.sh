#!/usr/bin/env bash
set -e

cd "$HOME/raspi_robot_ai"
source .venv/bin/activate
source scripts/pyenv.sh

while true; do
  clear
  echo "XiaoU Desktop Robot Arm AI - Raspberry Pi Menu"
  echo "========================================"
  echo "1. System check"
  echo "2. Camera test"
  echo "3. Object detection"
  echo "4. Cloud/local chat intent test"
  echo "5. Microphone record test"
  echo "6. Robot assistant main program"
  echo "8. Cloud AI chat"
  echo "9. Cloud AI chat + voice reply"
  echo "10. Chinese voice AI chat"
  echo "11. Baidu ASR test"
  echo "12. XiaoU integrated demo"
  echo "13. XiaoU face screen"
  echo "14. Audio+camera sanity check"
  echo "15. Audio device list"
  echo "16. Visual debug: small face window"
  echo "17. Audio debug: record and playback"
  echo "18. XiaoU voice dialog demo"
  echo "0. Exit"
  echo ""
  read -rp "Select: " choice
  case "$choice" in
    1) python robot_ai/00_system_check.py ;;
    2) python robot_ai/01_camera_test.py ;;
    3) python robot_ai/02_yolo_detect.py ;;
    4) python robot_ai/03_chat_test.py ;;
    5) python robot_ai/04_voice_record_test.py ;;
    6) bash scripts/run_demo_all.sh ;;
    8) python robot_ai/07_cloud_chat.py ;;
    9) python robot_ai/07_cloud_chat.py --voice ;;
    10) python robot_ai/08_voice_ai_chat.py ;;
    11) python robot_ai/09_baidu_asr_test.py ;;
    12) bash scripts/run_demo_all.sh ;;
    13) python robot_ai/face_display.py ;;
    14) bash scripts/run_av_sanity_check.sh ;;
    15) bash scripts/setup_audio_env.sh ;;
    16) bash scripts/run_visual_debug.sh ;;
    17) bash scripts/run_audio_debug.sh ;;
    18) bash scripts/run_xiaou_chat_yolo.sh ;;
    0) exit 0 ;;
    *) echo "Invalid choice." ;;
  esac
  echo ""
  read -rp "Press Enter to continue..."
done
