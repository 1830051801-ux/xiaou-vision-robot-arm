codex_deskpet_package

Features:
- face_greet.py: camera face detection. When a nearby face is detected, XiaoU greets, changes expression, logs event.
- scheduler.py: lightweight reminder/schedule system.
- run_demo.py: runs face greeting + scheduler together.
- run_tests.py: automated non-camera tests -> codex_deskpet_report.json.

Run tests:
  cd ~/raspi_robot_ai/codex_deskpet_package
  python3 run_tests.py

Run face greeting:
  cd ~/raspi_robot_ai/codex_deskpet_package
  python3 face_greet.py --camera 0 --cooldown 8 --min-area 6500

Add a reminder:
  cd ~/raspi_robot_ai/codex_deskpet_package
  python3 scheduler.py --add --in 60 --message "该喝水啦" --emote thinking

Run scheduler loop:
  python3 scheduler.py --run

Run both:
  python3 run_demo.py

Notes:
- DESKPET_TTS=false by default, so tests do not require a speaker.
- Set DESKPET_TTS=true if espeak/spd-say speaker output is installed.
- The package writes existing XiaoU face state to ../runtime/face_state.json, so it works with robot_ai/face_display.py.
