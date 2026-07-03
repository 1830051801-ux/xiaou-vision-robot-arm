# Raspberry Pi Setup

Run from the Raspberry Pi project directory:

```bash
cd ~/raspi_robot_ai
python3 -m venv --system-site-packages .venv
. .venv/bin/activate
pip install -r requirements.txt
cp config.env.example config.env
```

Edit `config.env` for the local camera, model and serial port.

Common run commands:

```bash
bash scripts/run_system_check.sh
bash scripts/run_camera_test.sh
bash scripts/run_yolo_test.sh
bash scripts/run_uart_test.sh
bash scripts/run_demo_all.sh --object pen
bash scripts/run_demo_all.sh --object pen --send-serial
```

`--send-serial` should only be used after the STM32 firmware is flashed, the arm is clear of obstacles, and UART wiring has been checked.

