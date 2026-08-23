Purpose:
  Minimal pickup pipeline for the Raspberry Pi side.

Files:
  create_charuco_board.py      generate charuco_board.png for printing
  capture_calib_images.py      capture calibration images from camera
  camera_calibration_charuco.py  Charuco camera calibration -> camera.yaml
  camera_calibration_chessboard.py  normal chessboard calibration -> camera.yaml
  create_workspace_homography.py    4-point pixel -> robot-base XY calibration
  test_workspace_homography.py      test one pixel point after 4-point calibration
  generate_grasp.py             bbox + camera.yaml -> camera-frame pose
  kalman_filter.py              simple 1D Kalman filter
  filter_and_publish.py         /tmp/det.json -> /tmp/det_filtered.json
  reachability_check.py         basic workspace check
  behavior_simple.py            simple orchestrator

One-line smoke test:
  python3 -m pip install --user -r requirements.txt && python3 camera_calibration_charuco.py --images "/path/to/calib/*.jpg" --squaresX 5 --squaresY 7 --square_length 0.03 --marker_length 0.02 --output camera.yaml || echo "skip calib"; echo '{"name":"cup","bbox":[120,80,200,240],"image":"/tmp/frame.jpg"}' > /tmp/det.json && python3 filter_and_publish.py && python3 behavior_simple.py

Notes:
  cam_h in behavior_simple.py must be measured on the real machine.
  If camera.yaml is not ready, behavior_simple.py will not send a real pose.
  MCU should only accept frames that pass header, checksum and tail validation.

Camera calibration test:
  python3 create_charuco_board.py --out charuco_board.png
  Print charuco_board.png or show it on a flat screen.
  python3 capture_calib_images.py --camera 0 --outdir calib_images
  Press s to save 15-25 images from different angles.
  python3 camera_calibration_charuco.py --images "calib_images/*.jpg" --squaresX 8 --squaresY 10 --square_length 0.01 --marker_length 0.005 --output camera.yaml

Normal chessboard calibration:
  python3 camera_calibration_chessboard.py --images "calib_images/*.jpg" --cols 7 --rows 9 --square_size 0.01 --output camera.yaml

4-point desktop coordinate calibration:
  1. Keep camera fixed.
  2. Put 4 marks on the desktop.
  3. Record each mark's camera pixel coordinate u,v.
  4. Move robot/gripper to the same 4 marks and record robot base X,Y in mm.
  5. Use the same point order in both lists, for example: left_top right_top right_bottom left_bottom.

Example:
  python3 create_workspace_homography.py --pixels 120 100 520 100 520 400 120 400 --base_mm 120 80 260 80 260 -80 120 -80 --output workspace_homography.yaml

Test:
  python3 test_workspace_homography.py --workspace workspace_homography.yaml --u 319 --v 290

After workspace_homography.yaml exists, generate_grasp.py automatically uses it for X/Y.
