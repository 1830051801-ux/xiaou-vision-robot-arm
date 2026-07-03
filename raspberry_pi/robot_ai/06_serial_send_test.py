from robot_protocol import encode_heartbeat, encode_motion_frame, frame_to_hex, send_robot_payload


def main() -> None:
    payload = {
        "cmd": "pick",
        "object": "cup",
        "x_base_mm": 200,
        "y_base_mm": 100,
        "pick_z_mm": 225,
        "theta_deg": 45,
        "z_safe_mm": 50,
        "grip_id": 3,
    }
    print(f"heartbeat: {frame_to_hex(encode_heartbeat(seq=1))}")
    print(f"motion:    {frame_to_hex(encode_motion_frame(payload, seq=2))}")
    send_robot_payload(payload, seq=2)


if __name__ == "__main__":
    main()
