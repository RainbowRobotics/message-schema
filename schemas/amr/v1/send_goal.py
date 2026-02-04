#!/usr/bin/env python3
"""
move/goal queryable 테스트 스크립트
사용법: python send_goal.py [robot_type]
예: python send_goal.py S100-A-3D
"""
import zenoh
import sys
import os

def test_goal():
    robot_type = sys.argv[1] if len(sys.argv) > 1 else "S100-A-3D"
    key = f"{robot_type}/move/goal"

    # goal.bin 파일 경로
    script_dir = os.path.dirname(os.path.abspath(__file__))
    bin_path = os.path.join(script_dir, "goal.bin")

    with open(bin_path, "rb") as f:
        payload = f.read()

    print(f"[TEST] Querying: {key}")
    print(f"[TEST] Payload: {payload.hex()}")

    session = zenoh.open(zenoh.Config())

    replies = session.get(key, payload=payload, timeout=5.0)

    for reply in replies:
        if reply.ok:
            resp = reply.ok.payload.to_bytes()
            print(f"[OK] Response: {resp.hex()}")
        else:
            print(f"[ERR] {reply.err}")

    session.close()
    print("[TEST] Done")

if __name__ == "__main__":
    test_goal()
