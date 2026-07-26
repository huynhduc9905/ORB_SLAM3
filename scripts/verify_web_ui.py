#!/usr/bin/env python3
import sys
import time
import subprocess
import requests
from pathlib import Path
from playwright.sync_api import sync_playwright

def main():
    print("=== ORB-SLAM3 Web Visualizer Verification ===")

    # 1. Check if backend process is running or start mono_web_runner
    print("[1/4] Checking backend healthz endpoint at http://127.0.0.1:8085/healthz...")
    try:
        resp = requests.get("http://127.0.0.1:8085/healthz", timeout=3)
        print("Healthz response:", resp.json())
        assert resp.status_code == 200, "Healthz status code is not 200"
    except Exception as e:
        print(f"Healthz endpoint error: {e}")
        return False

    print("[2/4] Verifying public configuration endpoint at http://127.0.0.1:8085/config...")
    resp_cfg = requests.get("http://127.0.0.1:8085/config")
    print("Config response:", resp_cfg.json())

    print("[3/4] Launching Playwright Headless Browser to verify Web UI...")
    artifacts_dir = Path("docs/superpowers/artifacts")
    artifacts_dir.mkdir(parents=True, exist_ok=True)

    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        context = browser.new_context(viewport={'width': 1280, 'height': 800})
        page = context.new_page()

        page.goto("http://127.0.0.1:8085")
        page.wait_for_selector("#webgl-canvas", timeout=10000)
        print("WebGL canvas loaded successfully.")

        # Check HUD elements
        status_text = page.inner_text("#stat-status")
        print(f"HUD Status text: {status_text}")

        time.sleep(3) # Wait for WebSocket messages and rendering

        kp_text = page.inner_text("#stat-kp")
        points_text = page.inner_text("#stat-points")
        print(f"HUD Tracked Keypoints: {kp_text}")
        print(f"HUD Active Map Points: {points_text}")

        screenshot_path = artifacts_dir / "ui_verification.png"
        page.screenshot(path=str(screenshot_path))
        print(f"[4/4] Saved desktop verification screenshot artifact to {screenshot_path}")

        # Mobile Viewport Verification
        mobile_context = browser.new_context(viewport={'width': 390, 'height': 844}, is_mobile=True, has_touch=True)
        mobile_page = mobile_context.new_page()
        mobile_page.goto("http://127.0.0.1:8085")
        mobile_page.wait_for_selector("#webgl-canvas", timeout=10000)
        time.sleep(2)

        mobile_screenshot_path = artifacts_dir / "mobile_ui_verification.png"
        mobile_page.screenshot(path=str(mobile_screenshot_path))
        print(f"Saved mobile verification screenshot artifact to {mobile_screenshot_path}")

        browser.close()

    print("=== Verification Successful! ===")
    return True

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
