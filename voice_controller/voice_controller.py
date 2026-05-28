#!/usr/bin/env python3
import json
import os
import sys
import threading
import time

import anthropic
import numpy as np
import paramiko
import sounddevice as sd
from faster_whisper import WhisperModel
from pynput import keyboard

# ──────────────────────────────────────────────
# CONFIG
# ──────────────────────────────────────────────
with open("api_key.txt", "r") as f:
    ANTHROPIC_API_KEY = f.read().strip()

ROBOT_HOST = "stretch-re2-2204.local"
ROBOT_USER = "willy"
ROBOT_DIR  = "/home/willy/stretch_script"

SSH_KEY = os.path.expanduser("~/.ssh/id_ed25519")
SSH_PASS = None

WHISPER_MODEL   = "small"
WHISPER_DEVICE  = "cpu"
WHISPER_COMPUTE = "int8"

VALID_LOCATIONS = {"stationA", "stationB", "stationC", "stationD", "center", "origin"}

# ✅ TEST MODE FLAG
TEST_MODE = "--test" in sys.argv

# ──────────────────────────────────────────────
# SYSTEM PROMPT
# ──────────────────────────────────────────────
SYSTEM_PROMPT = """
You control a Stretch3 robot.

Available commands:
- dock_pos.sh <location>
  Valid: stationA, stationB, stationC, stationD, center, origin
  "home", "base", "start" → origin

- pick_plate.sh
- place_plate.sh
- pick_object.sh <object>
- place_object.sh

Rules:
- Only output valid locations (never invent new ones)
- NEVER use "home" (map to origin)
- NEVER repeat the same command more than once in a single response
- pick_plate.sh and place_plate.sh can only appear once per command list
- Output ONLY JSON:
{
  "understood": true/false,
  "commands": ["./cmd arg"],
  "explanation": "short sentence"
}
"""

# ──────────────────────────────────────────────
# SSH
# ──────────────────────────────────────────────
def make_ssh_client():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    kw = {"hostname": ROBOT_HOST, "username": ROBOT_USER}

    if SSH_KEY and os.path.exists(SSH_KEY):
        kw["key_filename"] = SSH_KEY
    elif SSH_PASS:
        kw["password"] = SSH_PASS

    ssh.connect(**kw)
    return ssh


def run_commands_on_robot(commands):
    if TEST_MODE:
        for cmd in commands:
            print(f"\n[TEST MODE] Would run: {cmd}")
        return

    ssh = make_ssh_client()

    for cmd in commands:
        remote_script = f"{ROBOT_DIR}/{cmd.lstrip('./')}"
        full_cmd = f"bash -i -c '{remote_script}'"

        print(f"\nRobot running: {full_cmd}")

        stdin, stdout, stderr = ssh.exec_command(full_cmd)

        exit_code = stdout.channel.recv_exit_status()
        out = stdout.read().decode().strip()
        err = stderr.read().decode().strip()

        # Filter harmless bash -i TTY warnings
        real_errors = "\n".join(
            l for l in err.splitlines()
            if "cannot set terminal process group" not in l
            and "no job control" not in l
        ).strip()

        if exit_code != 0 and real_errors:
            print(f"FAILED ({exit_code}): {real_errors}")
            ssh.close()
            return

        if out:
            print(out)

    ssh.close()


# ──────────────────────────────────────────────
# CLAUDE
# ──────────────────────────────────────────────
def ask_claude(client, text):
    msg = client.messages.create(
        model="claude-haiku-4-5-20251001",
        max_tokens=512,
        system=SYSTEM_PROMPT,
        messages=[{"role": "user", "content": text}],
    )

    raw = msg.content[0].text.strip()

    if raw.startswith("```"):
        raw = raw.split("```")[1]
        if raw.startswith("json"):
            raw = raw[4:]

    return json.loads(raw.strip())


# ──────────────────────────────────────────────
# PUSH-TO-TALK STATE
# ──────────────────────────────────────────────
RATE = 16000

recording = False
robot_busy = False
audio_frames = []
audio_lock = threading.Lock()
process_event = threading.Event()
anthropic_client = None
whisper_model = None


def audio_callback(indata, frames, time_info, status):
    """Collects audio frames while recording is active."""
    if recording:
        with audio_lock:
            audio_frames.append(indata.copy())


def process_audio(client, model):
    """Transcribes and dispatches the captured audio."""
    with audio_lock:
        frames = list(audio_frames)

    if not frames:
        print("No audio captured.")
        return

    audio_np = np.concatenate(frames).flatten().astype(np.float32)

    # Noise gate — skip if too quiet
    level = np.abs(audio_np).mean()
    print(f"Audio level: {level:.5f}")
    if level < 0.001:
        print("Too quiet, ignoring.")
        return

    print("Transcribing...")
    segments, _ = model.transcribe(audio_np, language="en")
    text = "".join(s.text for s in segments).strip()

    if not text:
        print("Nothing understood.")
        return

    print(f"\n> {text}")

    try:
        plan = ask_claude(client, text)
        print(f"Plan: {plan['explanation']}")

        cmds = plan.get("commands", [])

        if not plan.get("understood") or not cmds:
            print("No valid command.")
            return

        validated_cmds = []
        for c in cmds:
            if "dock_pos.sh" in c:
                loc = c.split()[-1]

                if loc == "home":
                    loc = "origin"
                    c = f"./dock_pos.sh {loc}"

                if loc not in VALID_LOCATIONS:
                    print(f"Blocked invalid location: {loc}")
                    continue

            validated_cmds.append(c)

        if not validated_cmds:
            print("No safe commands to run.")
            return

        print(f"Commands: {validated_cmds}")
        global robot_busy
        robot_busy = True
        try:
            run_commands_on_robot(validated_cmds)
        finally:
            robot_busy = False

    except Exception as e:
        print(f"Error: {e}")


# ──────────────────────────────────────────────
# KEYBOARD LISTENER
# ──────────────────────────────────────────────
def on_press(key):
    global recording

    if key == keyboard.Key.space and not recording and not robot_busy:
        recording = True
        with audio_lock:
            audio_frames.clear()
        print("\n🔴 Recording... (release Space to stop)")

    elif key == keyboard.Key.space and robot_busy:
        print("⚠️  Robot is busy, please wait...")


def on_release(key):
    global recording

    if key == keyboard.Key.space and recording:
        recording = False
        print("⏹  Stopped. Processing...")
        # Signal the processing thread
        process_event.set()

    if key == keyboard.Key.esc:
        print("\nExiting.")
        return False  # Stop listener


def processing_loop():
    """Waits for recording to finish, then processes audio."""
    while True:
        process_event.wait()
        process_event.clear()
        process_audio(anthropic_client, whisper_model)
        print("\nReady — hold Space to record, Esc to quit.")


# ──────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────
def main():
    global anthropic_client, whisper_model

    print("Loading Whisper...")
    whisper_model = WhisperModel(
        WHISPER_MODEL,
        device=WHISPER_DEVICE,
        compute_type=WHISPER_COMPUTE
    )
    print("Whisper ready.\n")

    if not TEST_MODE:
        print("Testing SSH...")
        ssh = make_ssh_client()
        ssh.close()
        print("SSH OK.\n")
    else:
        print("TEST MODE: SSH connection skipped.\n")

    anthropic_client = anthropic.Anthropic(api_key=ANTHROPIC_API_KEY)

    # Start processing thread
    threading.Thread(target=processing_loop, daemon=True).start()

    # Start audio stream (always open, only records when flag is set)
    stream = sd.InputStream(
        channels=1,
        samplerate=RATE,
        callback=audio_callback
    )
    stream.start()

    print("Ready — hold Space to record, Esc to quit.\n")

    # Block on keyboard listener
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

    stream.stop()
    stream.close()


if __name__ == "__main__":
    main()
