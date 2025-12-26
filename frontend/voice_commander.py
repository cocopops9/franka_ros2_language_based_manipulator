#!/usr/bin/env python3
"""
Voice Commander - Automatic voice-to-robot command pipeline.

This script is an entry point that uses existing hmi_pkg modules (VAD, ASR, NLU, ROS bridge)
but runs without a UI and sends commands directly to the robot without confirmation.

Robot Motion State Handling:
- After sending a movement command (pick), the robot is considered "busy"
- While busy, ONLY "stop" commands are accepted
- Robot returns to "idle" after "release" command or timeout

Launch from terminal:
    python3 voice_commander.py

Monitor modes (run in separate terminals):
    python3 voice_commander.py --monitor asr          # Monitor ASR text
    python3 voice_commander.py --monitor command      # Monitor parsed commands
    python3 voice_commander.py --monitor objects      # Monitor detected objects
    python3 voice_commander.py --monitor robot        # Monitor robot state
    python3 voice_commander.py --monitor all          # Monitor all topics
    python3 voice_commander.py --monitor topics       # List all ROS2 topics

Main mode (captures voice and sends commands):
    python3 voice_commander.py                        # Run voice commander
    python3 voice_commander.py --no-ros               # Run without ROS2 (testing)
"""

import os
import sys
import json
import argparse
import asyncio
import subprocess
import time
from datetime import datetime
from typing import Optional, List
from enum import Enum

# Add hmi_pkg/nodes to path to import existing modules
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HMI_NODES_PATH = os.path.join(SCRIPT_DIR, "..", "hmi_pkg", "nodes")
sys.path.insert(0, os.path.abspath(HMI_NODES_PATH))

# Set default capabilities path if not set
if not os.environ.get("CAPABILITIES_PATH"):
    os.environ["CAPABILITIES_PATH"] = os.path.abspath(
        os.path.join(SCRIPT_DIR, "..", "hmi_pkg", "config", "capabilities.json")
    )

# Import hmi_pkg modules
from config import Config
from vad import VADProcessor
from nlu_engine import load_capabilities
from audio_utils import ms_of_bytes

# ASR import (async function)
from asr_whisper import transcribe_with_openai

# ROS bridge
from ros_bridge import ROSPublisherThread, ROS_ENABLED

# Audio capture
try:
    import pyaudio
except ImportError:
    print("ERROR: pyaudio not installed. Install with: pip install pyaudio")
    print("On Ubuntu/Debian: sudo apt-get install portaudio19-dev python3-pyaudio")
    sys.exit(1)


# ============================================================================
# ROBOT STATE MANAGEMENT
# ============================================================================

class RobotState(Enum):
    """Robot motion state."""
    IDLE = "idle"
    MOVING = "moving"
    WAITING_RELEASE = "waiting_release"


class RobotStateTracker:
    """
    Tracks robot motion state internally.

    Since the robot controller doesn't publish motion state, we track it based on:
    - Commands we send (pick -> MOVING -> WAITING_RELEASE)
    - Release command (-> IDLE)
    - Timeout (-> IDLE)
    """

    # Commands that start robot motion
    MOTION_COMMANDS = {"pick"}

    # Commands that end robot motion
    RELEASE_COMMANDS = {"release"}

    # Commands allowed while robot is moving/busy
    ALLOWED_WHILE_BUSY = {"stop", "cancel"}

    # Timeout for motion (seconds) - safety fallback
    MOTION_TIMEOUT = 120.0

    def __init__(self):
        self.state = RobotState.IDLE
        self.state_start_time = time.time()
        self.last_command = None

    def on_command_sent(self, action: str, obj: Optional[str] = None):
        """Called when a command is sent to the robot."""
        self.last_command = {"action": action, "object": obj}

        if action in self.MOTION_COMMANDS:
            self.state = RobotState.MOVING
            self.state_start_time = time.time()
            print(f"[STATE] Robot is now MOVING (picking {obj})")

        elif action in self.RELEASE_COMMANDS:
            self.state = RobotState.IDLE
            self.state_start_time = time.time()
            print(f"[STATE] Robot is now IDLE (released)")

        elif action == "stop":
            self.state = RobotState.IDLE
            self.state_start_time = time.time()
            print(f"[STATE] Robot is now IDLE (stopped)")

    def is_busy(self) -> bool:
        """Check if robot is busy (in motion or waiting for release)."""
        # Check timeout
        if self.state != RobotState.IDLE:
            elapsed = time.time() - self.state_start_time
            if elapsed > self.MOTION_TIMEOUT:
                print(f"[STATE] Motion timeout after {elapsed:.1f}s, assuming IDLE")
                self.state = RobotState.IDLE
                return False

        return self.state != RobotState.IDLE

    def can_accept_command(self, action: str) -> tuple[bool, str]:
        """
        Check if a command can be accepted given current robot state.

        Returns: (can_accept, reason)
        """
        if not self.is_busy():
            return True, ""

        # Robot is busy - only allow stop/cancel
        if action in self.ALLOWED_WHILE_BUSY:
            return True, ""

        return False, f"Robot is {self.state.value}. Only 'stop' or 'cancel' commands accepted."

    def get_state_display(self) -> str:
        """Get state string for display."""
        if self.is_busy():
            elapsed = time.time() - self.state_start_time
            return f"{self.state.value} ({elapsed:.1f}s)"
        return self.state.value


# ============================================================================
# VOICE COMMANDER
# ============================================================================

class VoiceCommander:
    """
    Automatic voice-to-robot command pipeline.
    Uses hmi_pkg modules: VAD, ASR (Whisper), NLU, ROS bridge.
    """

    def __init__(self, use_ros: bool = True):
        self.use_ros = use_ros and ROS_ENABLED

        # Initialize components from hmi_pkg
        self.vad = VADProcessor()
        self.nlu = load_capabilities()
        self.ros_thread: Optional[ROSPublisherThread] = None

        # Robot state tracking
        self.robot_state = RobotStateTracker()

        # Audio settings (from Config)
        self.sample_rate = Config.SR_DEFAULT
        self.chunk_size = 1024
        self.format = pyaudio.paInt16
        self.channels = 1

        # PyAudio
        self.pyaudio = pyaudio.PyAudio()
        self.stream = None

        # State
        self.running = False
        self.last_error = [""]  # Mutable ref for ASR error tracking

    def _find_microphone(self) -> int:
        """Find the best microphone device."""
        try:
            default_input = self.pyaudio.get_default_input_device_info()
            print(f"[MIC] Default input: {default_input.get('name')}")
            return int(default_input.get('index'))
        except Exception as e:
            print(f"[MIC] Could not get default device: {e}")
            # Try to find any input device
            info = self.pyaudio.get_host_api_info_by_index(0)
            for i in range(info.get('deviceCount', 0)):
                dev = self.pyaudio.get_device_info_by_host_api_device_index(0, i)
                if dev.get('maxInputChannels') > 0:
                    print(f"[MIC] Using: {dev.get('name')}")
                    return i
            raise RuntimeError("No input device found")

    def _init_ros(self):
        """Initialize ROS bridge thread."""
        if not self.use_ros:
            print("[ROS] ROS disabled")
            return

        print(f"[ROS] Initializing bridge (ASR->{Config.ROS_ASR_TOPIC}, intent->{Config.ROS_INTENT_TOPIC})")
        self.ros_thread = ROSPublisherThread()
        self.ros_thread.start()

        # Wait for ROS thread to be ready
        for _ in range(50):  # 5 second timeout
            if self.ros_thread.ready:
                print("[ROS] Bridge ready")
                return
            if self.ros_thread.init_error:
                print(f"[ROS] Init error: {self.ros_thread.init_error}")
                self.ros_thread = None
                return
            time.sleep(0.1)

        print("[ROS] Bridge timeout, continuing without ROS")
        self.ros_thread = None

    def start(self):
        """Start the voice commander."""
        print("\n" + "=" * 60)
        print("  VOICE COMMANDER - Automatic Voice Control")
        print("=" * 60)
        print(f"  Using hmi_pkg modules from: {HMI_NODES_PATH}")
        print(f"  ROS2 Mode: {'Enabled' if self.use_ros else 'Disabled'}")
        print(f"  Capabilities: {Config.CAPABILITIES_PATH}")
        print("=" * 60)
        print("\nInitializing...\n")

        # Check OpenAI key
        if not Config.OPENAI_KEY:
            print("[ERROR] OPENAI_API_KEY environment variable not set!")
            print("Set it with: export OPENAI_API_KEY='your-key-here'")
            return

        # Initialize ROS
        self._init_ros()

        # Find microphone
        try:
            device_index = self._find_microphone()
        except Exception as e:
            print(f"[ERROR] Could not find microphone: {e}")
            return

        # Open audio stream
        try:
            self.stream = self.pyaudio.open(
                format=self.format,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                input_device_index=device_index,
                frames_per_buffer=self.chunk_size
            )
            print("[MIC] Audio stream opened")
        except Exception as e:
            print(f"[ERROR] Could not open audio stream: {e}")
            return

        self.running = True

        print("\n" + "-" * 60)
        print("  READY! Speak commands like:")
        print("    - 'pick cup'      (starts robot motion)")
        print("    - 'pick bottle'")
        print("    - 'stop'          (allowed during motion)")
        print("    - 'release'       (ends motion sequence)")
        print("    - 'next'")
        print("-" * 60)
        print("\n  NOTE: While robot is moving, only 'stop' is accepted!")
        print("\nCalibrating noise floor... (stay quiet for a moment)")
        print("\nPress Ctrl+C to exit\n")

        # Run async main loop
        try:
            asyncio.run(self._main_loop())
        except KeyboardInterrupt:
            print("\n\n[INFO] Shutting down...")
        finally:
            self.stop()

    async def _main_loop(self):
        """Main processing loop."""
        while self.running:
            try:
                # Read audio chunk
                audio_data = self.stream.read(self.chunk_size, exception_on_overflow=False)

                # Process calibration first
                if self.vad.state.calibrating:
                    done = self.vad.process_calibration(audio_data, self.sample_rate)
                    if done:
                        print(f"[VAD] Calibration complete: gate_rms={self.vad.state.gate_rms:.4f}")
                    continue

                # Process through VAD
                segment = self.vad.process_audio(audio_data, self.sample_rate)

                if segment:
                    await self._process_utterance(segment)

                # Small yield to prevent blocking
                await asyncio.sleep(0.001)

            except Exception as e:
                print(f"[ERROR] Processing error: {e}")
                await asyncio.sleep(0.1)

    async def _process_utterance(self, audio_data: bytes):
        """Process a complete utterance."""
        seg_ms = ms_of_bytes(len(audio_data), self.sample_rate)
        print(f"\n[PROCESSING] Transcribing speech ({seg_ms}ms)...")

        # ASR using hmi_pkg's transcribe_with_openai
        text = await transcribe_with_openai(audio_data, self.sample_rate, self.last_error)

        timestamp = datetime.now().strftime("%H:%M:%S")

        if not text:
            if self.last_error[0]:
                print(f"[ASR] Error: {self.last_error[0]}")
                self.last_error[0] = ""
            else:
                print("[ASR] No transcription result")
            return

        print(f"\n{'=' * 50}")
        print(f"[{timestamp}] ASR TEXT: \"{text}\"")
        print(f"[{timestamp}] ROBOT STATE: {self.robot_state.get_state_display()}")

        # Publish ASR text to ROS
        if self.ros_thread and self.ros_thread.ready:
            self.ros_thread.publish_asr(text)

        # NLU parsing using hmi_pkg's NLU
        intent = self.nlu.parse(text)

        if intent:
            action = intent.get('action')
            obj = intent.get('object')
            print(f"[{timestamp}] PARSED COMMAND:")
            print(f"    Action: {action or '(none)'}")
            print(f"    Object: {obj or '(none)'}")

            if action:
                # Check if command is allowed given robot state
                can_accept, reason = self.robot_state.can_accept_command(action)

                if can_accept:
                    intent_json = json.dumps({"action": action, "object": obj})
                    if self.ros_thread and self.ros_thread.ready:
                        self.ros_thread.publish_intent(intent_json)
                        print(f"[SENT] Command sent to robot: {intent_json}")
                    else:
                        print(f"[DEMO] Would send to robot: {intent_json}")

                    # Update robot state
                    self.robot_state.on_command_sent(action, obj)
                else:
                    print(f"[BLOCKED] {reason}")
                    print(f"[BLOCKED] Say 'stop' to halt the robot first.")
            else:
                print(f"[SKIP] No valid action detected")
        else:
            print(f"[{timestamp}] No valid command detected")

        print(f"{'=' * 50}\n")

    def stop(self):
        """Stop the voice commander."""
        self.running = False

        if self.stream:
            self.stream.stop_stream()
            self.stream.close()

        self.pyaudio.terminate()
        print("[INFO] Voice commander stopped")


# ============================================================================
# MONITOR MODE
# ============================================================================

class TopicMonitor:
    """ROS2 topic monitor for terminal display."""

    def __init__(self, topics: List[str]):
        self.topics = topics

    def start(self):
        """Start monitoring topics."""
        # Check if ROS2 is available
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import String
        except ImportError:
            print("[ERROR] ROS2 (rclpy) not available. Cannot monitor topics.")
            return

        print("\n" + "=" * 60)
        print("  TOPIC MONITOR")
        print("=" * 60)
        print(f"  Monitoring: {', '.join(self.topics)}")
        print("=" * 60)
        print("\nPress Ctrl+C to exit\n")

        try:
            rclpy.init()
            node = rclpy.create_node('voice_commander_monitor')

            # Create subscriptions
            for topic in self.topics:
                self._create_subscription(node, topic, String)

            while True:
                rclpy.spin_once(node, timeout_sec=0.1)

        except KeyboardInterrupt:
            print("\n[INFO] Monitor stopped")
        finally:
            try:
                node.destroy_node()
                rclpy.shutdown()
            except:
                pass

    def _create_subscription(self, node, topic: str, msg_type):
        """Create subscription for a topic."""
        def callback(msg):
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            data = msg.data if hasattr(msg, 'data') else str(msg)

            # Format output based on topic
            if 'asr' in topic.lower():
                print(f"[{timestamp}] [ASR] {data}")
            elif 'intent' in topic.lower():
                try:
                    intent = json.loads(data)
                    action = intent.get('action', '?')
                    obj = intent.get('object', '?')
                    print(f"[{timestamp}] [CMD] action={action}, object={obj}")
                except:
                    print(f"[{timestamp}] [CMD] {data}")
            elif 'detected_objects' in topic.lower():
                try:
                    objects = json.loads(data)
                    if objects:
                        labels = [f"{o.get('label', '?')} ({o.get('x', 0):.2f},{o.get('y', 0):.2f},{o.get('z', 0):.2f})"
                                  for o in objects[:5]]  # Limit to 5
                        print(f"[{timestamp}] [OBJ] {', '.join(labels)}")
                except:
                    if len(data) > 80:
                        data = data[:80] + "..."
                    print(f"[{timestamp}] [OBJ] {data}")
            elif 'joint' in topic.lower():
                print(f"[{timestamp}] [JOINTS] (data received)")
            else:
                if len(data) > 80:
                    data = data[:80] + "..."
                print(f"[{timestamp}] [{topic}] {data}")

        node.create_subscription(msg_type, topic, callback, 10)
        print(f"[SUB] Subscribed to: {topic}")


def run_monitor(mode: str):
    """Run in monitor mode."""
    topic_map = {
        'asr': [Config.ROS_ASR_TOPIC],
        'command': [Config.ROS_INTENT_TOPIC],
        'objects': ['/vision/detected_objects_json', '/vision_module/detected_objects'],
        'robot': ['/joint_states'],
        'all': [
            Config.ROS_ASR_TOPIC,
            Config.ROS_INTENT_TOPIC,
            '/vision/detected_objects_json',
        ],
        'topics': None  # Special case
    }

    if mode == 'topics':
        # List all ROS2 topics
        print("\n[INFO] Listing all ROS2 topics...\n")
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True)
            if result.returncode == 0:
                print(result.stdout)
            else:
                print(f"[ERROR] {result.stderr}")
        except FileNotFoundError:
            print("[ERROR] 'ros2' command not found. Is ROS2 sourced?")
        except Exception as e:
            print(f"[ERROR] Could not list topics: {e}")
        return

    topics = topic_map.get(mode)
    if not topics:
        print(f"[ERROR] Unknown monitor mode: {mode}")
        print("Available modes: asr, command, objects, robot, all, topics")
        return

    monitor = TopicMonitor(topics)
    monitor.start()


# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Voice Commander - Automatic voice-to-robot command pipeline',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Monitor Modes (run in separate terminals):
  --monitor asr        Monitor ASR transcriptions
  --monitor command    Monitor parsed commands
  --monitor objects    Monitor detected objects
  --monitor robot      Monitor robot joint states
  --monitor all        Monitor all main topics
  --monitor topics     List all ROS2 topics

Robot Motion State:
  While robot is moving, only 'stop' commands are accepted.
  After 'pick' -> robot is MOVING
  After 'stop' or 'release' -> robot is IDLE

Examples:
  python3 voice_commander.py                # Run voice commander
  python3 voice_commander.py --no-ros       # Run without ROS2
  python3 voice_commander.py --monitor asr  # Monitor ASR topic
        """
    )

    parser.add_argument(
        '--monitor', '-m',
        type=str,
        choices=['asr', 'command', 'objects', 'robot', 'all', 'topics'],
        help='Run in monitor mode to view specific topics'
    )

    parser.add_argument(
        '--no-ros',
        action='store_true',
        help='Run without ROS2 (for testing)'
    )

    parser.add_argument(
        '--list-devices',
        action='store_true',
        help='List available audio input devices and exit'
    )

    args = parser.parse_args()

    # List devices mode
    if args.list_devices:
        pa = pyaudio.PyAudio()
        info = pa.get_host_api_info_by_index(0)
        print("\nAvailable Audio Input Devices:")
        print("-" * 40)
        for i in range(info.get('deviceCount', 0)):
            dev = pa.get_device_info_by_host_api_device_index(0, i)
            if dev.get('maxInputChannels') > 0:
                print(f"  [{i}] {dev.get('name')}")
        pa.terminate()
        return

    # Monitor mode
    if args.monitor:
        run_monitor(args.monitor)
        return

    # Main voice commander mode
    commander = VoiceCommander(use_ros=not args.no_ros)
    commander.start()


if __name__ == "__main__":
    main()
