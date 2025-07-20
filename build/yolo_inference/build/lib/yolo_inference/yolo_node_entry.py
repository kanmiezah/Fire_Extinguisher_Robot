import os
import sys
import subprocess

def main():
    venv_python = "/home/nathaniel/ros2_ws/venv/bin/python"  # ✅ use correct path from `which python`
    node_path = "/home/nathaniel/fxb_ws/src/yolo_inference/yolo_inference/yolo_node.py"
    subprocess.run([venv_python, node_path] + sys.argv[1:])

