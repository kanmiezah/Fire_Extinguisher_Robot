import os
import sys
import subprocess

def main():
    python_exec = sys.executable  # Uses current interpreter
    node_path = os.path.join(os.path.dirname(__file__), 'yolo_node.py')
    subprocess.run([python_exec, node_path] + sys.argv[1:])
