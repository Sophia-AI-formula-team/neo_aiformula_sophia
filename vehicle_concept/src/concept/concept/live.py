"""The short command and ros2 launch use identical launch arguments."""
import os
import sys


def main():
    os.execvp('ros2', ['ros2', 'launch', 'concept', 'concept.launch.py', *sys.argv[1:]])
