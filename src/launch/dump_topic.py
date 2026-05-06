#!/usr/bin/env python3
"""
Dump ros2 topic echo output to a text file.

Usage:
    python3 dump_topic.py                        # dumps /odom
    python3 dump_topic.py /tf                    # dumps /tf
    python3 dump_topic.py /map out.txt           # custom output file
"""

import os
import subprocess
import sys
import signal
from datetime import datetime

topic = sys.argv[1] if len(sys.argv) > 1 else '/odom'
os.makedirs('analysis', exist_ok=True)
timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
topic_slug = topic.lstrip('/').replace('/', '_')
outfile = sys.argv[2] if len(sys.argv) > 2 else f"analysis/{topic_slug}_{timestamp}.txt"

print(f"Dumping {topic} → {outfile}  (Ctrl+C to stop)")

with open(outfile, 'w') as f:
    f.write(f"# topic: {topic}\n# started: {datetime.now().isoformat()}\n\n")
    proc = subprocess.Popen(
        ['ros2', 'topic', 'echo', topic],
        stdout=f,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        proc.wait()
    except KeyboardInterrupt:
        proc.send_signal(signal.SIGINT)
        proc.wait()

print(f"Saved to {outfile}")
