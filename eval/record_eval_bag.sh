#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "Usage: bash eval/record_eval_bag.sh <bag_name>"
    exit 1
fi

BAG_NAME="$1"
BAG_PATH="/home/mnt/bags/${BAG_NAME}"

TOPICS=(
    "/odom"
    "/localization_pose"
    "/info"
    "/tf"
)

echo "Checking that all topics are publishing..."
failed=0
for topic in "${TOPICS[@]}"; do
    pub_count=$(ros2 topic info "$topic" 2>/dev/null | grep -oP 'Publisher count: \K[0-9]+' || echo "0")
    if [[ "$pub_count" -eq 0 ]]; then
        echo "ERROR: No publishers on topic: $topic"
        failed=1
    fi
done

if [[ $failed -ne 0 ]]; then
    exit 1
fi

echo "All topics are active."

cleanup() {
    echo ""
    echo "Bag saved to: ${BAG_PATH}"
    if [[ -d "$BAG_PATH" ]]; then
        size=$(du -sh "$BAG_PATH" | cut -f1)
        echo "Bag size: ${size}"
    fi
}
trap cleanup INT TERM

ros2 bag record /odom /localization_pose /info /tf -o "$BAG_PATH" &
REC_PID=$!

echo "Recording started. Press Ctrl+C to stop."
wait $REC_PID
