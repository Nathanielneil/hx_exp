#!/bin/sh
UE4_TRUE_SCRIPT_NAME=$(echo "$0" | xargs readlink -f)
UE4_PROJECT_ROOT=$(dirname "$UE4_TRUE_SCRIPT_NAME")
chmod +x "$UE4_PROJECT_ROOT/Blocks/Binaries/Linux/Blocks"

# 强制窗口模式启动，适合调试
"$UE4_PROJECT_ROOT/Blocks/Binaries/Linux/Blocks" Blocks -windowed -ResX=1280 -ResY=720 -WINDOWED -ForceWindowed "$@"