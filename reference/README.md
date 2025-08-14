# Reference Materials

This directory contains reference materials for the HX UAV Control System project.

## Contents

### tutorial_demo/
Original demonstration code from the Prometheus project. These files serve as reference for understanding the original implementation but are not part of the HX Python enhanced system.

**Purpose**: Reference and educational material
**Status**: Archived - not actively maintained
**Usage**: For studying original Prometheus examples

### uav_control/
Original Prometheus control algorithms in C++ format. The core control algorithms from these files have been faithfully ported to Python in the HX system.

**Purpose**: Algorithm reference - source of truth for control algorithms
**Status**: Reference only - algorithms preserved in HX Python version
**Usage**: For algorithm verification and parameter reference

## Important Note

**The control algorithms in `uav_control/` have been 100% faithfully preserved** in the HX Python enhanced version located at `/src/hx_exp_py/python_enhanced/controllers/`. 

- **PID Controller**: `pos_controller_PID.h` → `pid_controller.py`
- **UDE Controller**: `pos_controller_UDE.h` → `ude_controller.py`  
- **ADRC Controller**: `pos_controller_ADRC.h` → `adrc_controller.py`

## Main Project Location

The active HX UAV Control System is located at:
```
/src/hx_exp_py/
```

This reference directory exists only for historical context and algorithm verification purposes.