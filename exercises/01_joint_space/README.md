# 1. Joint space

Sliders straight to the servos. No time, no coordination.

## Wire

- `ui/joints` to the driver
- `driver/state` back to the ui

## Try

- move `shoulder_pan` by 1.5 rad and `wrist_flex` by 0.3 rad in one go
- watch the measured velocity plot

## Look for

- each joint runs at the servo's top speed, 3 rad/s
- the short move finishes first
- velocity jumps from 0 to full in 0.15 s
