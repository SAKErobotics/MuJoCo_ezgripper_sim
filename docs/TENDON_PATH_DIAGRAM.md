# EZGripper Tendon Path Diagram

## Current Configuration (REVERSED PATH)

### Finger 1 Tendon Path:
```
ACTUATOR → palm_peg0 → palm_peg1 → palm_pulley_f1 → f1l1_peg0 → f1l1_pulley → f1l1_peg1 → f1l2_peg → f1l2_pin (FINGER TIP)
           (0.022,0,0)  (0.063,0,0)    (wraps)      (0.0115)      (wraps)      (0.0355)    (0.011)    (0.0185)
```

**Path Direction: REVERSED (tip to palm)**
```
START: f1l2_pin (finger tip)
   ↓
   f1l2_peg
   ↓
   f1l1_peg1
   ↓
   f1l1_pulley (wraps around cylinder)
   ↓
   f1l1_peg0
   ↓
   palm_pulley_f1 (wraps around cylinder)
   ↓
   palm_peg1
   ↓
END: palm_peg0 → ACTUATOR
```

## Visual Representation

### Side View of Finger 1:

```
                                    FINGER TIP
                                    ┌─────────┐
                                    │  f1l2   │
                                    │  (L2)   │
                                    └────┬────┘
                                         │ f1l2_pin (0.0185)
                                         │ f1l2_peg (0.011)
                                         │
                    F1_knuckle_tip joint │
                                    ┌────┴────┐
                                    │  f1l1   │
                                    │  (L1)   │
                                    │         │
                    f1l1_peg1 ──────┤ ◉       │ (0.0355)
                    (tendon)        │ │       │
                                    │ │pulley │
                    f1l1_peg0 ──────┤ ◉       │ (0.0115)
                                    └────┬────┘
                                         │
                F1_palm_knuckle joint    │
                                    ┌────┴────┐
                                    │  PALM   │
                                    │         │
                palm_peg2 ──────────┤ ◉       │ (0.085)
                (pulley wrap)       │ │       │
                palm_peg1 ──────────┤ ●       │ (0.063)
                                    │         │
                palm_peg0 ──────────┤ ●───────┼──→ ACTUATOR
                                    │         │    (0.022)
                                    └─────────┘
```

## Tendon Mechanics

### When Actuator PULLS (shortens tendon):
1. Tendon length DECREASES
2. Pulls from palm_peg0 end
3. Since path is REVERSED (tip→palm), shortening pulls the TIP toward PALM
4. This CLOSES the finger (rotates toward +25°)

### When Actuator RELEASES (lengthens tendon):
1. Tendon length INCREASES
2. Springs push finger open
3. Finger rotates toward -90° (OPEN position)
4. Hits hard stop at -90°

## Key Points:

- **Reversed path**: Tendon defined from TIP to PALM (not palm to tip)
- **Pulleys**: Wrap around cylinders at palm and L1 knuckle
- **Actuator control**: 
  - Shorter tendon length = CLOSED (+25°)
  - Longer tendon length = OPEN (-90°)
- **Springs**: Pull toward -180° (very closed), but limited by -90° hard stop
- **Equality constraint**: Both finger tendons must have equal length (symmetric closing)

## Position Control Range:
- Control value = desired tendon length in meters
- Range: 0.145m (closed) to 0.170m (open)
- Natural length at 0°: ~0.151m

## Current Status:
✅ Tendon path REVERSED to fix direction
✅ Position actuator with kp=100
✅ Equality constraint for symmetric closing
⏳ Testing to verify fingers now close toward +25°
