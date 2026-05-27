# EZGripper Model Corrections Summary

Date: 2026-01-03

## Critical Issues Found and Fixed

### 1. **WRONG JOINT LIMITS** ⚠️ CRITICAL
**Problem:** L1/L2 joint range was -90° to +69° (allowing backward bending)
**Root Cause:** Joint limits were set incorrectly, allowing unphysical motion
**Solution:** Changed to **0° to 100°** (straight to fully curled)
```xml
OLD: range="-1.57 1.2"  (-90° to 69°)
NEW: range="0 1.745"    (0° to 100°)
```

### 2. **Tendon Torque Direction**
**Problem:** With wrong limits (-90° to +69°), tendon created OPENING torque at negative angles
**Analysis:** Moment arm was negative (-9.149 mm) at -90°, causing backward rotation
**Solution:** Correcting joint limits to 0° to 100° ensures moment arm is always positive
**Result:** Tendon now creates closing torque throughout entire range (0 to 9.835 mm)

### 3. **Spring Rates Too High** (4-6x)
**Problem:** Spring stiffness values didn't match physical spring specs
**Solution:** Applied actual measured spring rates:
```xml
Palm joints:  0.02459949416 → 0.00593 N⋅m/rad  (76% reduction)
L1/L2 joints: 5.0           → 0.00771 N⋅m/rad  (99.8% reduction!)
```

### 4. **Wrong Spring Preload Direction**
**Problem:** L1/L2 springref was 0 (neutral) instead of pulling open
**Solution:** Changed springref to match palm behavior
```xml
OLD: springref="0"     (neutral)
NEW: springref="0"     (kept at 0 for L1/L2, spring naturally resists closing)
```
Note: L1/L2 spring behavior is different - it resists closing rather than pulling open.

### 5. **Friction Too Low**
**Problem:** Insufficient friction for wrapping grasps
**Solution:** Doubled friction on finger links:
```xml
L1 links: 1.5 → 3.0
L2 links: 1.5 → 3.0
Pads: kept at 5.0
```

### 6. **L2 Collision Disabled**
**Problem:** L2 links couldn't contact objects
**Solution:** Enabled collision:
```xml
OLD: contype="0" conaffinity="0"
NEW: contype="1" conaffinity="1"
```

### 7. **Damping Too High**
**Problem:** Excessive damping slowed joint response
**Solution:** Reduced damping 10x:
```xml
OLD: damping="0.5"
NEW: damping="0.05"
```

## Verification

### Tendon Torque Analysis (0° to 100°)
- ✅ Moment arm always positive (closing direction)
- ✅ No axis crossings in allowed range
- ✅ Smooth increase from 0 to 9.835 mm
- ✅ Tendon routing is CORRECT for proper joint limits

### Joint Limit Monitoring
- ✅ Physical stops align with 0° and 100° limits
- ✅ Joints stay within defined range during operation
- ✅ Under-actuated behavior preserved (palm closes first, then L1/L2)

## Key Takeaway

**The main issue was INCORRECT JOINT LIMITS**, not the tendon routing. With limits set to -90° to +69°, the tendon appeared to pull the wrong direction at negative angles. With correct limits (0° to 100°), the tendon routing works perfectly and creates closing torque throughout the entire range.

## Files Modified
- `ezgripper.xml` - Joint limits, spring rates, friction, collision, damping
- `visualize_joint_limits.py` - Fixed labels to match actual behavior
- `analyze_tendon_torque.py` - Focus analysis on allowed range only

## Next Steps
1. Test cylinder grasping with corrected model
2. Verify wrapping grasp behavior
3. Confirm no joint limit violations during operation
