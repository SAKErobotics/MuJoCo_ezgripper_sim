# EZGripper XML Configuration History

Complete history of geometry and spring setting changes across all commits.

---

## Summary of Key Changes

### Spring Stiffness Values

| Commit | Date | Palm Stiffness | L1/L2 Tip Stiffness | Notes |
|--------|------|----------------|---------------------|-------|
| acff90d | 2021-04-01 | 0.0511 | 0.0246 | **Original** - Springs SWAPPED! |
| 6defecf | 2021-04-13 | 0.0511 | 0.0246 | Same as original |
| fa01a8d | 2025-11-21 | **0.0246** | **0.0511** | **CORRECTED** - Springs fixed |
| 62a0ae7 | 2025-11-30 | 0.0246 | 0.0511 | Z offset removed from peg |
| 4ab2604 | 2026-01-02 | 0.0246 | 0.0511 | Working version |
| e7bb7e4 | 2026-01-03 | 0.0246 | 0.0511 | Current HEAD |
| **TODAY** | 2026-01-03 | **0.00593** | **0.00771** | **From actual spring specs** |

### L1/L2 Tip Joint Range Evolution

| Commit | Date | Range | springref | Notes |
|--------|------|-------|-----------|-------|
| acff90d-fa01a8d | 2021-2025 | 0 to 1.7 | -1.57 | Original range |
| 8957985-4e02511 | 2025-11-21 | -0.5 to 1.7 | -1.57 | Negative opening allowed |
| c57e9ad | 2025-11-21 | **0 to 1.7** | -1.57 | Hard stop at 0° |
| d0f5fd8 | 2025-11-21 | -0.2618 to 1.7 | -1.57 | 15° opening |
| ff7f27f | 2025-11-21 | 0.2618 to 1.7 | -1.57 | 15° closing minimum |
| 6fb70ec | 2025-11-21 | 0.5236 to 1.7 | -1.57 | 30° minimum |
| ef8aa54-2c1f7ef | 2025-11-25 | 0 to 1.7 | -1.57 | Back to 0° |
| 62a0ae7 | 2025-11-30 | **-0.01 to 1.2** | **0** | springref changed! |
| 4ab2604 | 2026-01-02 | -0.01 to 1.2 | 0 | Working version |
| e7bb7e4 | 2026-01-03 | **-1.57 to 1.2** | 0 | Extended negative range |
| **TODAY** | 2026-01-03 | -1.57 to 1.2 | **-1.57** | Reverted springref |

### Tendon Site Positions (f1l2_peg)

| Commit | Date | Position | Notes |
|--------|------|----------|-------|
| acff90d-eb39894 | 2021-2025 | `.011 0 -.004` | **Z offset = -4mm** |
| 62a0ae7-e7bb7e4 | 2025-2026 | `.011 0 0` | **Z offset removed** |
| **TODAY** | 2026-01-03 | `.011 0 0` | Keeping Z=0 (tendon in plane) |

---

## Detailed Commit History

### Commit: `e7bb7e4` (HEAD)
**Date:** 2026-01-03 09:41:41 -0800  
**Message:** Fix EZGripper tendon force calibration and symmetric closing  
**Author:** SAKE Robotics

**Joint Parameters:**
```xml
F1_palm_knuckle: stiffness="0.02459949416" springref="-3.14"
F1_knuckle_tip: stiffness="0.05114105365" springref="0"
```

**Tendon Sites:**
```xml
f1l2_peg: pos=".011 0 0"
f1l2_pin: pos=".0185 0 0"
```

**Changes from previous:**
- Extended L1/L2 range to -1.57 (from -0.01)
- Changed springref from 0 to... wait, this shows springref="0"

---

### Commit: `4ab2604`
**Date:** 2026-01-02 21:33:41 -0800  
**Message:** FIX: Implement symmetric closing with tendon equality constraint  
**Author:** SAKE Robotics

**Joint Parameters:**
```xml
F1_palm_knuckle: stiffness="0.02459949416" springref="-3.14"
F1_knuckle_tip: stiffness="0.05114105365" springref="0"
```

**Tendon Sites:**
```xml
f1l2_peg: pos=".011 0 0"
f1l2_pin: pos=".0185 0 0"
```

**Key Change:** springref changed from -1.57 to **0** for L1/L2 tip joint!

---

### Commit: `62a0ae7`
**Date:** 2025-11-30 12:38:22 -0800  
**Message:** Fix gripper finger synchronization and enable finger contact  
**Author:** SAKE Robotics

**Joint Parameters:**
```xml
F1_palm_knuckle: stiffness="0.02459949416" springref="-3.14"
F1_knuckle_tip: stiffness="0.05114105365" springref="0"
```

**Tendon Sites:**
```xml
f1l2_peg: pos=".011 0 0"  ← Z offset removed!
f1l2_pin: pos=".0185 0 0"
```

**Key Changes:**
- Z offset removed from peg (was -.004)
- springref changed to 0 (from -1.57)
- Range changed to -0.01 to 1.2

---

### Commit: `eb39894`
**Date:** 2025-11-25 23:51:17 -0800  
**Message:** Add mechanical stops to enforce joint limits in EZGripper  
**Author:** SAKE Robotics

**Joint Parameters:**
```xml
F1_palm_knuckle: stiffness="0.02459949416" springref="-3.14"
F1_knuckle_tip: stiffness="0.05114105365" springref="-1.57"
```

**Tendon Sites:**
```xml
f1l2_peg: pos=".011 0 -.004"  ← Z offset present
f1l2_pin: pos=".0185 0 0"
```

---

### Commit: `fa01a8d`
**Date:** 2025-11-21 14:32:00 -0800  
**Message:** Fix EZGripper spring configuration for under-actuation  
**Author:** SAKE Robotics

**Joint Parameters:**
```xml
F1_palm_knuckle: stiffness="0.02459949416" springref="-3.14"
F1_knuckle_tip: stiffness="0.05114105365" springref="-1.57"
```

**Key Change:** Springs CORRECTED! Palm and tip stiffness values swapped to correct values.

---

### Commit: `6defecf`
**Date:** 2021-04-13 00:28:02 -0700  
**Message:** Minor: Updating the pulley dims  
**Author:** Vikash Kumar

**Joint Parameters:**
```xml
F1_palm_knuckle: stiffness="0.05114105365" springref="-3.14"  ← WRONG!
F1_knuckle_tip: stiffness="0.02459949416" springref="-1.57"   ← WRONG!
```

**Note:** Springs were SWAPPED in original model!

---

### Commit: `acff90d` (ORIGINAL)
**Date:** 2021-04-01 15:34:51 +0530  
**Message:** Adding initial working model  
**Author:** Vikash Kumar

**Joint Parameters:**
```xml
F1_palm_knuckle: stiffness="0.05114105365" springref="-3.14"  ← WRONG!
F1_knuckle_tip: stiffness="0.02459949416" springref="-1.57"   ← WRONG!
```

**Tendon Sites:**
```xml
f1l2_peg: pos=".011 0 -.004"
f1l2_pin: pos=".0185 0 0"
```

---

## Critical Findings

### 1. Spring Stiffness Error (2021-2025)
The original model had **swapped spring values**:
- Palm had 0.0511 (should be 0.0246)
- Tip had 0.0246 (should be 0.0511)

This was fixed in commit `fa01a8d` on 2025-11-21.

### 2. Spring Values vs Actual Specs (2026-01-03)
Even the "corrected" values were **4-6x too high**:
- Palm: 0.0246 N⋅m/rad (should be 0.00593 from spring spec)
- Tip: 0.0511 N⋅m/rad (should be 0.00771 from spring spec)

### 3. L1/L2 springref Change (2025-11-30)
In commit `62a0ae7`, springref changed from **-1.57 to 0**:
- Old: Spring pulls OPEN (toward -90°)
- New: Spring pulls to 0° (neutral)

This is a **major physics change**!

### 4. Z Offset Removal (2025-11-30)
The peg Z offset (-.004) was removed in the same commit. User confirmed this was **wrong** - tendon must stay in same plane (Z=0).

---

## Current Issues (2026-01-03)

1. **L1/L2 rotating backward** - Tendon creates wrong torque direction
2. **Spring rates too high** - Fixed today with actual specs (0.00593, 0.00771)
3. **springref confusion** - Changed multiple times (-1.57 → 0 → -1.57)
4. **Tendon routing** - May need peg/pin position adjustment for correct torque

## Recommendations

1. **Use actual spring specs**: 0.00593 (palm), 0.00771 (tip) ✅ Done today
2. **Keep tendon in plane**: Z=0 for all sites ✅ Confirmed
3. **Verify springref direction**: Need to determine correct preload direction
4. **Test tendon routing**: May need to adjust peg/pin positions for correct closing torque
