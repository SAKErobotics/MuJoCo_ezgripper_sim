# Complete EZGripper XML Change History

All changes to ezgripper.xml organized chronologically with full details.

---

## Commit Timeline (Newest to Oldest)

### 2026-01-03: `e7bb7e4` - Fix EZGripper tendon force calibration and symmetric closing

**Changes:**
- Extended L1/L2 tip joint range: `-0.01 to 1.2` → **`-1.57 to 1.2`**
- Increased tip joint upper limit range significantly
- Maintained springref="0" for tip joints
- No other geometry changes

---

### 2026-01-02: `4ab2604` - FIX: Implement symmetric closing with tendon equality constraint

**Major Changes:**
- **Added equality constraint** between finger1_tendon and finger2_tendon
- Changed actuator from motor to **tendon equality**
- Removed separate actuators for each finger
- Changed motor gear from -200 to single tendon actuator

**Actuator Changes:**
```xml
OLD: <motor tendon="finger1_tendon" gear="-200" name="gripper_actuator"/>
NEW: <tendon tendon1="finger1_tendon" tendon2="finger2_tendon" equality constraint/>
```

---

### 2025-11-30: `62a0ae7` - Fix gripper finger synchronization and enable finger contact

**Critical Changes:**
1. **L1/L2 springref changed:** `-1.57` → **`0`** (MAJOR PHYSICS CHANGE!)
2. **L1/L2 range changed:** `0 to 1.7` → **`-0.01 to 1.2`**
3. **Removed Z offsets from tendon sites:**
   - `f1l1_peg0`: `.0115 0 -.007` → `.0115 0 0`
   - `f1l1_peg1`: `.0355 0 -.006` → `.0355 0 0`
   - `f1l2_peg`: `.011 0 -.004` → `.011 0 0`
   - `f2l1_peg0`: `.0115 0 -.007` → `.0115 0 0`
   - `f2l1_peg1`: `.0355 0 -.006` → `.0355 0 0`
   - `f2l2_peg`: `.011 0 -.004` → `.011 0 0`

4. **Enabled L1/L2 collision:**
   - `f1l2`: contype="0" → **contype="3"**
   - `f2l2`: contype="0" → **contype="3"**

5. **Removed L2 pulleys from tendon path:**
   - Removed `<geom geom="f1l2_pulley"/>` from tendon
   - Removed `<geom geom="f2l2_pulley"/>` from tendon

6. **Disabled mechanical stops** (commented out)

7. **Increased actuator kv:** 100 → **1000**

---

### 2025-11-25: `eb39894` - Add mechanical stops to enforce joint limits in EZGripper

**Changes:**
1. **Added mechanical stop collision boxes:**
   - Palm stops (lower and upper limits)
   - L1 stops (lower and upper limits)
   - L1-L2 joint stop

2. **Adjusted finger base position:**
   - `F1_L1`: pos="0.0755" → **"0.07255"**
   - `F2_L1`: pos="0.0755" → **"0.07255"**

3. **Palm range changed:**
   - `-1.57 to 0.35` → **`-1.57075 to 0.436`**

4. **Reduced damping:**
   - Palm: 1.0 → **0.05**
   - Tip: 1.0 → **0.05**

5. **Removed solimplimit/solreflimit** from joints

6. **Re-enabled L1 collision:**
   - `f1l1`: contype="0" → **contype="1"**
   - `f2l1`: contype="0" → **contype="1"**

---

### 2025-11-25: `3ddeb14` - Add heavy damping (0.1) to all gripper joints for realistic slow motion

**Changes:**
1. **Added option parameters:**
   - timestep="0.001"
   - iterations="100"
   - solver="Newton"
   - tolerance="1e-10"

2. **Removed contact exclusions:**
   - Deleted: `<exclude body1="mount" body2="F1_L1"/>`
   - Deleted: `<exclude body1="mount" body2="F2_L1"/>`

3. **Added mechanical stops on palm**

4. **Increased damping:**
   - All joints: 0.05 → **1.0** (NOTE: Title says 0.1 but actual is 1.0!)

---

### 2025-11-25: `2c1f7ef` - Reverse spring directions to provide default open gripper behavior

**Changes:**
1. **Reversed palm spring direction:**
   - springref="-1.57" → **"-3.14"**

2. **Changed L1/L2 range:**
   - `0.5236 to 1.7` → **`0 to 1.7`**

---

### 2025-11-25: `8e2b1df` - Refine EZGripper joint ranges and spring behavior

**Changes:**
1. **Palm range changed:**
   - `-1.57075 to 0.27` → **`-1.57 to 0.35`**

2. **Adjusted finger position:**
   - pos="0.07255" → **"0.0755"**

3. **Disabled L1 collision:**
   - contype="1" → **contype="0"**

---

### 2025-11-21: `ef8aa54` - Implement mesh-based collision for realistic under-actuation

**Major Changes:**
1. **Enabled mesh collision for L1:**
   - Changed from box collision to mesh collision
   - `f1l1`: Added mesh collision geom
   - `f2l1`: Added mesh collision geom

2. **Added finger pad mesh collision:**
   - `f1_tip`: type="mesh" with collision
   - `f2_tip`: type="mesh" with collision

3. **Removed box-based collision geoms**

4. **Changed friction values:**
   - Finger links: friction="1.5 0.01 0.0001"

---

### 2025-11-21: `026b683` - Add tennis ball to simulation at 160mm in front of origin

**Changes:**
1. **Added tennis ball body** (commented out in later versions)
2. **Palm range changed:**
   - `-1.57075 0.436` → **`-1.57075 0.27`**

---

### 2025-11-21: `6fb70ec` - Set L1-L2 joint minimum angle to 30° (30° to 97° range)

**Changes:**
1. **L1/L2 range changed:**
   - `0.2618 to 1.7` → **`0.5236 to 1.7`** (30° minimum)

---

### 2025-11-21: `ff7f27f` - Set L1-L2 hard stop at +15° closing angle

**Changes:**
1. **L1/L2 range changed:**
   - `-0.2618 to 1.7` → **`0.2618 to 1.7`** (15° minimum)

---

### 2025-11-21: `d0f5fd8` - Set L1-L2 hard stop at 15° opening angle

**Changes:**
1. **L1/L2 range changed:**
   - `0 to 1.7` → **`-0.2618 to 1.7`** (allow 15° opening)

---

### 2025-11-21: `c57e9ad` - Set L1-L2 hard stop at 0° using joint limits (clean solution)

**Changes:**
1. **L1/L2 range changed:**
   - `-0.5 to 1.7` → **`0 to 1.7`** (no negative angles)

---

### 2025-11-21: `4e02511` - Set L1-L2 hard stop at 0° joint angle (no opening allowed)

**Changes:**
1. **L1/L2 range changed:**
   - `-0.5 to 1.7` → **`-0.5 to 1.7`** (same, but added mechanical stops)

---

### 2025-11-21: `8957985` - Final L1-L2 collision implementation using proxy collision boxes

**Changes:**
1. **Added L2 collision boxes** as proxies
2. **L1/L2 range set to:** `-0.5 to 1.7`

---

### 2025-11-21: `fa01a8d` - Fix EZGripper spring configuration for under-actuation

**CRITICAL FIX:**
1. **Swapped spring stiffness values** (they were backwards!):
   - Palm: `0.05114105365` → **`0.02459949416`**
   - Tip: `0.02459949416` → **`0.05114105365`**

2. **Changed palm springref:**
   - `-3.14` → **`-3.14`** (kept same)

3. **L1/L2 range changed:**
   - `-0.5 to 1.7` → **`0 to 1.7`**

---

### 2021-04-13: `6defecf` - Minor: Updating the pulley dims

**Changes:**
1. **Updated pulley dimensions** (minor adjustments)

---

### 2021-04-01: `acff90d` - Adding initial working model

**Initial Configuration:**
- Palm stiffness: **0.05114105365** (WRONG - too high)
- Tip stiffness: **0.02459949416** (WRONG - too low)
- Palm springref: **-3.14**
- Tip springref: **-1.57**
- L1/L2 range: **0 to 1.7**
- Tendon sites with Z offsets:
  - `f1l2_peg`: `.011 0 -.004`
  - `f1l1_peg0`: `.0115 0 -.007`
  - `f1l1_peg1`: `.0355 0 -.006`

---

## Summary of Key Parameter Evolution

### Spring Stiffness (N⋅m/rad)
| Date | Palm | Tip | Notes |
|------|------|-----|-------|
| 2021-04-01 | 0.0511 | 0.0246 | **SWAPPED** (wrong) |
| 2025-11-21 | **0.0246** | **0.0511** | Fixed |
| **2026-01-03** | **0.00593** | **0.00771** | **From actual specs** |

### L1/L2 Tip Joint springref
| Date | Value | Meaning |
|------|-------|---------|
| 2021-04-01 | -1.57 | Pull OPEN (toward -90°) |
| 2025-11-30 | **0** | Pull to neutral (0°) |
| **2026-01-03** | **-1.57** | Reverted to OPEN |

### L1/L2 Tip Joint Range
| Date | Range | Notes |
|------|-------|-------|
| 2021-04-01 | 0 to 1.7 | No negative opening |
| 2025-11-21 | -0.5 to 1.7 | Allow opening |
| 2025-11-21 | 0 to 1.7 | Hard stop at 0° |
| 2025-11-21 | 0.5236 to 1.7 | 30° minimum |
| 2025-11-25 | 0 to 1.7 | Back to 0° |
| 2025-11-30 | **-0.01 to 1.2** | Small negative, reduced max |
| 2026-01-03 | **-1.57 to 1.2** | Full negative range |

### Tendon Site Z Offsets
| Date | f1l2_peg | f1l1_peg0 | f1l1_peg1 |
|------|----------|-----------|-----------|
| 2021-04-01 | -.004 | -.007 | -.006 |
| 2025-11-30 | **0** | **0** | **0** |
| **Current** | **0** | **0** | **0** |

### Damping
| Date | Value | Notes |
|------|-------|-------|
| 2021-04-01 | 0.05 | Low damping |
| 2025-11-25 | **1.0** | High damping |
| 2025-11-25 | **0.05** | Back to low |

### Actuator Type
| Date | Type | Details |
|------|------|---------|
| 2021-2025 | motor | gear=-200, single actuator |
| 2026-01-02 | **equality** | Tendon equality constraint |

---

## Critical Issues Found

1. **Springs were swapped** for 4+ years (2021-2025)
2. **Spring values 4-6x too high** even after fix
3. **springref changed** from -1.57 to 0 (major physics change)
4. **Z offsets removed** from all tendon sites
5. **L2 pulleys removed** from tendon path
6. **Actuator type changed** from motor to equality constraint
7. **Multiple range adjustments** to L1/L2 joint
