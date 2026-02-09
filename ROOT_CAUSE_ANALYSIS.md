# Root Cause Analysis: Steering Stops Working with `python manage.py drive`

## Summary

When running `python manage.py drive` on certain DonkeyCar setups using the DonkeyHat (RP2040-based RC Hat), steering stops responding. The same `code.py` firmware works on zlite's car but fails on other cars. This document provides a detailed root cause analysis and fix.

## Root Cause Identified: Serial Parsing Bug in `code.py`

### The Bug

The RP2040 `code.py` has a critical flaw in how it processes incoming serial data from the Raspberry Pi.

**How DonkeyCar sends servo commands (Pi → RP2040):**

The `RoboHATDriver.write_pwm()` in `donkeycar/parts/robohat.py` sends:
```python
self.pwm.write(b"%d, %d\r" % (steering, throttle))
# Example: b"1500, 1500\r"
```

**How the RP2040 was parsing it (BUGGY):**

The inner byte-reading loop had two separate code paths:

1. **In-loop path** (`len(datastr) >= 10`): Parses servo data as bytes accumulate — works when all 10+ data bytes arrive before `\r`
2. **`\r` handler path**: Passes accumulated data to `handle_command()` — but `handle_command()` only knows about `'r'`, `'p'`, and `'c'` commands, **NOT servo PWM data**

```python
# BUGGY: When \r arrives, servo data goes to handle_command() which ignores it
if byte == b'\r':
    command = datastr.strip()
    datastr = ''
    handle_command(command)  # ← Silently drops "1500, 1500"!
    data = bytearray()
    break
```

**When does this fail?**

The bug manifests when the `\r` byte arrives BEFORE the in-loop `len(datastr) >= 10` check has fired. This can happen due to:

- **UART timing variations**: Different RP2040 boards, crystal oscillator tolerances, or CircuitPython versions may cause `uart.read(1)` to return `None` between bytes, splitting the data across multiple main loop iterations
- **Buffer fragmentation**: When the main loop's smoothing interval (`continue` at line 196-197) delays UART reads, partial data accumulates and is split across reads
- **`strip()` side effects**: The `datastr = ''.join([chr(c) for c in data]).strip()` call removes trailing spaces from `"1500, "`, which can affect the character count progression

**Why it worked on zlite's car:** On zlite's specific hardware, the UART timing likely ensures all 10+ data bytes are always available before `\r`, so the in-loop parse always fires first. On other hardware, timing differences cause the `\r` to be processed before the in-loop check triggers.

### The Fix

The `\r` handler now checks if the accumulated data is servo PWM data (starts with a digit, 10+ chars) and parses it directly, falling back to `handle_command()` only for actual commands:

```python
if byte == b'\r':
    command = datastr.strip()
    datastr = ''
    data = bytearray()
    # Check if this is servo PWM data (e.g. "1500, 1500") or a command
    if len(command) >= 10 and command[0].isdigit():
        try:
            steering_val = int(command[:4])
            throttle_val = int(command[-4:])
            got_data = True
        except ValueError:
            handle_command(command)
    elif command:
        handle_command(command)
    break
```

This ensures servo data is **always** parsed correctly, regardless of byte arrival timing.

### Reference: Upstream Code Comparison

The original upstream code in `autorope/donkeycar/contrib/robohat/code.py` handles this differently — it preserves `datastr` when `\r` is received and checks it **outside** the inner while loop:

```python
# Upstream: datastr preserved across \r, checked OUTSIDE inner loop
if byte == b'\r':
    data = bytearray('')  # Only clears data, NOT datastr
    break

# Outside inner loop:
if len(datastr) >= 10:
    # Parse servo data here
```

---

## Additional Diagnostic Checklist

If the fix above doesn't fully resolve the issue, work through these diagnostics:

### 1. Verify Serial Communication

```bash
# On the Raspberry Pi, test sending servo commands directly:
echo -ne "1500,1500\r" > /dev/ttyAMA0

# Monitor what the RP2040 sends back:
cat /dev/ttyAMA0
# Should see: "1500, 1500" (RC values from the RP2040)
```

### 2. Check Serial Port Configuration

```bash
# Verify serial port is enabled and at correct baud rate:
stty -F /dev/ttyAMA0

# Ensure Bluetooth is not using the serial port (Pi 3/4/5):
# In /boot/config.txt (or /boot/firmware/config.txt on newer OS):
dtoverlay=disable-bt

# Ensure serial console is disabled:
sudo raspi-config  # → Interface Options → Serial Port
# Answer: No to login shell, Yes to hardware enabled
```

### 3. Verify myconfig.py Settings

```python
# These must be set for RoboHat/DonkeyHat:
DRIVE_TRAIN_TYPE = "MM1"
CONTROLLER_TYPE = 'MM1'
MM1_SERIAL_PORT = '/dev/ttyAMA0'  # Must match actual port
```

### 4. Check CircuitPython Version and Board

```
# Connect the RP2040 via USB and check boot_out.txt on the CIRCUITPY drive
# Expected: CircuitPython 10.x on a supported RP2040 board
# The UART behavior can differ between CircuitPython versions
```

### 5. Verify RC Receiver Wiring

| Signal | RP2040 Pin | Purpose |
|--------|-----------|---------|
| RC CH1 (Steering) | GP27 | PulseIn for steering RC input |
| RC CH2 (Throttle) | GP26 | PulseIn for throttle RC input |
| RC CH3 (Mode/E-Stop) | GP29 | PulseIn for e-stop input |
| Steering Servo | GP11 | PWM output to steering servo |
| Throttle ESC | GP10 | PWM output to ESC |
| UART TX | TX pin | Serial to Pi RX |
| UART RX | RX pin | Serial from Pi TX |

**Common wiring issues:**
- TX/RX crossed (RP2040 TX → Pi RX, RP2040 RX → Pi TX)
- Ground not shared between RP2040 and Pi
- RC receiver signal wires on wrong channels

### 6. E-Stop False Trigger

The e-stop activates when CH3 > 1700. If no RC receiver is connected to GP29, the `PulseIn` channel may pick up noise that triggers e-stop, forcing throttle to 1500 (neutral) while steering appears to work but the car doesn't respond.

**Test:** Set `ESTOP_THRESHOLD = 2100` temporarily to rule out false triggering.

### 7. Check for Dual Serial Port Opening Issue

The DonkeyCar framework creates BOTH `RoboHATController` (reads serial) and `RoboHATDriver` (writes serial), each opening `MM1_SERIAL_PORT` separately. On some Linux configurations, this can cause data races where incoming data is split between the two serial instances.

**Symptom:** RC values from the RP2040 appear garbled or missing in the Pi's web UI.

### 8. Debug Serial Data Flow

Add temporary debug output to the RP2040 `code.py`:

```python
# Uncomment the print in the got_data section (line ~266):
print("Set: steering=%i, throttle=%i" % (steering_val, throttle_val))
```

Then monitor the RP2040's USB console (via `screen /dev/ttyACM0 115200` or Mu editor) while running `manage.py drive` on the Pi. You should see alternating "RC control" and "Serial control" messages. If you only see "RC control", the serial parsing is failing.

### 9. Verify Steering Servo Response

```bash
# Test steering independently via calibration commands:
echo -ne "1000,1500\r" > /dev/ttyAMA0   # Steering right
sleep 1
echo -ne "1500,1500\r" > /dev/ttyAMA0   # Steering center
sleep 1
echo -ne "2000,1500\r" > /dev/ttyAMA0   # Steering left
```

If the servo responds to these but not to `manage.py drive`, the issue is in the serial parsing (fixed above).

### 10. PWM Frequency and Servo Compatibility

The code uses 60Hz PWM frequency. Most RC servos expect 50Hz. While 60Hz usually works, some servos may be sensitive:

```python
# In code.py, try changing frequency to 50:
steering_pwm = PWMOut(Steering, duty_cycle=2 ** 15, frequency=50)
throttle_pwm = PWMOut(Throttle, duty_cycle=2 ** 15, frequency=50)
```

Also update the `servo_duty_cycle()` calls if you change the frequency.
