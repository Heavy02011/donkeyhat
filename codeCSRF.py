# Donkey Car Driver with CRSF (GP27/GP26) and E-Stop (CH5)
import time
import board
import busio
import neopixel
from pwmio import PWMOut
import digitalio
import rotaryio

# Customisation variables
DEBUG = False
USB_SERIAL = False
SMOOTHING_INTERVAL_IN_S = 0.020 
USE_QUADRATURE = False 

# CRSF Setup: Using GP27 for RX (RC1) and GP26 for TX (RC2)
# This allows bidirectional communication with CRSF/ELRS receivers
rc_uart = busio.UART(board.GP26, board.GP27, baudrate=420000, timeout=0)

# Hardware Setup for PWM Outputs
steering_pwm = PWMOut(board.GP11, duty_cycle=0, frequency=60)
throttle_pwm = PWMOut(board.GP10, duty_cycle=0, frequency=60)

# Encoder Setup
if USE_QUADRATURE:
    encoder1 = rotaryio.IncrementalEncoder(board.GP8, board.GP9)
    encoder2 = rotaryio.IncrementalEncoder(board.GP13, board.GP14)
else:
    encoder1 = digitalio.DigitalInOut(board.GP8)
    encoder1.direction = digitalio.Direction.INPUT
    encoder1.pull = digitalio.Pull.DOWN
    encoder2 = digitalio.DigitalInOut(board.GP13)
    encoder2.direction = digitalio.Direction.INPUT
    encoder2.pull = digitalio.Pull.DOWN

pixel = neopixel.NeoPixel(board.NEOPIXEL, 1)
uart_pi = busio.UART(board.TX, board.RX, baudrate=115200, timeout=0)

# Global States
rc_values = {"steer": 1500, "throttle": 1500, "estop": 1000}
position1, position2 = 0, 0
continuous_mode = False
continuous_delay = 0
datastr = ""

def servo_duty_cycle(pulse_ms, frequency=60):
    period_ms = 1000.0 / frequency
    duty_cycle = int((pulse_ms / 1000.0) / (period_ms / 65535.0))
    return max(0, min(65535, duty_cycle))

def parse_crsf(data):
    """Parses CRSF RC_CHANNELS_PACKED (0x16) frame"""
    global rc_values
    if len(data) < 26 or data[2] != 0x16: 
        return
    
    # CRSF uses 11-bit values (172=1000ms, 992=1500ms, 1811=2000ms)
    # Bit-packing logic for CRSF channels
    raw_steer = ((data[3] | data[4] << 8) & 0x07FF)
    raw_thr   = ((data[4] >> 3 | data[5] << 5) & 0x07FF)
    raw_ch5   = ((data[5] >> 6 | data[6] << 2 | data[7] << 10) & 0x07FF)
    
    # Map to standard PWM 1000-2000 range
    rc_values["steer"] = int((raw_steer - 992) * 0.625 + 1500)
    rc_values["throttle"] = int((raw_thr - 992) * 0.625 + 1500)
    rc_values["estop"] = int((raw_ch5 - 992) * 0.625 + 1500)

def main():
    global position1, position2, datastr, continuous_mode
    last_update = time.monotonic()
    last_input_pi = 0
    crsf_buffer = bytearray()
    
    if not USE_QUADRATURE:
        last_state1, last_state2 = encoder1.value, encoder2.value

    while True:
        current_time = time.monotonic()

        # 1. Read CRSF Data from Receiver (UART1 on GP26/27)
        if rc_uart.in_waiting:
            b = rc_uart.read(1)
            if b == b'\xc8': # Sync byte
                crsf_buffer = bytearray(b'\xc8')
            elif len(crsf_buffer) > 0:
                crsf_buffer.extend(b)
                if len(crsf_buffer) >= 2 and len(crsf_buffer) == crsf_buffer[1] + 2:
                    parse_crsf(crsf_buffer)
                    crsf_buffer = bytearray()

        # 2. Encoder Logic
        if USE_QUADRATURE:
            position1, position2 = encoder1.position, encoder2.position
        else:
            curr1, curr2 = encoder1.value, encoder2.value
            if curr1 != last_state1 and not curr1: position1 += 1
            if curr2 != last_state2 and not curr2: position2 += 1
            last_state1, last_state2 = curr1, curr2

        # 3. Process Serial from Pi (UART0 on TX/RX pins)
        incoming_pi = uart_pi.read(32)
        pi_steer, pi_thr = None, None
        if incoming_pi:
            for b in incoming_pi:
                char = chr(b)
                if char in ('\r', '\n'):
                    if len(datastr) >= 8 and datastr[0].isdigit():
                        try:
                            pi_steer = int(datastr[:4])
                            pi_thr = int(datastr[-4:])
                            last_input_pi = current_time
                        except ValueError: pass
                    datastr = ""
                else: datastr += char

        # 4. Control Handover & E-Stop
        # Check Channel 5: If High (>1700), force E-Stop
        if rc_values["estop"] > 1700:
            final_throttle = 1500 
            final_steer = rc_values["steer"]
            pixel.fill((255, 0, 0)) # Red for E-Stop
        else:
            pixel.fill((0, 0, 255)) # Blue for Normal
            # Use Pi control if recent, otherwise fall back to RC
            if current_time - last_input_pi < 0.2 and pi_thr is not None:
                final_throttle = pi_thr
                final_steer = pi_steer
            else:
                final_throttle = rc_values["throttle"]
                final_steer = rc_values["steer"]

        # Final PWM output
        steering_pwm.duty_cycle = servo_duty_cycle(final_steer)
        throttle_pwm.duty_cycle = servo_duty_cycle(final_throttle)
        pixel.show()

print("Donkey Driver Ready: CRSF on GP26/27, E-Stop on CH5")
main()