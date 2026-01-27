"""
ESP32 WROOM Servo Sweep Controller
MicroPython script that moves a servo motor back and forth continuously (0-180 degrees).

Wiring:
- Servo Signal (Orange/Yellow) -> GPIO 4 (D4)
- Servo VCC (Red) -> 5V (or external power supply)
- Servo GND (Brown/Black) -> GND

Upload this to your ESP32 using Thonny, ampy, or similar tools.
"""

from machine import Pin, PWM
from time import sleep

# Configuration
SERVO_PIN = 4           # GPIO pin D4 connected to servo signal wire
SERVO_FREQ = 50         # Standard servo frequency (50Hz = 20ms period)

# Pulse width in microseconds for servo positions
# Standard servos: 500us = 0°, 2500us = 180°
# Adjust these if your servo needs different values
MIN_PULSE_US = 500      # Pulse width for 0 degrees (microseconds)
MAX_PULSE_US = 2500     # Pulse width for 180 degrees (microseconds)

SWEEP_DELAY = 0.02      # Delay between each step (seconds)

def angle_to_duty_ns(angle):
    """
    Convert angle (0-180) to duty cycle in nanoseconds.
    This uses duty_ns() for precise servo control.
    """
    # Map 0-180 degrees to pulse width range
    pulse_us = MIN_PULSE_US + (MAX_PULSE_US - MIN_PULSE_US) * angle / 180
    # Convert microseconds to nanoseconds
    return int(pulse_us * 1000)

def setup_servo(pin_num):
    """Initialize and return a PWM object for the servo."""
    servo = PWM(Pin(pin_num))
    servo.freq(SERVO_FREQ)
    return servo

def sweep_servo(servo):
    """
    Continuously sweep the servo back and forth from 0 to 180 degrees.
    Press Ctrl+C to stop.
    """
    print("Starting servo sweep (0° to 180°)...")
    print("Press Ctrl+C to stop")
    
    try:
        while True:
            # Sweep from 0 to 180 degrees
            print("Sweeping: 0° -> 180°")
            for angle in range(0, 181, 5):
                duty_ns = angle_to_duty_ns(angle)
                servo.duty_ns(duty_ns)
                sleep(SWEEP_DELAY)
            
            sleep(0.5)  # Pause at 180 degrees
            
            # Sweep from 180 to 0 degrees
            print("Sweeping: 180° -> 0°")
            for angle in range(180, -1, -5):
                duty_ns = angle_to_duty_ns(angle)
                servo.duty_ns(duty_ns)
                sleep(SWEEP_DELAY)
            
            sleep(0.5)  # Pause at 0 degrees
            
    except KeyboardInterrupt:
        print("\nStopping servo...")
        servo.duty_ns(0)  # Stop sending signal
        servo.deinit()
        print("Servo stopped and PWM released.")

def main():
    """Main entry point."""
    print("=" * 40)
    print("ESP32 Servo Sweep Controller")
    print("=" * 40)
    print(f"Servo on GPIO {SERVO_PIN}")
    print(f"Pulse range: {MIN_PULSE_US}us (0°) to {MAX_PULSE_US}us (180°)")
    print()
    
    # Initialize servo
    servo = setup_servo(SERVO_PIN)
    
    # Center the servo first
    print("Centering servo at 90°...")
    servo.duty_ns(angle_to_duty_ns(90))
    sleep(1)
    
    # Start sweeping
    sweep_servo(servo)

# Run the main function when script is executed
if __name__ == "__main__":
    main()
