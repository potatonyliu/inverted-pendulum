import pygame
import time
from telemetrix_rpi_pico import telemetrix_rpi_pico

# ==========================================
# 1. HARDWARE SETUP (PICO PUPPET)
# ==========================================
ENA = 7  # PWM speed control
IN1 = 5  # Direction control
IN2 = 6  # Direction control

print("🔌 Connecting to Pico over USB...")
board = telemetrix_rpi_pico.TelemetrixRpiPico()

board.set_pin_mode_digital_output(IN1)
board.set_pin_mode_digital_output(IN2)
board.set_pin_mode_pwm_output(ENA)

# ==========================================
# 2. JOYSTICK SETUP (MAC)
# ==========================================
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("❌ No joystick found! Please make sure your 8BitDo is paired via Bluetooth.")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"✅ Connected to: {joystick.get_name()}")

# ==========================================
# 3. MOTOR CONTROL LOGIC
# ==========================================
def map_value(value, in_min, in_max, out_min, out_max):
    # Converts Pygame's trigger scale (-1.0 to 1.0) to our safe PWM scale (0 to 99)
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def forward(speed):
    board.digital_write(IN1, 0)
    board.digital_write(IN2, 1)
    board.pwm_write(ENA, speed)


def backward(speed):
    board.digital_write(IN1, 1)
    board.digital_write(IN2, 0)
    board.pwm_write(ENA, speed)

def stop_motor():
    board.digital_write(IN1, 0)
    board.digital_write(IN2, 0)
    board.pwm_write(ENA, 0)

# ==========================================
# 4. MAIN LOOP 
# ==========================================
print("🎮 Bridge active! Press the triggers to move... (Ctrl+C to stop)")

try:
    while True:
        pygame.event.pump()
        
        # Axis 4 (Right Trigger), Axis 5 (Left Trigger)
        right_trigger = joystick.get_axis(4) 
        left_trigger = joystick.get_axis(5)
        
        # If Right Trigger is pulled (greater than resting state of -1.0)
        if right_trigger > -0.9: 
            speed = int(map_value(right_trigger, -1.0, 1.0, 0, 99))
            forward(speed)
            print(f"⏩ Forward: {speed}%")
            
        # If Left Trigger is pulled
        elif left_trigger > -0.9: 
            speed = int(map_value(left_trigger, -1.0, 1.0, 0, 99))
            backward(speed)
            print(f"⏪ Backward: {speed}%")
            
        # If neither is pulled, stop
        else:
            stop_motor()
            
        time.sleep(0.05) # Run 20 times a second

except KeyboardInterrupt:
    print("\n🛑 Shutting down safely...")
    stop_motor()
    board.shutdown()
    pygame.quit()