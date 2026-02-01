import time
import RPi.GPIO as GPIO
from hx711 import HX711

# Setup HX711
hx = HX711(16, 20)
hx.set_reading_format("MSB", "MSB")
hx.set_reference_unit(406.84)  # Use your calculated reference unit here
hx.reset()
hx.tare()

print("Scale ready! Place items to weigh...")
print("Press Ctrl+C to exit")

try:
    while True:
        weight = hx.get_weight(3)  # Average of 3 readings
        print(f"Weight: {weight:.1f}g")
        
        hx.power_down()
        hx.power_up()
        time.sleep(0.5)
        
except KeyboardInterrupt:
    print("\nExiting...")
    GPIO.cleanup()