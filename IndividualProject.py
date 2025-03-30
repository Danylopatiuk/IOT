import time
from smbus2 import SMBus, i2c_msg
import requests
import threading
import wiringpi
import sys
import termios
import tty

bus = SMBus(0)

TARGET_LUX = 50

wiringpi.wiringPiSetup()
BUTTON_PIN = 14
wiringpi.pinMode(BUTTON_PIN, wiringpi.INPUT)
wiringpi.pullUpDnControl(BUTTON_PIN, wiringpi.PUD_UP)

LED_PIN = 2
wiringpi.softPwmCreate(LED_PIN, 0, 100)
current_brightness = 0
wiringpi.softPwmWrite(LED_PIN, current_brightness)


def set_brightness(brightness):
    global current_brightness

    brightness = max(0, min(brightness, 100))
    current_brightness = brightness
    wiringpi.softPwmWrite(LED_PIN, brightness)
    print(f"\nLED Brightness: {brightness}%")


def kbhit():
    dr, dw, de = select.select([sys.stdin], [], [], 0)
    return dr != []


print("Select run mode:")
print("1 - General run (timed)")
print("2 - Predefined run (runs until lux < 50 and temp < 25) - Press button to start")
mode_choice = input("Enter your choice (1 or 2): ")

if mode_choice == "1":
    try:
        runtime_seconds = float(input("For how long (in seconds) do you want to run the code? "))
    except ValueError:
        print("Invalid input. Using default 20 seconds.")
        runtime_seconds = 20.0
    mode = "general"
elif mode_choice == "2":
    print("Waiting for button press - short press for normal mode, long press (2s) for motor-until-red mode")


    button_pressed_time = None
    while True:
        if wiringpi.digitalRead(BUTTON_PIN) == 0:
            if button_pressed_time is None:
                button_pressed_time = time.time()
        else:
            if button_pressed_time is not None:
                press_duration = time.time() - button_pressed_time


                if press_duration < 2.0:
                    mode = "predefined"
                    print("Normal mode: Will stop when lux < 50 and temp < 25")
                    break


                else:
                    mode = "motor_until_red"
                    print("Motor-until-red mode: Motor will run until RGB turns red")
                    break

            button_pressed_time = None

        time.sleep(0.05)

    time.sleep(0.5)
else:
    print("Invalid choice. Defaulting to General run with 20 seconds.")
    mode = "general"
    runtime_seconds = 20.0



THINGSPEAK_API_KEY = "ZZJ2FTT1YOBNPEMP"
THINGSPEAK_URL = "https://api.thingspeak.com/update"

BH1750_ADDR = 0x23
BMP280_ADDR = 0x77

CALIBRATION_START = 0x88
TEMP_MSB_REG = 0xFA
PRESSURE_MSB_REG = 0xF7
CTRL_MEAS_REG = 0xF4

bus.write_byte(BH1750_ADDR, 0x10)
bus.write_byte_data(BMP280_ADDR, CTRL_MEAS_REG, 0x27)


motor_pins = [3, 4, 6, 9]
wiringpi.wiringPiSetup()
for pin in motor_pins:
    wiringpi.pinMode(pin, 1)


step_sequence = [
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1]
]

RGB_PINS = {'R': 5, 'G': 7, 'B': 8}
for pin in RGB_PINS.values():
    wiringpi.pinMode(pin, 1)

def set_rgb_color(r, g, b):
    wiringpi.digitalWrite(RGB_PINS['R'], int(r))
    wiringpi.digitalWrite(RGB_PINS['G'], int(g))
    wiringpi.digitalWrite(RGB_PINS['B'], int(b))

def read_bh1750(bus, address):
    write = i2c_msg.write(address, [0x10])
    read = i2c_msg.read(address, 2)
    bus.i2c_rdwr(write, read)
    bytes_read = list(read)
    return (((bytes_read[0] & 3) << 8) + bytes_read[1]) / 1.2

def read_calibration_data(bus, address):
    calib = bus.read_i2c_block_data(address, CALIBRATION_START, 24)

    dig_T1 = calib[1] << 8 | calib[0]
    dig_T2 = (calib[3] << 8 | calib[2]) if calib[3] < 128 else ((calib[3] << 8 | calib[2]) - 65536)
    dig_T3 = (calib[5] << 8 | calib[4]) if calib[5] < 128 else ((calib[5] << 8 | calib[4]) - 65536)

    dig_P1 = calib[7] << 8 | calib[6]
    dig_P2 = (calib[9] << 8 | calib[8]) if calib[9] < 128 else ((calib[9] << 8 | calib[8]) - 65536)
    dig_P3 = (calib[11] << 8 | calib[10]) if calib[11] < 128 else ((calib[11] << 8 | calib[10]) - 65536)
    dig_P4 = (calib[13] << 8 | calib[12]) if calib[13] < 128 else ((calib[13] << 8 | calib[12]) - 65536)
    dig_P5 = (calib[15] << 8 | calib[14]) if calib[15] < 128 else ((calib[15] << 8 | calib[14]) - 65536)
    dig_P6 = (calib[17] << 8 | calib[16]) if calib[17] < 128 else ((calib[17] << 8 | calib[16]) - 65536)
    dig_P7 = (calib[19] << 8 | calib[18]) if calib[19] < 128 else ((calib[19] << 8 | calib[18]) - 65536)
    dig_P8 = (calib[21] << 8 | calib[20]) if calib[21] < 128 else ((calib[21] << 8 | calib[20]) - 65536)
    dig_P9 = (calib[23] << 8 | calib[22]) if calib[23] < 128 else ((calib[23] << 8 | calib[22]) - 65536)

    return (dig_T1, dig_T2, dig_T3, dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9)

def read_raw_temperature(bus, address):
    data = bus.read_i2c_block_data(address, TEMP_MSB_REG, 3)
    raw_temp = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    return raw_temp

highest_temperature = float()

def calculate_temperature(raw_temp, dig_T1, dig_T2, dig_T3):
    var1 = ((raw_temp / 16384.0) - (dig_T1 / 1024.0)) * dig_T2
    var2 = (((raw_temp / 131072.0) - (dig_T1 / 8192.0)) ** 2) * dig_T3
    t_fine = var1 + var2
    temperature = t_fine / 5120.0
    return temperature, t_fine

def read_raw_pressure(bus, address):
    data = bus.read_i2c_block_data(address, PRESSURE_MSB_REG, 3)
    raw_pressure = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    return raw_pressure

def calculate_pressure(raw_pressure, t_fine, dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9):
    var1 = (t_fine / 2.0) - 64000.0
    var2 = var1 * var1 * dig_P6 / 32768.0
    var2 = var2 + var1 * dig_P5 * 2.0
    var2 = (var2 / 4.0) + (dig_P4 * 65536.0)
    var1 = (dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1) / 524288.0
    var1 = (1.0 + var1 / 32768.0) * dig_P1
    if var1 == 0:
        return 0

    pressure = 1048576.0 - raw_pressure
    pressure = ((pressure - (var2 / 4096.0)) * 6250.0) / var1
    var1 = dig_P9 * pressure * pressure / 2147483648.0
    var2 = pressure * dig_P8 / 32768.0
    pressure = pressure + (var1 + var2 + dig_P7) / 16.0

    return pressure / 100.0

calibration_data = read_calibration_data(bus, BMP280_ADDR)

def blink_motor():
    for step in step_sequence:
        for pin, state in zip(motor_pins, step):
            wiringpi.digitalWrite(pin, state)
        time.sleep(0.001)


motor_running = False

motor_on_events = 0
motor_off_events = 0
motor_total_on_time = 0.0
motor_state_start_time = 0.0
is_motor_on = False


def run_motor():
    global motor_running, motor_on_events, motor_state_start_time, is_motor_on

    if not motor_running:
        motor_running = True
        if not is_motor_on:
            motor_on_events += 1
            motor_state_start_time = time.time()
            is_motor_on = True
        print("\r--Starting Motor--")
        while motor_running:
            blink_motor()
        stop_motor()


def stop_motor():
    global motor_running, motor_off_events, motor_total_on_time, motor_state_start_time, is_motor_on

    if motor_running or is_motor_on:
        if is_motor_on:
            motor_off_events += 1
            motor_total_on_time += time.time() - motor_state_start_time
            is_motor_on = False
        motor_running = False
        for pin in motor_pins:
            wiringpi.digitalWrite(pin, 0)
        print("\r--Motor Stopped--")

motor_thread = None

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
import select


tty.setraw(fd)


print("\rLED Control: Press 0-9 to set brightness", end="")


start_time = time.time()

try:
    if mode == "general":
        print(f"\rRunning program for {runtime_seconds} seconds")
    elif mode == "predefined":
        print("\rRunning in predefined mode (will stop when lux < 50 and temp < 25)")
    elif mode == "motor_until_red":
        print("\rRunning in motor-until-red mode (will stop only when RGB is red)")

    while True:

        if mode == "general" and (time.time() - start_time >= runtime_seconds):
            break
        elif mode == "predefined":

            lux = read_bh1750(bus, BH1750_ADDR)
            raw_temp = read_raw_temperature(bus, BMP280_ADDR)
            temperature, t_fine = calculate_temperature(raw_temp, *calibration_data[:3])

            if lux < 50 and temperature < 27:
                print("\rPredefined conditions met (lux < 50 and temp < 25). Stopping.")
                break
        elif mode == "motor_until_red":

            if not motor_running and (motor_thread is None or not motor_thread.is_alive()):
                motor_thread = threading.Thread(target=run_motor)
                motor_thread.start()

        if kbhit():
            key = sys.stdin.read(1)
            if key.isdigit():

                brightness = int(key) * 10
                set_brightness(brightness)

        # Read all sensor values
        lux = read_bh1750(bus, BH1750_ADDR)
        raw_temp = read_raw_temperature(bus, BMP280_ADDR)
        temperature, t_fine = calculate_temperature(raw_temp, *calibration_data[:3])
        raw_pressure = read_raw_pressure(bus, BMP280_ADDR)
        pressure = calculate_pressure(raw_pressure, t_fine, *calibration_data[3:])

        if temperature > highest_temperature:
            highest_temperature = temperature


        print(f"\r[Lux: {round(lux)}] [Temperature: {round(temperature, 2)} °C] [Pressure: {round(pressure, 2)} hPa]",end="")




        motor_status = 20 if motor_running else 0


        if temperature <= 26:
            set_rgb_color(0, 0, 1)
            current_rgb = "blue"
        elif 26 < temperature <= 27:
            set_rgb_color(0, 1, 0)
            current_rgb = "green"
        elif 27 < temperature <= 29:
            set_rgb_color(1, 1, 1)
            current_rgb = "yellow"
        elif 29 < temperature <= 32:
            set_rgb_color(1, 0, 0)
            current_rgb = "red"


        if mode == "motor_until_red" and current_rgb == "red":
            print("\rRGB is red. Stopping motor and exiting.")
            stop_motor()
            break


        if mode != "motor_until_red":
            if lux > TARGET_LUX and (motor_thread is None or not motor_thread.is_alive()):
                motor_thread = threading.Thread(target=run_motor)
                motor_thread.start()
            elif lux < TARGET_LUX:
                motor_running = False

        payload = {
            'api_key': THINGSPEAK_API_KEY,
            'field1': lux,
            'field2': temperature,
            'field3': pressure,
            'field4': motor_status,
            'field5': int(mode == "motor_until_red"),
            'field6': TARGET_LUX
        }
        response = requests.get(THINGSPEAK_URL, params=payload)

        # if response.status_code == 200:
        #     print("\n✅ Data sent to ThingSpeak!", end="")
        # else:
        #     print("\n❌ Failed to send data:", response.text, end="")

        time.sleep(0.5)

except KeyboardInterrupt:
    print("\rProgram interrupted by user")

finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    motor_running = False
    set_rgb_color(0, 0, 0)
    stop_motor()
    if motor_thread is not None and motor_thread.is_alive():
        motor_thread.join()


    print("\r---Summary ---")
    print(f"Operating Mode: {mode}")
    print(f"Motor was turned ON {motor_on_events} times")
    print(f"Motor was turned OFF {motor_off_events - 1} times")
    print(f"Total motor running time: {motor_total_on_time:.2f} seconds")
    print(f"Max Temperature: {highest_temperature:.2f}°C")
    print(f"Last Recorded Temperature: {temperature:.2f}°C")
    print(f"Last Recorded Lux: {lux:.2f}")
    print("Program cleaned up and exited")