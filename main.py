import network
import socket
import time
import machine
import sys
import _thread

ssid = 'Rocket WLAN'
password = 'safeflight123'
status = "Normal"


def create_access_point():
    global status
    status = "creating access point"
    wlan = network.WLAN(network.AP_IF)  # Use AP_IF for Access Point mode
    wlan.active(True)
    wlan.config(essid=ssid, password=password)  # Configure SSID and password

    # Wait until the access point is active
    while not wlan.active():
        print('Activating Access Point...')
        time.sleep(0.5)

    ip = wlan.ifconfig()[0]  # Get the Pico's IP address in AP mode
    print(f"Access Point created, connect to '{ssid}' with IP: {ip}")
    status = "access point created, connect to", ssid, "with IP", ip
    return wlan, ip


def open_socket(ip):
    global status
    status = opening
    socket
    # Open a socket
    address = (ip, 80)
    connection = socket.socket()
    connection.bind(address)
    connection.listen(1)
    print("Socket opened")
    return connection


def webpage(total_dx, total_dy, total_dz, theta1, theta2, theta3, x_g, y_g, status):
    html = f"""
<!DOCTYPE html>
<html>
    <title>Flight Control</title>
    <meta http-equiv="refresh" content="0.3">
        <body>
            <p>
                total_dx = {total_dx}
            </p>
            <p>
                total_dy = {total_dy}
            </p>
            <p>
                total_dz = {total_dz}
            </p>
            <p>
                servo north angle = {theta1}
            </p>
            <p>
                servo east angle = {theta2}
            </p>
            <p>
                servo south angle = {servo3}
            </p>
            <p>
                servo west angle = {servo4}
            </p>
            <p>
                x_g = {x_g}
            </p>
            <p>
                y_g = {y_g}
            </p>
            <p>
                current status = {status}
            </p>
            <form action = "/launch" method = "POST">
                <input type = "submit" value = "Initiate launch">
            </form>
            <form action = "/shutdown" method = "POST">
                <input type = "submit" value = "shutdown pico">
            </form>
        </body>
</html>
"""
    return html


def shutdown(wlan, connection):
    global status
    try:
        status = "microcontroller shutdown in progress"
        print("Disconnecting from network...")
        wlan.active(False)  # Disable WLAN interface
        print("WLAN disabled.")

        print("Closing socket connection...")
        connection.close()  # Close the socket connection
        print("Socket closed.")

        print("Resetting Pico...")
        sys.exit()  # Reset the device (Reboot)
    except Exception as e:
        status = "error during shutdown", e
        print(f"error during shutdown: {e}")


# Set up I2C
i2c = machine.I2C(0, scl=machine.Pin(13), sda=machine.Pin(12))  # SCL on GP13, SDA on GP12

# MPU-6050 I2C address
MPU6050_ADDR = 0x68  # MPU-6050 address (can be 0x68 or 0x69)

# Register addresses
PWR_MGMT_1 = 0x6B  # Power Management 1
ACCEL_XOUT_H = 0x3B  # Accelerometer X-axis high byte
ACCEL_XOUT_L = 0x3C  # Accelerometer X-axis low byte
ACCEL_YOUT_H = 0x3D  # Accelerometer Y-axis high byte
ACCEL_YOUT_L = 0x3E  # Accelerometer Y-axis low byte
ACCEL_ZOUT_H = 0x3F  # Accelerometer Z-axis high byte
ACCEL_ZOUT_L = 0x40  # Accelerometer Z-axis low byte

# Scale factor for ±2g range
ACCEL_SCALE_FACTOR = 16384.0  # LSB/g


# Function to write to a register
def write_register(reg, value):
    i2c.writeto_mem(MPU6050_ADDR, reg, bytes([value]))


# Function to read 2 bytes from a register
def read_registers(reg, length):
    return i2c.readfrom_mem(MPU6050_ADDR, reg, length)


# Initialize the MPU-6050
loop = True
while loop:
    try:
        status = "waking up MPU6050"
        write_register(PWR_MGMT_1, 0x00)  # Wake up the MPU-6050 (write 0 to the power management register)
        loop = False
        status = "MPU6050 wakeup complete"
    except Exception as e:
        status = "MPU 6050 wakeup failed, possible connection fault", e
        print(f"MPU 6050 not connected properly, {e}")


# Function to read accelerometer data
def read_accelerometer():
    # Read 6 bytes (X, Y, Z axes data)
    data = read_registers(ACCEL_XOUT_H, 6)

    # Combine the high and low bytes for each axis
    x = (data[0] << 8) | data[1]
    y = (data[2] << 8) | data[3]

    # Convert to signed 16-bit values (2's complement)
    if x >= 0x8000:
        x -= 0x10000
    if y >= 0x8000:
        y -= 0x10000

    # Convert raw values to g's
    x_g = x / ACCEL_SCALE_FACTOR
    y_g = y / ACCEL_SCALE_FACTOR

    return x_g, y_g


servo1_pin = machine.Pin(15)  # Replace with the correct GPIO pin
servo1 = machine.PWM(servo1_pin)
servo1.freq(50)  # Set frequency to 50 Hz


def set_servo1_angle(angle):
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180
    min_duty = 1638
    max_duty = 8192
    duty = int(min_duty + (angle / 180) * (max_duty - min_duty))
    servo1.duty_u16(duty)


servo2_pin = machine.Pin(9)  # Replace with the correct GPIO pin
servo2 = machine.PWM(servo2_pin)
servo2.freq(50)  # Set frequency to 50 Hz


def set_servo2_angle(angle):
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180
    min_duty = 1638
    max_duty = 8192
    duty = int(min_duty + (angle / 180) * (max_duty - min_duty))
    servo2.duty_u16(duty)


servo3_pin = machine.Pin(11)  # Replace with the correct GPIO pin
servo3 = machine.PWM(servo3_pin)
servo3.freq(50)  # Set frequency to 50 Hz


def set_servo3_angle(angle):
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180
    min_duty = 1638
    max_duty = 8192
    duty = int(min_duty + (angle / 180) * (max_duty - min_duty))
    servo3.duty_u16(duty)


servo4_pin = machine.Pin(10)  # Replace with the correct GPIO pin
servo4 = machine.PWM(servo4_pin)
servo4.freq(50)  # Set frequency to 50 Hz


def set_servo4_angle(angle):
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180
    min_duty = 1638
    max_duty = 8192
    duty = int(min_duty + (angle / 180) * (max_duty - min_duty))
    servo4.duty_u16(duty)


print("Servo test initiated")
for i in range(0, 181, 90):
    set_servo1_angle(i)
    set_servo2_angle(i)
    set_servo3_angle(i)
    set_servo4_angle(i)
    print(f"testing angle {i} degrees")
    time.sleep(1)
print("Servo test finished")

# MPU6050 Register addresses
GYRO_XOUT_H = 0x43
GYRO_SCALE = 131  # Scale factor for 250°/s gyroscope range


def read_word(register):
    # Read two bytes and combine them to make a word (16-bit)
    high = i2c.readfrom_mem(MPU6050_ADDR, register, 1)
    low = i2c.readfrom_mem(MPU6050_ADDR, register + 1, 1)
    value = (high[0] << 8) + low[0]

    # Two's complement conversion for negative values
    if value >= 0x8000:  # Check if value is negative
        value -= 0x10000  # Convert to negative value using two's complement
    return value


def read_gyro_data():
    # Read gyroscope data
    gyro_x = read_word(GYRO_XOUT_H)
    gyro_y = read_word(GYRO_XOUT_H + 2)
    gyro_z = read_word(GYRO_XOUT_H + 4)

    # Normalize gyroscope data (scaled to degrees per second)
    gyro_x = gyro_x / GYRO_SCALE
    gyro_y = gyro_y / GYRO_SCALE
    gyro_z = gyro_z / GYRO_SCALE

    return gyro_x, gyro_y, gyro_z


# Collect multiple readings and calculate the bias (zero offset)
def get_bias_gyro_data(num_samples=100):
    gyro_x_total = 0
    gyro_y_total = 0
    gyro_z_total = 0

    for _ in range(num_samples):
        gyro_x, gyro_y, gyro_z = read_gyro_data()
        gyro_x_total += gyro_x
        gyro_y_total += gyro_y
        gyro_z_total += gyro_z
        # time.sleep(0.01)  # Small delay to avoid overloading the sensor with too many reads

    # Calculate the average (bias)
    bias_x = gyro_x_total / num_samples
    bias_y = gyro_y_total / num_samples
    bias_z = gyro_z_total / num_samples

    return bias_x, bias_y, bias_z


# Subtract the bias (zero offset) from the readings to adjust the gyroscope data
def adjust_gyro_data(gyro_x, gyro_y, gyro_z, bias_x, bias_y, bias_z):
    gyro_x_adjusted = gyro_x - bias_x
    gyro_y_adjusted = gyro_y - bias_y
    gyro_z_adjusted = gyro_z - bias_z
    return gyro_x_adjusted, gyro_y_adjusted, gyro_z_adjusted


# Get the initial bias values by averaging the first set of readings
bias_x, bias_y, bias_z = get_bias_gyro_data(num_samples=100)

total_dx = 0
total_dy = 0
total_dz = 0
prev_gyro_x, prev_gyro_y, prev_gyro_z = read_gyro_data()

start_time = time.ticks_ms()
activation_angle = 0

theta1 = 90
theta2 = 90
theta3 = 90
theta4 = 90

try:
    wlan, ip = create_access_point()
    connection = open_socket(ip)
    print("network opening complete")
except Exception as e:
    print(f"error whilst opening the network: {e}")
    status = "error whilst opening the network", e

total_dx = 0
total_dy = 0
total_dz = 0
x_g = 0
y_g = 0


def flight_adjustments(prev_gyro_x, prev_gyro_y, prev_gyro_z, start_time):
    while True:
        try:
            global total_dx
            global total_dy
            global total_dz
            global theta1
            global theta2
            global theta3
            global theta4
            global x_g
            global y_g
            global status
            # Get gyroscope data (from the current reading)
            # Calculate dt if the gyro readings change
            gyro_x, gyro_y, gyro_z = read_gyro_data()
            # The below if statement is used instead of != to reject negletible changes. Otherwise they will rarely be identical.
            if abs(gyro_x - prev_gyro_x) >= 1 or abs(gyro_y - prev_gyro_y) >= 1 or abs(gyro_z - prev_gyro_z) >= 1:
                end_time = time.ticks_ms()
                prev_gyro_x = gyro_x
                prev_gyro_y = gyro_y
                prev_gyro_z = gyro_z
                dt = (end_time - start_time) / 1000
                # print(dt)
                start_time = time.ticks_ms()

                # Adjust the gyroscope data by subtracting the bias
                adjusted_gyro_x, adjusted_gyro_y, adjusted_gyro_z = adjust_gyro_data(gyro_x, gyro_y, gyro_z, bias_x,
                                                                                     bias_y, bias_z)

                # Additional filter for noise/drift
                if abs(adjusted_gyro_x) < 0.5:
                    adjusted_gyro_x = 0
                if abs(adjusted_gyro_y) < 0.5:
                    adjusted_gyro_y = 0
                if abs(adjusted_gyro_z) < 0.5:
                    adjusted_gyro_z = 0

                # Calculating angle by doing angular velocity * time.
                dx = adjusted_gyro_x * dt
                dy = adjusted_gyro_y * dt
                dz = adjusted_gyro_z * dt
                if abs(dz) < 0.1:
                    dz = 0

                # Using accelerometer to mitigate gyroscope drift.
                x_g, y_g = read_accelerometer()

                if abs(x_g) < 0.2:
                    total_dy = 0
                else:
                    total_dy += dy

                if abs(y_g) < 0.2:
                    total_dx = 0
                else:
                    total_dx += dx

                total_dz += dz
                # print(f"total dx: {total_dx}, total dy: {total_dy}, total dz: {total_dz}")

                if total_dz > 20:
                    theta1 = 90 + int(total_dz)
                    theta2 = 90 + int(total_dz)
                    theta3 = 90 + int(total_dz)
                    theta4 = 90 + int(total_dz)
                    if theta1 > 170:
                        theta1 = 170
                    if theta2 > 170:
                        theta2 = 170
                    if theta3 > 170:
                        theta3 = 170
                    if theta4 > 170:
                        theta4 = 170
                    set_servo1_angle(theta1)
                    set_servo2_angle(theta2)
                    set_servo3_angle(theta3)
                    set_servo4_angle(theta4)

                elif total_dz < -20:
                    theta1 = 90 + int(total_dz)
                    theta2 = 90 + int(total_dz)
                    theta3 = 90 + int(total_dz)
                    theta4 = 90 + int(total_dz)
                    if theta1 < 10:
                        theta1 = 10
                    if theta2 < 10:
                        theta2 = 10
                    if theta3 < 10:
                        theta3 = 10
                    if theta4 < 10:
                        theta4 = 10
                    set_servo1_angle(theta1)
                    set_servo2_angle(theta2)
                    set_servo3_angle(theta3)
                    set_servo4_angle(theta4)

                else:
                    if total_dx > activation_angle:
                        theta4 = 90 + int(total_dx)
                        theta2 = 90 - int(total_dx)
                        if theta4 > 170:
                            theta4 = 170
                        if theta2 < 10:
                            theta2 = 10
                        set_servo4_angle(theta4)
                        set_servo2_angle(theta2)

                    elif total_dx < -activation_angle:
                        theta4 = 90 + int(total_dx)
                        theta2 = 90 - int(total_dx)
                        if theta4 < 10:
                            theta4 = 10
                        if theta2 > 170:
                            theta2 = 170
                        set_servo4_angle(theta4)
                        set_servo2_angle(theta2)

                    else:
                        set_servo4_angle(90)
                        set_servo2_angle(90)

                    if total_dy > activation_angle:
                        theta3 = 90 + int(total_dy)
                        theta1 = 90 - int(total_dy)
                        if theta3 > 170:
                            theta3 = 170
                        if theta1 < 10:
                            theta1 = 10
                        set_servo3_angle(theta3)
                        set_servo1_angle(theta1)

                    elif total_dy < -activation_angle:
                        theta3 = 90 + int(total_dy)
                        theta1 = 90 - int(total_dy)
                        if theta3 < 10:
                            theta3 = 10
                        if theta1 > 170:
                            theta1 = 170
                        set_servo3_angle(theta3)
                        set_servo1_angle(theta1)

                    else:
                        set_servo3_angle(90)
                        set_servo1_angle(90)

        except Exception as e:
            print(f"flight adjustment error: {e}")
            status = "flight adjustment error", e


def website_hosting(connection, wlan):
    while True:
        try:
            global total_dx
            global total_dy
            global total_dz
            global theta1
            global theta2
            global theta3
            global theta4
            global x_g
            global y_g
            global status

            # start_time = time.time()
            client, addr = connection.accept()
            print(f"Connection from {addr}")
            request = client.recv(1024)
            request = str(request)
            print(request)
            if "/shutdown" in request:
                print("shutdown request recieved")
                shutdown(wlan, connection)
            html = webpage(total_dx, total_dy, total_dz, theta1, theta2, theta3, x_g, y_g, status)
            client.send(html)
            client.close()
            # end_time = time.time()
            # elapsed_time = end_time - start_time
            # print(f"time elapsed for networking was {elapsed_time} ms")

        except Exception as e:
            print(f"website hosting error: {e}")
            status = "website hosting error", e


_thread.start_new_thread(flight_adjustments, (prev_gyro_x, prev_gyro_y, prev_gyro_z, start_time))
website_hosting(connection, wlan)


