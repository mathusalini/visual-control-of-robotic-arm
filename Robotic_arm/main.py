import serial
import time

class SSC32U:
    def __init__(self, port='COM7', baudrate=9600):
        """
        Initialize SSC-32U servo controller
        
        Args:
            port: Serial port (e.g., 'COM7' on Windows, '/dev/ttyUSB0' on Linux)
            baudrate: Communication speed (default 115200, can be 9600, 38400, etc.)
        """
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for connection to establish
        print(f"Connected to SSC-32U on {port} at {baudrate} baud")
    
    def move_servo(self, channel, pulse_width, speed=None, time_ms=None):
        """
        Move a servo to a specific position
        
        Args:
            channel: Servo channel (0-31)
            pulse_width: Pulse width in microseconds (500-2500, typically 1500 is center)
            speed: Optional speed in microseconds per second
            time_ms: Optional time to complete move in milliseconds
        """
        command = f"#{channel}P{pulse_width}"
        
        if speed is not None:
            command += f"S{speed}"
        
        if time_ms is not None:
            command += f"T{time_ms}"
        
        command += "\r"
        
        self.ser.write(command.encode())
        print(f"Sent: {command.strip()}")
    
    def move_multiple_servos(self, servos, time_ms=None):
        """
        Move multiple servos simultaneously
        
        Args:
            servos: List of tuples [(channel, pulse_width), ...]
            time_ms: Optional time to complete move in milliseconds
        """
        command = ""
        for channel, pulse_width in servos:
            command += f"#{channel}P{pulse_width}"
        
        if time_ms is not None:
            command += f"T{time_ms}"
        
        command += "\r"
        
        self.ser.write(command.encode())
        print(f"Sent: {command.strip()}")
    
    def set_servo_angle(self, channel, angle, min_pw=500, max_pw=2500, speed=None):
        """
        Move servo to a specific angle (0-180 degrees)
        
        Args:
            channel: Servo channel (0-31)
            angle: Angle in degrees (0-180)
            min_pw: Minimum pulse width (default 500μs for 0°)
            max_pw: Maximum pulse width (default 2500μs for 180°)
            speed: Optional speed
        """
        # Map angle (0-180) to pulse width
        pulse_width = int(min_pw + (max_pw - min_pw) * (angle / 180.0))
        self.move_servo(channel, pulse_width, speed)
    
    def stop_all(self):
        """Stop all servo movement"""
        command = "STOP\r"
        self.ser.write(command.encode())
        print("All servos stopped")
    
    def close(self):
        """Close the serial connection"""
        self.ser.close()
        print("Connection closed")  


# Example usage
if __name__ == "__main__":
    # Initialize controller (adjust COM port for your system)
    # Windows: 'COM3', 'COM4', etc.
    # Linux: '/dev/ttyUSB0', '/dev/ttyACM0', etc.
    # Mac: '/dev/tty.usbserial-XXXX'
    
    controller = SSC32U(port='COM7', baudrate=9600)
    
    try:
        # # Example 1: Move servo on channel 0 to center position (1500μs)
        # print("\n--- Moving to center ---")
        # controller.move_servo(channel=0, pulse_width=1500)
        # time.sleep(1)
        
        # # Example 2: Move servo to minimum position (500μs) with speed
        # print("\n--- Moving to minimum ---")
        # controller.move_servo(channel=0, pulse_width=500, speed=100)
        # time.sleep(2)
        
        # # Example 3: Move servo to maximum position (2500μs) in 2 seconds
        # print("\n--- Moving to maximum ---")
        # controller.move_servo(channel=0, pulse_width=2500, time_ms=2000)
        # time.sleep(2)
        
        # # Example 4: Move servo using angle (90 degrees)
        # print("\n--- Moving to 90 degrees ---")
        # controller.set_servo_angle(channel=0, angle=90)
        # time.sleep(1)
        
        # Example 5: Move multiple servos simultaneously
        print("\n--- Moving multiple servos ---")
        controller.move_multiple_servos([
            (0, 1500),  # Channel 0 to center
            (1, 1500),  # Channel 1 to 2000μs
            (2, 1000)   # Channel 2 to 1000μs
        ], time_ms=1500)
        time.sleep(2)
        
        # # Example 6: Sweep servo back and forth
        # print("\n--- Sweeping servo ---")
        # for angle in range(0, 30, 10):
        #     controller.set_servo_angle(channel=0, angle=angle)
        #     time.sleep(0.1)
        
    finally:
        controller.close()