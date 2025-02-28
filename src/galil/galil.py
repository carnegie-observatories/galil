import socket
from datetime import datetime
import time
from typing import Tuple


class Galil(object):
    ip_address: str
    port: int
    axis: str = "A"
    brushless_init_voltage: int = 3
    acceleration: int = 256000
    deceleration: int = 256000
    speed: int = 250000
    # encoder calibration
    ring_length: float = 2992.36701058  # mm
    max_error: float = ring_length * 0.005  # mm (0.5% of the ring length)
    encoder_ratio: float = -25.4  # aux to main encoder ratio
    aux_counts_per_mm: float = 100000  # aux encoder counts per mm
    soft_limits: Tuple[int, int] = (3800000, 292000000)  # aux encoder counts
    zero_deg_position: float = 1672  # mm

    def __init__(self, ip_address: str, port: int = 23, auto_connect: bool = True) -> None:
        self.ip_address = ip_address
        self.port = port
        self._socket = None
        self.connected = False
        self._brushless_zero_initialized = False
        if auto_connect:
            self.connect()

    def connect(self) -> bool:
        if not self.connected:
            self.close()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.connect((self.ip_address, self.port))
        self.connected = True
        return self.connected

    def reset(self) -> bool:
        return self.send_command_check("RS")

    def close(self):
        if self._socket:
            self.motor_off()
            # self.reset()
            self._socket.close()
            self._socket = None
            self.connected = False

    def send_command(self, command: str) -> str:
        if not self.connected and not self.connect():
            raise Exception("Not connected to Galil controller")
        self._socket.send(command.encode("utf-8") + b"\r")
        output = self._socket.recv(1024).decode("utf-8")
        if output.startswith("?"):
            raise Exception(f"Error command '{command}': {output.strip()}")
        return output

    def send_command_check(self, command: str) -> str:
        return self.send_command(command).endswith(":")

    def brushless_zero_init(self) -> bool:
        if not self._brushless_zero_initialized:
            self._brushless_zero_initialized = self.send_command_check(
                f"BZ{self.axis}={self.brushless_init_voltage}"
            )
        return self._brushless_zero_initialized

    def motor_off(self) -> bool:
        return self.send_command_check(f"MO{self.axis}")

    def motor_on(self) -> bool:
        if self.brushless_zero_init():
            return self.send_command_check(f"SH{self.axis}")
        return False

    def set_speed(self) -> bool:
        return self.send_command_check(f"SP{self.axis}={self.speed}")

    def set_acceleration(self) -> bool:
        return self.send_command_check(f"AC{self.axis}={self.acceleration}")

    def set_deceleration(self) -> bool:
        return self.send_command_check(f"DC{self.axis}={self.deceleration}")

    def set_position(self, position: int) -> bool:
        return self.send_command_check(f"DP{self.axis}={position}")

    def init_absolute_distance(self, distance: int) -> bool:
        ret = True
        ret &= self.set_acceleration()
        ret &= self.set_deceleration()
        ret &= self.set_speed()
        ret &= self.set_position(0)
        ret &= self.send_command_check(f"PA{self.axis}={distance}")
        ret &= self.motor_on()
        return ret

    def is_moving(self) -> bool:
        # this method is not working
        # cmd = self.send_command(f"MG_BG{self.axis}")
        # return float(cmd.split("\r")[0].strip()) > 0
        p0 = self.get_position_aux()
        time.sleep(0.2)
        p1 = self.get_position_aux()
        return p1 != p0

    def begin(self) -> bool:
        return self.send_command_check(f"BG{self.axis}")

    def move_absolute_distance(self, distance: int) -> bool:
        if not self.init_absolute_distance(distance):
            return False
        ret = self.begin()
        while self.is_moving():
            pass
        ret &= self.motor_off()
        return ret

    def get_position_aux(self) -> int:
        return int(self.send_command(f"TD{self.axis}").split("\r")[0].strip())

    def get_position_main(self) -> int:
        return int(self.send_command(f"TP{self.axis}").split("\r")[0].strip())

    def jog(self, velocity: int) -> bool:
        ret = self.motor_on()
        ret &= self.send_command_check(f"JG{self.axis}={velocity}")
        ret &= self.begin()
        return ret

    def stop(self) -> bool:
        return self.send_command_check(f"ST{self.axis}")

    def _calibrate_position(self, step_size: int = -50000) -> bool:
        # NEG limit = 295043072
        #             250000000
        # POS limit = 2818048

        # 3500000
        #   50000
        fp = open(
            f"calibration_{step_size}_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.txt",
            "w",
        )
        while 2500000 < (aux_before := self.get_position_aux()) < 290000000:
            self.move_absolute_distance(step_size)
            main = self.get_position_main()
            aux_after = self.get_position_aux()
            print(f"{main}, {aux_before}, {aux_after}")
            fp.write(f"{main}, {aux_before}, {aux_after}\n")
            fp.flush()
        fp.close()
        return True

    def move_to_position(self, position: float) -> bool:
        """
        Move the motor to the specified position in mm on the aux encoder.
        Args:
            position (float): The target position in mm.
        """
        current_position = self.get_position_aux()
        current_position_mm = current_position / self.aux_counts_per_mm
        # Check if the position is already reached
        if abs(position - current_position_mm) < self.max_error:
            return True
        # Calculate the distance to move
        distance_aux = position * self.aux_counts_per_mm - current_position
        distance_main = distance_aux / self.encoder_ratio
        # Move the motor
        self.move_absolute_distance(int(distance_main))
        position_error = abs(
            self.get_position_aux() / self.aux_counts_per_mm - position
        )
        print(
            f"Distance aux: {distance_aux} Distance to move: {distance_main}, Position error: {position_error}"
        )
        return position_error < self.max_error

    def angle_to_position(self, angle: float) -> float:
        """
        Convert an angle in degrees to the corresponding position on the aux encoder.
        Args:
            angle (float): The angle in degrees.
        Returns:
            float: The corresponding position on the aux encoder in mm.
        """
        position = self.zero_deg_position + self.ring_length * angle / 360.0
        if abs(position) > self.soft_limits[1] / self.aux_counts_per_mm:
            raise Exception(
                f"Distance {position} exceeds soft limit {self.soft_limits[1] / self.aux_counts_per_mm}."
            )
        if abs(position) < self.soft_limits[0] / self.aux_counts_per_mm:
            raise Exception(
                f"Distance {position} is below soft limit {self.soft_limits[0] / self.aux_counts_per_mm}."
            )
        return position

    def move_to_angle(self, angle: float) -> bool:
        """
        Move the motor to the specified angle in degrees.
        Args:
            angle (float): The target angle in degrees.
        """
        position = self.angle_to_position(angle)
        return self.move_to_position(position)

    def move_to_angle_with_limit(self, angle: float, limit: float = 180.0) -> bool:
        """
        Move the motor to the specified angle in degrees with a limit check.
        Args:
            angle (float): The target angle in degrees.
            limit (float): The limit in degrees to check against.
        """
        if abs(angle) > limit:
            print(f"Angle {angle} exceeds limit {limit}.")
            return False
        return self.move_to_angle(angle)

    def _test_move_angles(self):
        for i in list(range(-8, 0)) + list(range(0, 7)):
            t0 = time.time()
            angle = i * 22.5
            success = self.move_to_angle(angle)
            if not success:
                print(f"Failed to move to angle: {angle}")
            else:
                elapsed_time = time.time() - t0
                print(
                    f"Time taken to move to angle {angle}: {elapsed_time:.2f} seconds"
                )
            time.sleep(1)
