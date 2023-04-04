import struct
from typing import Tuple

import numpy as np
from cobs import cobs
from serial import Serial


TEENSY_SERIAL_DEVICE = "/dev/ttyS0"
TEENSY_SERIAL_BAUD_RATE = 1000000
TEENSY_SERIAL_TX_START_BYTE = 0b11010110
TEENSY_SERIAL_TX_END_BYTE = 0b00110010


class TeensySerial:
    def __init__(self) -> None:
        self._serial = Serial(TEENSY_SERIAL_DEVICE, TEENSY_SERIAL_BAUD_RATE)

    def write_packet(
        self,
        ball: Tuple[float, float],
        blue_goal: Tuple[float, float],
        yellow_goal: Tuple[float, float],
    ) -> None:
        # Prepare data
        new_data = True
        ball_angle = (
            round(ball[0] * 100)  # -179(.)99º to 180(.)00º
            if ball[0]
            else np.iinfo(np.int16).max  # Flag for no ball
        )
        ball_distance = (
            round(ball[1] * 100)  # 0(.)00 cm to 400(.)00 cm
            if ball[1]
            else np.iinfo(np.uint16).max  # Flag for no ball
        )
        blue_goal_angle = (
            round(blue_goal[0] * 100)  # -179(.)99º to 180(.)00º
            if blue_goal[0]
            else np.iinfo(np.int16).max  # Flag for no goal
        )
        blue_goal_distance = (
            round(blue_goal[1] * 100)  # 0(.)00 cm to 400(.)00 cm
            if blue_goal[1]
            else np.iinfo(np.uint16).max  # Flag for no goal
        )
        yellow_goal_angle = (
            round(yellow_goal[0] * 100)  # -179(.)99º to 180(.)00º
            if yellow_goal[0]
            else np.iinfo(np.int16).max  # Flag for no ball
        )
        yellow_goal_distance = (
            round(yellow_goal[1] * 100)  # 0(.)00 cm to 400(.)00 cm
            if yellow_goal[1]
            else np.iinfo(np.uint16).max  # Flag for no ball
        )
        # Clamp to max of 400(.)00 cm
        ball_distance = min(ball_distance, 40000)
        blue_goal_distance = min(blue_goal_distance, 40000)
        yellow_goal_distance = min(yellow_goal_distance, 40000)

        # Pack data
        buf = struct.pack(
            "<HhHhHhH",
            new_data,  # H, unsigned short (bool is padded to 2 bytes in struct on Teensy)
            ball_angle,  # h, short
            ball_distance,  # H, unsigned short
            blue_goal_angle,  # h, short
            blue_goal_distance,  # H, unsigned short
            yellow_goal_angle,  # h, short
            yellow_goal_distance,  # H, unsigned short
        )

        # Encode with COBS
        buf = cobs.encode(buf)
        buf += b"\x00"  # delimiter byte

        # Send packet
        try:
            self._serial.write(buf)
        except (
            Exception
        ) as exc:  # sometimes it can't write when the coral is starting up
            print("Failed to write to serial: ", exc)
