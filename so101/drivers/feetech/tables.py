"""STS3215 control table and constants.

Ported from lerobot/motors/feetech/tables.py (Apache-2.0).
Stripped to STS3215 only — no SCS series, SM8512BL, etc.
"""

# data_name: (address, byte_length)
# Reference: http://doc.feetech.cn/#/prodinfodownload?srcType=FT-SMS-STS-emanual-229f4476422d4059abfb1cb0
STS3215_CONTROL_TABLE: dict[str, tuple[int, int]] = {
    # EPROM
    "Firmware_Major_Version": (0, 1),
    "Firmware_Minor_Version": (1, 1),
    "Model_Number": (3, 2),
    "ID": (5, 1),
    "Baud_Rate": (6, 1),
    "Return_Delay_Time": (7, 1),
    "Response_Status_Level": (8, 1),
    "Min_Position_Limit": (9, 2),
    "Max_Position_Limit": (11, 2),
    "Max_Temperature_Limit": (13, 1),
    "Max_Voltage_Limit": (14, 1),
    "Min_Voltage_Limit": (15, 1),
    "Max_Torque_Limit": (16, 2),
    "Phase": (18, 1),
    "Unloading_Condition": (19, 1),
    "LED_Alarm_Condition": (20, 1),
    "P_Coefficient": (21, 1),
    "D_Coefficient": (22, 1),
    "I_Coefficient": (23, 1),
    "Minimum_Startup_Force": (24, 2),
    "CW_Dead_Zone": (26, 1),
    "CCW_Dead_Zone": (27, 1),
    "Protection_Current": (28, 2),
    "Angular_Resolution": (30, 1),
    "Homing_Offset": (31, 2),
    "Operating_Mode": (33, 1),
    "Protective_Torque": (34, 1),
    "Protection_Time": (35, 1),
    "Overload_Torque": (36, 1),
    "Velocity_closed_loop_P_proportional_coefficient": (37, 1),
    "Over_Current_Protection_Time": (38, 1),
    "Velocity_closed_loop_I_integral_coefficient": (39, 1),
    # SRAM
    "Torque_Enable": (40, 1),
    "Acceleration": (41, 1),
    "Goal_Position": (42, 2),
    "Goal_Time": (44, 2),
    "Goal_Velocity": (46, 2),
    "Torque_Limit": (48, 2),
    "Lock": (55, 1),
    "Present_Position": (56, 2),
    "Present_Velocity": (58, 2),
    "Present_Load": (60, 2),
    "Present_Voltage": (62, 1),
    "Present_Temperature": (63, 1),
    "Status": (65, 1),
    "Moving": (66, 1),
    "Present_Current": (69, 2),
    "Goal_Position_2": (71, 2),
    # Factory
    "Moving_Velocity_Threshold": (80, 1),
    "DTs": (81, 1),
    "Velocity_Unit_factor": (82, 1),
    "Hts": (83, 1),
    "Maximum_Velocity_Limit": (84, 1),
    "Maximum_Acceleration": (85, 1),
    "Acceleration_Multiplier": (86, 1),
}

# Sign-magnitude encoding bits for registers that use it
STS3215_ENCODING_TABLE: dict[str, int] = {
    "Homing_Offset": 11,
    "Goal_Position": 15,
    "Goal_Velocity": 15,
    "Present_Position": 15,
    "Present_Velocity": 15,
}

STS3215_RESOLUTION = 4096
STS3215_MODEL_NUMBER = 777

BAUDRATE_TABLE: dict[int, int] = {
    1_000_000: 0,
    500_000: 1,
    250_000: 2,
    128_000: 3,
    115_200: 4,
    57_600: 5,
    38_400: 6,
    19_200: 7,
}

DEFAULT_BAUDRATE = 1_000_000
