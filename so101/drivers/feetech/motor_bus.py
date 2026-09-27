"""Standalone STS3215 motor bus — direct communication via scservo_sdk.

Ported from lerobot/motors/feetech/feetech.py + motors_bus.py (Apache-2.0).
Simplified for STS3215 only, protocol 0, no normalization layer.
"""

from __future__ import annotations

import logging

from .encoding import decode_sign_magnitude, encode_sign_magnitude
from .tables import (
    DEFAULT_BAUDRATE,
    STS3215_CONTROL_TABLE,
    STS3215_ENCODING_TABLE,
    STS3215_MODEL_NUMBER,
)

logger = logging.getLogger(__name__)

_PROTOCOL_VERSION = 0
_TIMEOUT_MS = 1000


def _patch_set_packet_timeout(self: object, packet_length: int) -> None:
    """Monkey-patch for scservo_sdk PortHandler timeout bug.

    Fixes https://gitee.com/ftservo/SCServoSDK/issues/IBY2S6
    The official Feetech SDK repo has the fix but the PyPI package does not.
    """
    self.packet_start_time = self.getCurrentTime()  # type: ignore[attr-defined]
    self.packet_timeout = (
        (  # type: ignore[attr-defined]
            self.tx_time_per_byte * packet_length  # type: ignore[attr-defined]
        )
        + (self.tx_time_per_byte * 3.0)
        + 50
    )  # type: ignore[attr-defined]


def _split_into_bytes(value: int, length: int) -> list[int]:
    """Little-endian byte split using scservo_sdk helpers."""
    import scservo_sdk as scs

    if length == 1:
        return [value]
    elif length == 2:
        return [scs.SCS_LOBYTE(value), scs.SCS_HIBYTE(value)]
    elif length == 4:
        return [
            scs.SCS_LOBYTE(scs.SCS_LOWORD(value)),
            scs.SCS_HIBYTE(scs.SCS_LOWORD(value)),
            scs.SCS_LOBYTE(scs.SCS_HIWORD(value)),
            scs.SCS_HIBYTE(scs.SCS_HIWORD(value)),
        ]
    raise ValueError(f"Unsupported byte length: {length}")


def _get_register(name: str) -> tuple[int, int]:
    """Look up (address, byte_length) for a register name."""
    entry = STS3215_CONTROL_TABLE.get(name)
    if entry is None:
        raise KeyError(f"Unknown register '{name}' — not in STS3215 control table.")
    return entry


class FeetechMotorsBus:
    """Direct STS3215 bus communication via scservo_sdk.

    No normalization, no calibration logic — returns raw encoder values.
    The driver layer (FeetechDriver) handles radians and calibration.
    """

    def __init__(self, port: str, motors: dict[str, int]) -> None:
        """
        Args:
            port: Serial port path (e.g. "/dev/ttyACM0").
            motors: Mapping of motor name to servo ID.
                    Example: {"shoulder_pan": 1, "shoulder_lift": 2, ...}
        """
        self._port = port
        self._motors = motors  # name -> id
        self._id_to_name = {id_: name for name, id_ in motors.items()}
        self._connected = False

        # SDK objects — initialized lazily in connect()
        self._port_handler: object = None
        self._packet_handler: object = None
        self._sync_reader: object = None
        self._sync_writer: object = None
        self._comm_success: int = 0

    @property
    def port(self) -> str:
        return self._port

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self, handshake: bool = True) -> None:
        """Open the serial port and optionally verify all motors respond."""
        if self._connected:
            raise RuntimeError(f"Already connected to '{self._port}'")

        import scservo_sdk as scs

        self._port_handler = scs.PortHandler(self._port)
        # Apply timeout monkey-patch
        self._port_handler.setPacketTimeout = _patch_set_packet_timeout.__get__(
            self._port_handler, type(self._port_handler)
        )
        self._packet_handler = scs.PacketHandler(_PROTOCOL_VERSION)
        self._sync_reader = scs.GroupSyncRead(self._port_handler, self._packet_handler, 0, 0)
        self._sync_writer = scs.GroupSyncWrite(self._port_handler, self._packet_handler, 0, 0)
        self._comm_success = scs.COMM_SUCCESS

        if not self._port_handler.openPort():
            raise ConnectionError(f"Failed to open port '{self._port}'")

        self._port_handler.setBaudRate(DEFAULT_BAUDRATE)
        self._port_handler.setPacketTimeoutMillis(_TIMEOUT_MS)
        self._connected = True

        if handshake:
            self._handshake()

        logger.info("FeetechMotorsBus connected on %s", self._port)

    def _handshake(self) -> None:
        """Ping each motor and verify model numbers."""
        expected = {id_: STS3215_MODEL_NUMBER for id_ in self._motors.values()}
        found: dict[int, int] = {}
        missing: list[int] = []

        for name, id_ in self._motors.items():
            model_nb, comm, error = self._packet_handler.ping(self._port_handler, id_)
            if comm == self._comm_success and error == 0:
                found[id_] = model_nb
            else:
                missing.append(id_)

        wrong = {id_: (expected[id_], found[id_]) for id_ in found if found[id_] != expected[id_]}

        if missing or wrong:
            lines = [f"Motor check failed on port '{self._port}':"]
            if missing:
                lines.append("\nMissing motor IDs:")
                for id_ in missing:
                    lines.append(f"  - {id_} ({self._id_to_name.get(id_, '?')})")
            if wrong:
                lines.append("\nWrong model numbers:")
                for id_, (exp, got) in wrong.items():
                    lines.append(
                        f"  - {id_} ({self._id_to_name.get(id_, '?')}): expected {exp}, found {got}"
                    )
            raise RuntimeError("\n".join(lines))

    def disconnect(self, disable_torque: bool = True) -> None:
        """Close the serial port, optionally disabling torque first."""
        if not self._connected:
            return

        if disable_torque:
            try:
                self._port_handler.clearPort()
                self._port_handler.is_using = False
                self.disable_torque()
            except Exception:
                logger.warning("Error disabling torque during disconnect", exc_info=True)

        self._port_handler.closePort()
        self._connected = False
        logger.info("FeetechMotorsBus disconnected from %s", self._port)

    # --- Single motor read/write (confirmed response) ---

    def read(self, register: str, motor: str) -> int:
        """Read a single register from one motor. Returns raw value."""
        self._assert_connected()
        id_ = self._motors[motor]
        addr, length = _get_register(register)

        read_fn = self._get_read_fn(length)
        value, comm, error = read_fn(self._port_handler, id_, addr)

        if comm != self._comm_success:
            raise ConnectionError(
                f"Failed to read '{register}' from motor '{motor}' (id={id_}): "
                + self._packet_handler.getTxRxResult(comm)
            )
        if error != 0:
            raise RuntimeError(
                f"Error reading '{register}' from motor '{motor}' (id={id_}): "
                + self._packet_handler.getRxPacketError(error)
            )

        # Decode sign-magnitude if applicable
        if register in STS3215_ENCODING_TABLE:
            value = decode_sign_magnitude(value, STS3215_ENCODING_TABLE[register])

        return value

    def write(self, register: str, motor: str, value: int) -> None:
        """Write a single register on one motor. Waits for response."""
        self._assert_connected()
        id_ = self._motors[motor]
        addr, length = _get_register(register)

        # Encode sign-magnitude if applicable
        if register in STS3215_ENCODING_TABLE:
            value = encode_sign_magnitude(value, STS3215_ENCODING_TABLE[register])

        data = _split_into_bytes(value, length)
        comm, error = self._packet_handler.writeTxRx(self._port_handler, id_, addr, length, data)

        if comm != self._comm_success:
            raise ConnectionError(
                f"Failed to write '{register}' to motor '{motor}' (id={id_}): "
                + self._packet_handler.getTxRxResult(comm)
            )
        if error != 0:
            raise RuntimeError(
                f"Error writing '{register}' to motor '{motor}' (id={id_}): "
                + self._packet_handler.getRxPacketError(error)
            )

    # --- Batch read/write (fast, for control loop) ---

    def sync_read(self, register: str) -> dict[str, int]:
        """Read a register from all motors simultaneously. Returns name -> raw value."""
        self._assert_connected()
        addr, length = _get_register(register)
        ids = list(self._motors.values())

        # Setup sync reader
        self._sync_reader.clearParam()
        self._sync_reader.start_address = addr
        self._sync_reader.data_length = length
        for id_ in ids:
            self._sync_reader.addParam(id_)

        comm = self._sync_reader.txRxPacket()
        if comm != self._comm_success:
            raise ConnectionError(
                f"Sync read '{register}' failed: " + self._packet_handler.getTxRxResult(comm)
            )

        result: dict[str, int] = {}
        for name, id_ in self._motors.items():
            value = self._sync_reader.getData(id_, addr, length)
            # Decode sign-magnitude if applicable
            if register in STS3215_ENCODING_TABLE:
                value = decode_sign_magnitude(value, STS3215_ENCODING_TABLE[register])
            result[name] = value

        return result

    def sync_write(self, register: str, values: dict[str, int]) -> None:
        """Write a register to multiple motors simultaneously."""
        self._assert_connected()
        addr, length = _get_register(register)

        self._sync_writer.clearParam()
        self._sync_writer.start_address = addr
        self._sync_writer.data_length = length

        for name, value in values.items():
            id_ = self._motors[name]
            # Encode sign-magnitude if applicable
            if register in STS3215_ENCODING_TABLE:
                value = encode_sign_magnitude(value, STS3215_ENCODING_TABLE[register])
            data = _split_into_bytes(value, length)
            self._sync_writer.addParam(id_, data)

        comm = self._sync_writer.txPacket()
        if comm != self._comm_success:
            raise ConnectionError(
                f"Sync write '{register}' failed: " + self._packet_handler.getTxRxResult(comm)
            )

    # --- Torque control ---

    def enable_torque(self, motors: list[str] | None = None) -> None:
        """Enable torque (and lock EEPROM) on specified motors, or all."""
        for name in motors or list(self._motors):
            self.write("Torque_Enable", name, 1)
            self.write("Lock", name, 1)

    def disable_torque(self, motors: list[str] | None = None) -> None:
        """Disable torque (and unlock EEPROM) on specified motors, or all."""
        for name in motors or list(self._motors):
            self.write("Torque_Enable", name, 0)
            self.write("Lock", name, 0)

    # --- Internal helpers ---

    def _assert_connected(self) -> None:
        if not self._connected:
            raise RuntimeError(f"Not connected to '{self._port}'")

    def _get_read_fn(self, length: int):
        read_fns = {
            1: self._packet_handler.read1ByteTxRx,
            2: self._packet_handler.read2ByteTxRx,
            4: self._packet_handler.read4ByteTxRx,
        }
        if length not in read_fns:
            raise ValueError(f"Unsupported read length: {length}")
        return read_fns[length]
