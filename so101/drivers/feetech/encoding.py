"""Sign-magnitude encoding/decoding for Feetech STS3215 registers.

Ported from lerobot/motors/encoding_utils.py (Apache-2.0).
"""


def encode_sign_magnitude(value: int, sign_bit_index: int) -> int:
    """Encode a signed integer into sign-magnitude representation.

    https://en.wikipedia.org/wiki/Signed_number_representations#Sign%E2%80%93magnitude
    """
    max_magnitude = (1 << sign_bit_index) - 1
    magnitude = abs(value)
    if magnitude > max_magnitude:
        raise ValueError(
            f"Magnitude {magnitude} exceeds {max_magnitude} (max for {sign_bit_index=})"
        )
    direction_bit = 1 if value < 0 else 0
    return (direction_bit << sign_bit_index) | magnitude


def decode_sign_magnitude(encoded_value: int, sign_bit_index: int) -> int:
    """Decode a sign-magnitude encoded value back to a signed integer.

    https://en.wikipedia.org/wiki/Signed_number_representations#Sign%E2%80%93magnitude
    """
    direction_bit = (encoded_value >> sign_bit_index) & 1
    magnitude_mask = (1 << sign_bit_index) - 1
    magnitude = encoded_value & magnitude_mask
    return -magnitude if direction_bit else magnitude
