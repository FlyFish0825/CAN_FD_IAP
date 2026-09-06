#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Convert an APP .bin file into Bootloader transfer frames.

Two modes are supported:

1) classic
   For the current CANPro that only supports Classic CAN.
   Each 64-byte logical Bootloader DATA packet is split into 8 Classic CAN frames:
       0x100 ~ 0x107, 8 bytes each.

2) fd
   Native CAN FD mode.
   Each 56-byte firmware chunk becomes ONE 64-byte CAN FD frame:
       ID = 0x100, DLC = 64, BRS = ON.

The Bootloader logical DATA packet is identical in both modes:
    Byte0       Target Node
    Byte1       WRITE_DATA = 0x01
    Byte2~3     Sequence (uint16 LE)
    Byte4~7     Reserved = 0
    Byte8~63    56-byte firmware payload

The last firmware chunk is padded with 0xFF.
"""

from __future__ import annotations

import argparse
from pathlib import Path

WRITE_DATA_CMD = 0x01
DATA_BYTES_PER_PACKET = 56
LOGICAL_PACKET_SIZE = 64
FRAGMENT_SIZE = 8
CAN_DATA_BASE_ID = 0x100


def crc8_atm(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def crc32_mpeg2_bytes(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for value in data:
        crc ^= value << 24
        for _ in range(8):
            crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF if (crc & 0x80000000) else (crc << 1) & 0xFFFFFFFF
    return crc


def control_frame(target: int, cmd: int, byte2: int = 0, param4: bytes = b"\x00\x00\x00\x00") -> bytes:
    raw7 = bytes([target & 0xFF, cmd & 0xFF, byte2 & 0xFF]) + param4
    return raw7 + bytes([crc8_atm(raw7)])


def build_data_packet(target: int, sequence: int, firmware_chunk: bytes) -> bytes:
    if len(firmware_chunk) > DATA_BYTES_PER_PACKET:
        raise ValueError("firmware chunk too large")

    payload = firmware_chunk.ljust(DATA_BYTES_PER_PACKET, b"\xFF")

    packet = (
        bytes([target & 0xFF, WRITE_DATA_CMD])
        + int(sequence).to_bytes(2, "little")
        + bytes(4)
        + payload
    )

    if len(packet) != LOGICAL_PACKET_SIZE:
        raise RuntimeError("internal packet size error")

    return packet


# ---------------- Classic CANPro XML ----------------

def canpro_classic_obj(can_id: int, payload: bytes) -> str:
    """
    Exact Classic CANPro obj format observed from user's SendList:
      4B CAN ID little-endian
      8B zero metadata
      1B DLC
      8B payload
      3B zero tail
    """
    if len(payload) != 8:
        raise ValueError("Classic CAN payload must be 8 bytes")

    raw = (
        int(can_id).to_bytes(4, "little")
        + bytes(8)
        + bytes([8])
        + payload
        + bytes(3)
    )
    return raw.hex().upper()


def classic_tag(can_id: int, payload: bytes, interval_ms: int) -> str:
    return (
        f'    <tagSendUint iInterval="{interval_ms}" iTimes="1" len="1" '
        f'bIncreaseID="0" bIncreaseData="0" '
        f'obj="{canpro_classic_obj(can_id, payload)}" />'
    )


def emit_classic_canpro(fw: bytes, target: int, interval_ms: int, out: Path) -> None:
    packet_count = (len(fw) + DATA_BYTES_PER_PACKET - 1) // DATA_BYTES_PER_PACKET

    lines = ['<SendList m_dwCycles="1">']

    for seq in range(packet_count):
        chunk = fw[seq * DATA_BYTES_PER_PACKET:(seq + 1) * DATA_BYTES_PER_PACKET]
        logical = build_data_packet(target, seq, chunk)

        for frag in range(8):
            can_id = CAN_DATA_BASE_ID + frag
            payload = logical[frag * FRAGMENT_SIZE:(frag + 1) * FRAGMENT_SIZE]
            lines.append(classic_tag(can_id, payload, interval_ms))

    lines.append("</SendList>")
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")


# ---------------- Native CAN FD generic list ----------------

def emit_fd_text(fw: bytes, target: int, out: Path) -> None:
    """
    Generic text list because the exact CANPro FD XML encoding has not been
    established from a real CANPro FD SendList sample.

    Format:
        ID=100 FD=1 BRS=1 DLC=64 DATA=<128 hex chars>
    """
    packet_count = (len(fw) + DATA_BYTES_PER_PACKET - 1) // DATA_BYTES_PER_PACKET
    lines = []

    for seq in range(packet_count):
        chunk = fw[seq * DATA_BYTES_PER_PACKET:(seq + 1) * DATA_BYTES_PER_PACKET]
        logical = build_data_packet(target, seq, chunk)

        lines.append(
            f"ID=0x100 FD=1 BRS=1 DLC=64 SEQ={seq} DATA={logical.hex(' ').upper()}"
        )

    out.write_text("\n".join(lines) + "\n", encoding="utf-8")


def print_control_frames(fw: bytes, target: int) -> None:
    size = len(fw)
    fw_crc = crc32_mpeg2_bytes(fw)

    erase = control_frame(target, 0x10)
    write = control_frame(target, 0x11, 0x00, size.to_bytes(4, "little"))
    write_end = control_frame(target, 0x14)
    verify = control_frame(target, 0x13, 0x00, fw_crc.to_bytes(4, "little"))

    packet_count = (size + DATA_BYTES_PER_PACKET - 1) // DATA_BYTES_PER_PACKET

    print(f"Firmware size : {size} bytes")
    print(f"Packet count  : {packet_count}")
    print(f"APP CRC32     : 0x{fw_crc:08X}")
    print()
    print("Control frames (CAN ID 0x000, Classic CAN, DLC=8):")
    print("ERASE     :", erase.hex(" ").upper())
    print("WRITE     :", write.hex(" ").upper())
    print("WRITE_END :", write_end.hex(" ").upper())
    print("VERIFY    :", verify.hex(" ").upper())


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("bin", type=Path, help="APP .bin file")
    ap.add_argument("--mode", choices=["classic", "fd", "both"], default="both")
    ap.add_argument("--target", type=lambda x: int(x, 0), default=1)
    ap.add_argument("--interval", type=int, default=0.1, help="Classic fragment interval in ms")
    ap.add_argument("-o", "--out-prefix", type=Path, default=None)

    args = ap.parse_args()

    fw = args.bin.read_bytes()

    if not fw:
        raise SystemExit("BIN is empty")

    if len(fw) > 106 * 1024:
        raise SystemExit("BIN exceeds current APP region (106 KiB)")

    prefix = args.out_prefix or args.bin.with_suffix("")

    print_control_frames(fw, args.target)
    print()

    if args.mode in ("classic", "both"):
        classic_out = Path(str(prefix) + ".classic.canpro.list")
        emit_classic_canpro(fw, args.target, args.interval, classic_out)
        print("Classic CANPro file :", classic_out)

    if args.mode in ("fd", "both"):
        fd_out = Path(str(prefix) + ".canfd.txt")
        emit_fd_text(fw, args.target, fd_out)
        print("CAN FD frame list   :", fd_out)

    print()
    print("Native CAN FD DATA format:")
    print("  ID  = 0x100")
    print("  FD  = 1")
    print("  BRS = 1")
    print("  DLC = 64")
    print("  Byte0    = Target")
    print("  Byte1    = WRITE_DATA (0x01)")
    print("  Byte2~3  = Sequence (uint16 LE)")
    print("  Byte4~7  = Reserved")
    print("  Byte8~63 = 56-byte firmware payload")
    print()
    print("Classic fallback DATA format:")
    print("  0x100 = logical bytes  0..7")
    print("  0x101 = logical bytes  8..15")
    print("  ...")
    print("  0x107 = logical bytes 56..63")


if __name__ == "__main__":
    main()
