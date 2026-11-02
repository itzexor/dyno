#!/usr/bin/env python3
"""Deterministic build ID and application-image boundary checks."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import zlib

ORIGIN = 0x08002000
LIMIT = 0x0800F800 - ORIGIN
INITIAL_SP = 0x20003FF8


def inspect(image):
    raw = Path(image).read_bytes()
    if not 192 <= len(raw) <= LIMIT:
        raise ValueError("image is empty or overlaps the settings page")
    vectors = struct.unpack_from("<48I", raw)
    if vectors[0] != INITIAL_SP:
        raise ValueError("unexpected application stack pointer")
    for value in vectors[1:]:
        if value and (not value & 1 or not ORIGIN <= (value & ~1) < ORIGIN + len(raw)):
            raise ValueError(f"invalid Thumb vector: {value:#x}")
    return dict(size=len(raw), origin=hex(ORIGIN), end_exclusive=hex(ORIGIN + len(raw)),
                sha256=hashlib.sha256(raw).hexdigest(), crc32=f"{zlib.crc32(raw):08x}",
                initial_sp=hex(vectors[0]), reset_vector=hex(vectors[1]))


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    build = sub.add_parser("build-id")
    build.add_argument("--source", required=True)
    build.add_argument("--output", type=Path, required=True)
    build.add_argument("files", nargs="+")
    check = sub.add_parser("inspect")
    check.add_argument("--source", required=True)
    check.add_argument("--image", required=True)
    check.add_argument("--output", required=True, type=Path)
    check.add_argument("--build-header", required=True, type=Path)
    args = parser.parse_args()
    if args.command == "build-id":
        digest = hashlib.sha256(args.source.encode())
        for name in sorted(args.files):
            digest.update(name.encode() + b"\0" + Path(name).read_bytes() + b"\0")
        args.output.write_text(f"#define DYNO_BUILD_ID 0x{digest.hexdigest()[:8]}u\n")
    else:
        report = inspect(args.image)
        report["source"] = args.source
        report["build_id"] = re.search(r"0x([0-9a-f]+)u", args.build_header.read_text())[1]
        args.output.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
