#!/usr/bin/env python3
"""Generate tiny GLB fixtures and sibling STL files for glb_visual_prep tests."""

from __future__ import annotations

import json
import struct
import zlib
from pathlib import Path

OUT_DIR = Path(__file__).resolve().parent


def png_2x2() -> bytes:
    def chunk(tag: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + tag
            + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        )

    raw = b""
    for _y in range(2):
        raw += b"\x00"
        for _x in range(2):
            raw += b"\xff\x00\x00\xff"
    ihdr = struct.pack(">IIBBBBB", 2, 2, 8, 6, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", ihdr)
        + chunk(b"IDAT", zlib.compress(raw))
        + chunk(b"IEND", b"")
    )


def pack_f32(values: list[float]) -> bytes:
    return b"".join(struct.pack("<f", v) for v in values)


def pack_u16(values: list[int]) -> bytes:
    return b"".join(struct.pack("<H", v) for v in values)


def write_glb(path: Path, gltf: dict, bin_data: bytes) -> None:
    json_bytes = json.dumps(gltf, separators=(",", ":")).encode("utf-8")
    json_pad = (4 - len(json_bytes) % 4) % 4
    json_bytes += b" " * json_pad

    bin_pad = (4 - len(bin_data) % 4) % 4
    bin_data += b"\x00" * bin_pad

    total = 12 + 8 + len(json_bytes) + 8 + len(bin_data)
    header = struct.pack("<III", 0x46546C67, 2, total)
    json_chunk = struct.pack("<II", len(json_bytes), 0x4E4F534A) + json_bytes
    bin_chunk = struct.pack("<II", len(bin_data), 0x004E4942) + bin_data
    path.write_bytes(header + json_chunk + bin_chunk)


# Tetrahedron: 4 vertices, 4 faces (MuJoCo requires >= 4 mesh vertices).
TETRA_POSITIONS = [
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.5,
    1.0,
    0.0,
    0.5,
    0.5,
    1.0,
]
TETRA_INDICES = [0, 1, 2, 0, 2, 3, 0, 3, 1, 1, 3, 2]
TETRA_TEXCOORDS = [
    0.0,
    0.0,
    1.0,
    0.0,
    0.5,
    1.0,
    0.5,
    0.5,
]


def write_textured_tri(path: Path) -> None:
    positions = pack_f32(TETRA_POSITIONS)
    texcoords = pack_f32(TETRA_TEXCOORDS)
    indices = pack_u16(TETRA_INDICES)
    png = png_2x2()

    pos_len = len(positions)
    uv_len = len(texcoords)
    idx_len = len(indices)
    png_off = pos_len + uv_len + idx_len
    bin_data = positions + texcoords + indices + png

    gltf = {
        "asset": {"version": "2.0"},
        "buffers": [{"byteLength": len(bin_data)}],
        "bufferViews": [
            {"buffer": 0, "byteOffset": 0, "byteLength": pos_len},
            {"buffer": 0, "byteOffset": pos_len, "byteLength": uv_len},
            {"buffer": 0, "byteOffset": pos_len + uv_len, "byteLength": idx_len},
            {"buffer": 0, "byteOffset": png_off, "byteLength": len(png)},
        ],
        "accessors": [
            {"bufferView": 0, "componentType": 5126, "count": 4, "type": "VEC3"},
            {"bufferView": 1, "componentType": 5126, "count": 4, "type": "VEC2"},
            {"bufferView": 2, "componentType": 5123, "count": 12, "type": "SCALAR"},
        ],
        "images": [{"bufferView": 3, "mimeType": "image/png"}],
        "textures": [{"source": 0}],
        "materials": [{"pbrMetallicRoughness": {"baseColorTexture": {"index": 0}}}],
        "meshes": [
            {
                "primitives": [
                    {
                        "attributes": {"POSITION": 0, "TEXCOORD_0": 1},
                        "indices": 2,
                        "material": 0,
                    }
                ]
            }
        ],
    }
    write_glb(path, gltf, bin_data)


def write_factor_only(path: Path) -> None:
    positions = pack_f32(TETRA_POSITIONS)
    indices = pack_u16(TETRA_INDICES)
    bin_data = positions + indices

    pos_len = len(positions)
    gltf = {
        "asset": {"version": "2.0"},
        "buffers": [{"byteLength": len(bin_data)}],
        "bufferViews": [
            {"buffer": 0, "byteOffset": 0, "byteLength": pos_len},
            {"buffer": 0, "byteOffset": pos_len, "byteLength": len(indices)},
        ],
        "accessors": [
            {"bufferView": 0, "componentType": 5126, "count": 4, "type": "VEC3"},
            {"bufferView": 1, "componentType": 5123, "count": 12, "type": "SCALAR"},
        ],
        "materials": [
            {
                "pbrMetallicRoughness": {
                    "baseColorFactor": [0.2, 0.4, 0.6, 1.0],
                }
            }
        ],
        "meshes": [
            {
                "primitives": [
                    {
                        "attributes": {"POSITION": 0},
                        "indices": 1,
                        "material": 0,
                    }
                ]
            }
        ],
    }
    write_glb(path, gltf, bin_data)


def write_no_uv(path: Path) -> None:
    positions = pack_f32(TETRA_POSITIONS)
    indices = pack_u16(TETRA_INDICES)
    bin_data = positions + indices

    pos_len = len(positions)
    gltf = {
        "asset": {"version": "2.0"},
        "buffers": [{"byteLength": len(bin_data)}],
        "bufferViews": [
            {"buffer": 0, "byteOffset": 0, "byteLength": pos_len},
            {"buffer": 0, "byteOffset": pos_len, "byteLength": len(indices)},
        ],
        "accessors": [
            {"bufferView": 0, "componentType": 5126, "count": 4, "type": "VEC3"},
            {"bufferView": 1, "componentType": 5123, "count": 12, "type": "SCALAR"},
        ],
        "meshes": [
            {
                "primitives": [
                    {
                        "attributes": {"POSITION": 0},
                        "indices": 1,
                    }
                ]
            }
        ],
    }
    write_glb(path, gltf, bin_data)


def write_minimal_binary_stl(path: Path) -> None:
    header = bytes(80)
    faces = [
        (TETRA_POSITIONS[0:3], TETRA_POSITIONS[3:6], TETRA_POSITIONS[6:9]),
        (TETRA_POSITIONS[0:3], TETRA_POSITIONS[6:9], TETRA_POSITIONS[9:12]),
        (TETRA_POSITIONS[0:3], TETRA_POSITIONS[9:12], TETRA_POSITIONS[3:6]),
        (TETRA_POSITIONS[3:6], TETRA_POSITIONS[9:12], TETRA_POSITIONS[6:9]),
    ]
    count = struct.pack("<I", len(faces))
    body = b""
    for v0, v1, v2 in faces:
        normal = struct.pack("<fff", 0.0, 0.0, 1.0)
        verts = struct.pack(
            "<fffffffff",
            v0[0],
            v0[1],
            v0[2],
            v1[0],
            v1[1],
            v1[2],
            v2[0],
            v2[1],
            v2[2],
        )
        attr = struct.pack("<H", 0)
        body += normal + verts + attr
    path.write_bytes(header + count + body)


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    stems = ("textured_tri", "factor_only", "no_uv")
    write_textured_tri(OUT_DIR / "textured_tri.glb")
    write_factor_only(OUT_DIR / "factor_only.glb")
    write_no_uv(OUT_DIR / "no_uv.glb")
    for stem in stems:
        write_minimal_binary_stl(OUT_DIR / f"{stem}.STL")
    print(f"Wrote fixtures under {OUT_DIR}")


if __name__ == "__main__":
    main()
