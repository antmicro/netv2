#!/usr/bin/env python

yuv_422_col = {
    "green": [0x2b, 0x95, 0x15, 0x95],
    "cyan":  [0xab, 0xb2, 0x00, 0xb2],
}

with open("fb.bin", "wb") as f:
    for i in range(2 * 1280 * 720):
        for b in yuv_422_col["cyan"]:
            f.write(bytes([b]))
