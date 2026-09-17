#!/usr/bin/env python3
"""Generate the city/industrial occupancy map used by Nav2 map_server."""

from math import ceil, floor
from pathlib import Path


RESOLUTION = 0.05
ORIGIN_X = -7.2
ORIGIN_Y = -5.2
WIDTH = 288
HEIGHT = 208


def world_to_pixel(x, y):
    col = int(floor((x - ORIGIN_X) / RESOLUTION))
    row_from_bottom = int(floor((y - ORIGIN_Y) / RESOLUTION))
    return col, HEIGHT - 1 - row_from_bottom


def fill_box(image, cx, cy, sx, sy):
    x0 = int(floor((cx - sx / 2.0 - ORIGIN_X) / RESOLUTION))
    x1 = int(ceil((cx + sx / 2.0 - ORIGIN_X) / RESOLUTION))
    y0 = int(floor((cy - sy / 2.0 - ORIGIN_Y) / RESOLUTION))
    y1 = int(ceil((cy + sy / 2.0 - ORIGIN_Y) / RESOLUTION))
    for bottom_row in range(max(0, y0), min(HEIGHT, y1 + 1)):
        row = HEIGHT - 1 - bottom_row
        start = max(0, x0)
        end = min(WIDTH - 1, x1)
        image[row][start:end + 1] = bytes([0]) * (end - start + 1)


def fill_circle(image, cx, cy, radius):
    center_col, center_row = world_to_pixel(cx, cy)
    radius_px = int(ceil(radius / RESOLUTION))
    radius_sq = radius_px * radius_px
    for row in range(max(0, center_row - radius_px), min(HEIGHT, center_row + radius_px + 1)):
        dy = row - center_row
        for col in range(max(0, center_col - radius_px), min(WIDTH, center_col + radius_px + 1)):
            dx = col - center_col
            if dx * dx + dy * dy <= radius_sq:
                image[row][col] = 0


def main():
    image = [bytearray([254]) * WIDTH for _ in range(HEIGHT)]

    boxes = [
        (0.0, 5.0, 14.0, 0.14), (0.0, -5.0, 14.0, 0.14),
        (7.0, 0.0, 0.14, 10.0), (-7.0, 0.0, 0.14, 10.0),
        (-3.55, -1.78, 2.75, 1.65), (-3.70, 1.75, 2.45, 1.70),
        (1.45, -1.72, 3.00, 1.75), (4.25, -1.72, 1.35, 1.75),
        (-5.55, -1.35, 0.65, 0.75), (-5.55, 1.30, 0.65, 0.90),
        (-1.70, 2.05, 0.75, 1.15),
        (2.55, 3.00, 4.30, 0.10), (0.40, 1.75, 0.10, 2.60),
        (4.70, 1.75, 0.10, 2.60), (1.05, 0.50, 1.30, 0.10),
        (3.90, 0.50, 1.60, 0.10),
    ]
    for box in boxes:
        fill_box(image, *box)

    circles = [
        (1.30, 1.20, 0.43), (2.55, 1.20, 0.43), (3.80, 1.20, 0.43),
        (1.30, 2.30, 0.43), (2.55, 2.30, 0.43), (3.80, 2.30, 0.43),
        (-3.00, 0.00, 0.30), (4.40, 0.00, 0.28),
    ]
    for circle in circles:
        fill_circle(image, *circle)

    output = Path(__file__).resolve().parents[1] / 'maps' / 'city_industrial.pgm'
    with output.open('wb') as stream:
        stream.write(f'P5\n{WIDTH} {HEIGHT}\n255\n'.encode('ascii'))
        for row in image:
            stream.write(row)
    print(f'Generated {output} ({WIDTH}x{HEIGHT}, {RESOLUTION} m/cell)')


if __name__ == '__main__':
    main()
