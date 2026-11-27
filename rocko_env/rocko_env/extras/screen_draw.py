"""Paint a screen_layout page into a picture, and turn the picture into the panel's pixel format.

Needs Pillow (drawing) and numpy (the pixel conversion). StatusScreen.py imports this inside a try, so a Pi
without them still runs the robot; the screen just reports "Pillow not installed" as its error.
"""
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from rocko_env.extras import screen_layout as L

_fonts = {}


def font(size):
    """Pillow's built-in scalable font (Pillow 10.1 or later), so no font file has to be installed on the Pi."""
    if size not in _fonts:
        _fonts[size] = ImageFont.load_default(size=size)
    return _fonts[size]


def render(view, page):
    """The whole screen for this view and page number, as a WIDTH x HEIGHT RGB picture."""
    image = Image.new("RGB", (L.WIDTH, L.HEIGHT), L.BACKGROUND)
    draw = ImageDraw.Draw(image)

    # The strip at the top: filled with the status colour, so it reads from across the room.
    text, colour = L.banner(view)
    draw.rectangle((0, 0, L.WIDTH, L.BANNER_H), fill=colour)
    draw.text((L.WIDTH // 2, L.BANNER_H // 2), text, fill=L.WHITE, font=font(L.BANNER_SIZE), anchor="mm")

    # The page's lines, top to bottom, cut to the space between the strip and the buttons.
    _, page_function = L.PAGES[page % len(L.PAGES)]
    y = L.CONTENT_TOP
    for line in L.fit_lines(page_function(view), L.CONTENT_HEIGHT):
        draw.text((L.MARGIN_X, y), line.text, fill=line.colour, font=font(line.size))
        y += line.size + L.LINE_GAP

    # The three touch buttons.
    third = L.WIDTH // len(L.BUTTONS)
    inset = L.BUTTON_INSET
    for i, label in enumerate(L.button_labels(view, page)):
        box = (i * third + inset, L.BAR_Y + inset, (i + 1) * third - inset, L.HEIGHT - inset)
        draw.rounded_rectangle(box, radius=L.BUTTON_RADIUS, fill=L.BUTTON)
        draw.text(((box[0] + box[2]) // 2, (box[1] + box[3]) // 2), label, fill=L.WHITE, font=font(L.BUTTON_SIZE),
                  anchor="mm")
    return image


def to_rgb565(image):
    """RGB picture -> the bytes a 16-bit framebuffer wants: per pixel 5 bits red, 6 green, 5 blue, low byte first.

    The panel stores 2 bytes per pixel instead of 3, so each colour keeps only its top bits: red and blue lose 3 bits,
    green 2 (the eye sees green best). numpy does the shifts for all 153,600 pixels at once; a Python loop over them
    would take most of a second on the Pi."""
    rgb = np.asarray(image.convert("RGB"), dtype=np.uint16)
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    packed = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
    return packed.astype("<u2").tobytes()   # "<u2": unsigned 16-bit, little-endian (low byte first)
