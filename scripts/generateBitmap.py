#!/usr/bin/env python3
"""
Convert an image to a 1-bit monochrome BMP format for SSD1306 OLED displays.
Generates a C++ array that can be included in the firmware.

Usage:
    python generateBitmap.py input_image.png output_name

Example:
    python generateBitmap.py splash.png splash_screen
    This will create a file containing: static const std::array<uint8_t, SIZE> splash_screen = {...};
"""

import sys
from PIL import Image
import struct


def convert_to_monochrome_bmp(input_path, output_name):
    """
    Convert an image to 1-bit monochrome BMP format (128x64).
    Returns the BMP data as bytes.
    """
    # Load and convert image
    img = Image.open(input_path)

    # Resize to 128x64 if needed
    if img.size != (128, 64):
        print(f"Resizing image from {img.size} to (128, 64)")
        img = img.resize((128, 64), Image.Resampling.LANCZOS)

    # Convert to 1-bit (black and white)
    img = img.convert('1')

    # BMP format parameters
    width = 128
    height = 64
    bits_per_pixel = 1

    # Calculate padding (BMP rows must be aligned to 4-byte boundaries)
    bytes_per_row = (width + 7) // 8  # 16 bytes for 128 pixels
    padding = (4 - (bytes_per_row % 4)) % 4
    bytes_per_row_padded = bytes_per_row + padding

    # BMP file structure sizes
    file_header_size = 14
    dib_header_size = 108  # BITMAPV4HEADER for better compatibility
    color_table_size = 8   # 2 colors * 4 bytes each
    pixel_data_size = bytes_per_row_padded * height
    file_size = file_header_size + dib_header_size + color_table_size + pixel_data_size

    # Create BMP data
    bmp_data = bytearray()

    # --- BMP File Header (14 bytes) ---
    bmp_data += b'BM'                              # Signature
    bmp_data += struct.pack('<I', file_size)       # File size
    bmp_data += struct.pack('<H', 0)               # Reserved 1
    bmp_data += struct.pack('<H', 0)               # Reserved 2
    bmp_data += struct.pack('<I', file_header_size + dib_header_size + color_table_size)  # Pixel data offset

    # --- DIB Header (BITMAPV4HEADER - 108 bytes) ---
    bmp_data += struct.pack('<I', dib_header_size) # DIB header size
    bmp_data += struct.pack('<i', width)           # Image width
    bmp_data += struct.pack('<i', height)          # Image height (positive = bottom-up)
    bmp_data += struct.pack('<H', 1)               # Color planes (always 1)
    bmp_data += struct.pack('<H', bits_per_pixel)  # Bits per pixel (1 for monochrome)
    bmp_data += struct.pack('<I', 0)               # Compression (0 = none)
    bmp_data += struct.pack('<I', pixel_data_size) # Image size
    bmp_data += struct.pack('<i', 2835)            # X pixels per meter (~72 DPI)
    bmp_data += struct.pack('<i', 2835)            # Y pixels per meter
    bmp_data += struct.pack('<I', 2)               # Colors in palette (2 for monochrome)
    bmp_data += struct.pack('<I', 2)               # Important colors

    # V4 header additions (rest of 108 bytes)
    bmp_data += struct.pack('<I', 0x00FF0000)      # Red mask
    bmp_data += struct.pack('<I', 0x0000FF00)      # Green mask
    bmp_data += struct.pack('<I', 0x000000FF)      # Blue mask
    bmp_data += struct.pack('<I', 0xFF000000)      # Alpha mask
    bmp_data += b'Win '                             # Color space type
    bmp_data += b'\x00' * 36                        # CIEXYZTRIPLE (unused)
    bmp_data += struct.pack('<I', 0)               # Gamma Red
    bmp_data += struct.pack('<I', 0)               # Gamma Green
    bmp_data += struct.pack('<I', 0)               # Gamma Blue

    # --- Color Table (8 bytes: 2 colors) ---
    # Color 0: White (255, 255, 255, 0)
    bmp_data += struct.pack('<BBBB', 255, 255, 255, 0)
    # Color 1: Black (0, 0, 0, 0)
    bmp_data += struct.pack('<BBBB', 0, 0, 0, 0)

    # --- Pixel Data ---
    # BMP stores pixels bottom-to-top, left-to-right
    # Each row must be padded to 4-byte boundary
    pixels = img.load()

    for y in range(height - 1, -1, -1):  # Bottom to top
        row_data = bytearray()
        for x in range(0, width, 8):
            byte_val = 0
            for bit in range(8):
                if x + bit < width:
                    # In PIL's 1-bit mode: 0 = black, 255 = white
                    # In BMP with our color table: 0 = black, 1 = white
                    pixel_val = 1 if pixels[x + bit, y] != 0 else 0
                    byte_val |= (pixel_val << (7 - bit))
            row_data.append(byte_val)

        # Add padding
        row_data += b'\x00' * padding
        bmp_data += row_data

    return bytes(bmp_data)


def generate_cpp_array(bmp_data, array_name):
    """
    Generate C++ array declaration from BMP data.
    """
    output = f"static const std::array<uint8_t, {len(bmp_data)}> {array_name} = {{\n"

    # Format as hex bytes, 19 per line for readability
    for i in range(0, len(bmp_data), 19):
        chunk = bmp_data[i:i+19]
        hex_str = ', '.join(f'0x{b:02x}' for b in chunk)
        output += f"    {hex_str}"
        if i + 19 < len(bmp_data):
            output += ","
        output += "\n"

    output += "};\n"
    return output


def main():
    if len(sys.argv) != 3:
        print("Usage: python generateBitmap.py <input_image> <output_name>")
        print("\nExample:")
        print("  python generateBitmap.py splash.png splash_screen")
        print("\nThis will output C++ code to stdout that you can paste into MenuScreens.h")
        sys.exit(1)

    input_path = sys.argv[1]
    output_name = sys.argv[2]

    try:
        # Convert image to BMP
        print(f"Converting {input_path} to 1-bit monochrome BMP (128x64)...", file=sys.stderr)
        bmp_data = convert_to_monochrome_bmp(input_path, output_name)

        # Generate C++ array
        print(f"Generating C++ array '{output_name}'...", file=sys.stderr)
        cpp_code = generate_cpp_array(bmp_data, output_name)

        # Output to stdout
        print(cpp_code)

        print(f"\nSuccess! Generated {len(bmp_data)} bytes.", file=sys.stderr)
        print(f"Copy the array declaration above into your MenuScreens.h file.", file=sys.stderr)

    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
