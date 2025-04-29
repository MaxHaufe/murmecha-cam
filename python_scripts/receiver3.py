import serial
import struct
import time
import numpy as np
from PIL import Image


def receive_data():
    # Configure the serial port
    ser = serial.Serial(
        port='/dev/ttyACM0',  # For Windows: 'COM3' or similar
        baudrate=2000000,
        timeout=1
    )

    try:
        while True:
            # Look for header
            header = bytearray()
            while len(header) < 4:
                byte = ser.read(1)
                if not byte:
                    continue

                header.append(byte[0])
                # Check if we have a valid header
                if len(header) == 4 and header != b'\xAA\xBB\xCC\xDD':
                    # Not a valid header, shift and continue
                    header = header[1:]

            print("Found header")

            # Read image size, width, height (4 bytes each)
            size_bytes = ser.read(4)
            if not size_bytes or len(size_bytes) != 4:
                print("Failed to read size")
                continue

            image_size = struct.unpack("<I", size_bytes)[0]

            width_bytes = ser.read(4)
            if not width_bytes or len(width_bytes) != 4:
                print("Failed to read width")
                continue

            width = struct.unpack("<I", width_bytes)[0]

            height_bytes = ser.read(4)
            if not height_bytes or len(height_bytes) != 4:
                print("Failed to read height")
                continue

            height = struct.unpack("<I", height_bytes)[0]

            print(f"Expected image: {image_size} bytes, {width}x{height}")

            # Read image data
            image_data = bytearray()
            bytes_received = 0

            # Set longer timeout for image data
            ser.timeout = 5

            while bytes_received < image_size:
                chunk = ser.read(min(512, image_size - bytes_received))
                if not chunk:
                    print(f"Timeout after receiving {bytes_received}/{image_size} bytes")
                    break

                image_data.extend(chunk)
                bytes_received += len(chunk)
                print(f"Received {bytes_received}/{image_size} bytes", end="\r")

            print()  # New line after progress updates

            # Check if we received the expected amount of data
            if bytes_received != image_size:
                print(f"Incomplete image: got {bytes_received}/{image_size} bytes")
                continue

            # Look for footer
            footer = ser.read(4)
            if footer != b'\xDD\xCC\xBB\xAA':
                print(f"Invalid footer: {footer.hex()}")
                continue

            print("Valid footer received")

            # Save the image
            try:
                # Assume RGB format (3 bytes per pixel)
                if width * height * 3 == image_size:
                    img_array = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width, 3))
                    img = Image.fromarray(img_array)
                # Grayscale format (1 byte per pixel)
                elif width * height == image_size:
                    img_array = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width))
                    img = Image.fromarray(img_array, mode='L')
                else:
                    print(f"Image dimensions ({width}x{height}) don't match data size ({image_size})")
                    continue

                timestamp = int(time.time())
                filename = f"saved_files/captured_image_{timestamp}.png"
                img.save(filename)
                print(f"Image saved to {filename}")

            except Exception as e:
                print(f"Error saving image: {e}")

            # Reset timeout for header detection
            ser.timeout = 1

    except KeyboardInterrupt:
        print("\nStopping data reception")
    finally:
        ser.close()


if __name__ == "__main__":
    receive_data()