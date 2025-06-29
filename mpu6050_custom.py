import smbus2
import time

MUX_ADDR = 0x70
I2C_BUS = 1  # Use 0 for older Pi revisions

def select_mux_channel(bus, channel):
    """Selects the specified channel on the TCA9548A multiplexer."""
    if 0 <= channel <= 7:
        try:
            bus.write_byte(MUX_ADDR, 1 << channel)
            time.sleep(0.01)  # Give it a moment to switch
        except Exception as e:
            print(f"Error selecting channel {channel}: {e}")
            return False
        return True
    else:
        print("Invalid multiplexer channel")
        return False

def scan_channel(bus, channel):
    """Scans for I2C devices on a specific multiplexer channel."""
    devices = []
    for addr in range(0x03, 0x77 + 1):
        try:
            bus.read_byte(addr)  # Try to read a byte
            devices.append(addr)
        except OSError as e:
            if e.errno != 121:  # Ignore "Remote I/O error" which means no device
                print(f"Error at address 0x{addr:02x} on channel {channel}: {e}")
        except Exception as e:
            print(f"General error at address 0x{addr:02x} on channel {channel}: {e}")
    return devices

print("Scanning I2C bus through TCA9548A channels:")
try:
    main_bus = smbus2.SMBus(I2C_BUS)
    for channel in range(8):
        print(f"\n--- Channel {channel} ---")
        if select_mux_channel(main_bus, channel):
            devices_on_channel = scan_channel(main_bus, channel)
            if devices_on_channel:
                print(f"Detected devices on channel {channel}: {[hex(addr) for addr in devices_on_channel]}")
            else:
                print("No devices found on this channel.")
        else:
            print(f"Could not select channel {channel}.")
    main_bus.close()
except Exception as e:
    print(f"An error occurred during the scan: {e}")

print("\nScan complete.")