#!/usr/bin/env python3
"""
Enhanced GPIO Tester using gpiozero for Raspberry Pi 5
Clean, safe, and user-friendly GPIO testing tool
"""
from gpiozero import OutputDevice, GPIOPinInUse, BadPinFactory
import time
import sys

def test_pin(pin_num):
    """Test a single GPIO pin using gpiozero"""
    try:
        print(f"\nTesting GPIO {pin_num}")
        print("Connect voltmeter: RED to pin, BLACK to GND")
        print("⚠️  Ensure nothing else is connected to this pin!")
        
        # Create output device
        pin = OutputDevice(pin_num)
        
        # Test LOW
        pin.off()
        print(f"GPIO {pin_num} = OFF (should read 0V)")
        low_voltage = input("Press Enter when measured (or type voltage reading): ")
        
        # Test HIGH
        pin.on()
        print(f"GPIO {pin_num} = ON (should read 3.3V)")
        high_voltage = input("Press Enter when measured (or type voltage reading): ")
        
        # Analyze readings if provided
        if high_voltage.replace('.', '').isdigit():
            voltage = float(high_voltage)
            if voltage < 2.8:
                print(f"⚠️  LOW VOLTAGE DETECTED: {voltage}V")
                print("   Possible causes:")
                print("   - Pin damage from overvoltage")
                print("   - Something connected drawing current")
                print("   - 3.3V rail problem")
                print("   - Check if anything is connected to this pin")
            elif voltage < 3.1:
                print(f"⚠️  REDUCED VOLTAGE: {voltage}V (should be ~3.3V)")
                print("   May indicate minor loading or measurement error")
            else:
                print(f"✓ GOOD VOLTAGE: {voltage}V")
        
        # Back to LOW (safe)
        pin.off()
        print(f"GPIO {pin_num} = OFF")
        
        # Cleanup
        pin.close()
        print("✓ Test complete")
        return True
        
    except GPIOPinInUse:
        print(f"✗ GPIO {pin_num} is already in use by another process")
        return False
    except BadPinFactory:
        print(f"✗ GPIO {pin_num} is not a valid GPIO pin")
        return False
    except Exception as e:
        print(f"✗ Error testing GPIO {pin_num}: {e}")
        return False

def blink_test(pin_num, times=5, delay=0.5):
    """Blink a pin for visual confirmation"""
    try:
        pin = OutputDevice(pin_num)
        print(f"\nBlinking GPIO {pin_num} {times} times (every {delay}s)")
        print("Watch your voltmeter for alternating 0V/3.3V")
        
        for i in range(times):
            pin.on()
            time.sleep(delay)
            pin.off()
            time.sleep(delay)
            print(f"Blink {i+1}/{times}")
        
        pin.close()
        print("✓ Blink test complete")
        
    except GPIOPinInUse:
        print(f"✗ GPIO {pin_num} is already in use")
    except Exception as e:
        print(f"✗ Blink test error: {e}")

def show_pin_info():
    """Display comprehensive pin information"""
    print("\n" + "="*50)
    print("RASPBERRY PI 5 GPIO PIN REFERENCE")
    print("="*50)
    
    # BCM to Physical mapping for all GPIO pins
    pin_map = {
        2: 3, 3: 5, 4: 7, 5: 29, 6: 31, 7: 26, 8: 24, 9: 21, 10: 19, 11: 23,
        12: 32, 13: 33, 14: 8, 15: 10, 16: 36, 17: 11, 18: 12, 19: 35, 20: 38,
        21: 40, 22: 15, 23: 16, 24: 18, 25: 22, 26: 37, 27: 13
    }
    
    print("All GPIO pins (BCM → Physical):")
    for i, (bcm, phys) in enumerate(sorted(pin_map.items())):
        if i % 3 == 0 and i > 0:
            print()  # New line every 3 entries for readability
        print(f"  GPIO {bcm:2d} → Pin {phys:2d}", end="    " if (i + 1) % 3 != 0 else "")
    print()  # Final newline
    
    print(f"\nGround pins (Physical): 6, 9, 14, 20, 25, 30, 34, 39")
    print(f"3.3V pins (Physical): 1, 17")
    print(f"5V pins (Physical): 2, 4")
    print("="*50)

def get_valid_pin():
    """Get a valid GPIO pin number from user"""
    all_pins = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27]
    
    while True:
        try:
            pin = int(input(f"Enter GPIO number (2-27): "))
            if pin in all_pins:
                return pin
            else:
                print(f"Valid GPIO pins: {all_pins}")
        except ValueError:
            print("Please enter a valid number")
        except KeyboardInterrupt:
            print("\nExiting...")
            sys.exit(0)

def main():
    print("Enhanced GPIO Tester for Raspberry Pi 5")
    print("======================================")
    print("Using gpiozero library for safe GPIO control")
    
    # Test if we can access GPIO
    try:
        gpio_test = OutputDevice(4)
        gpio_test.close()
    except Exception as e:
        print(f"⚠️  GPIO access error: {e}")
        print("Make sure you're running as root or in gpio group")
        sys.exit(1)
    
    # Common safe pins to test (BCM numbering) - expanded list
    test_pins = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27]
    
    while True:
        print("\n" + "-"*40)
        print("OPTIONS:")
        print("1. Show pin information")
        print("2. Test specific pin")
        print("3. Test all safe pins")
        print("4. Blink test")
        print("5. Quick blink test (fast)")
        print("7. Test 3.3V power rail")
        print("8. Exit")
        print("-"*40)
        
        try:
            choice = input("Choose option (1-8): ").strip()
        except KeyboardInterrupt:
            print("\nGoodbye!")
            break
        
        if choice == "1":
            show_pin_info()
            
        elif choice == "2":
            pin = get_valid_pin()
            test_pin(pin)
            
        elif choice == "3":
            print(f"\nTesting ALL {len(test_pins)} GPIO pins...")
            print("⚠️  This will test every GPIO pin. Make sure nothing is connected!")
            confirm = input("Continue? (y/n): ").lower()
            if confirm != 'y':
                continue
                
            failed_pins = []
            low_voltage_pins = []
            
            for i, pin in enumerate(test_pins, 1):
                print(f"\n[{i}/{len(test_pins)}] Testing GPIO {pin}")
                success = test_pin(pin)
                
                if not success:
                    failed_pins.append(pin)
                    continue
                
                if i < len(test_pins):  # Not the last pin
                    try:
                        cont = input("Continue to next pin? (y/n/q): ").lower()
                        if cont == 'q':
                            break
                        elif cont != 'y':
                            break
                    except KeyboardInterrupt:
                        break
            
            print(f"\n{'='*50}")
            print("BATCH TEST SUMMARY")
            print("="*50)
            if failed_pins:
                print(f"⚠️  Failed pins: {failed_pins}")
            else:
                print("✓ All tested pins responded normally")
            print("✓ Batch test complete")
            
        elif choice == "4":
            pin = get_valid_pin()
            try:
                times = int(input("How many blinks (default 5): ") or "5")
                delay = float(input("Delay between blinks in seconds (default 0.5): ") or "0.5")
                blink_test(pin, times, delay)
            except ValueError:
                print("Using default values...")
                blink_test(pin)
                
        elif choice == "5":
            pin = get_valid_pin()
            print("Quick blink test (10 fast blinks)")
            blink_test(pin, 10, 0.1)
            
        elif choice == "7":
            print("\n3.3V POWER RAIL TEST")
            print("===================")
            print("Measure voltage between:")
            print("Physical pin 1 (3.3V) and Physical pin 6 (GND)")
            print("Should read 3.3V ±0.1V")
            voltage = input("Enter measured voltage: ")
            if voltage.replace('.', '').isdigit():
                v = float(voltage)
                if v < 3.2:
                    print(f"⚠️  LOW 3.3V RAIL: {v}V")
                    print("   This explains your GPIO voltage issues!")
                    print("   Possible causes:")
                    print("   - Overloaded 3.3V regulator")
                    print("   - Hardware damage")
                    print("   - Too many devices powered from 3.3V")
                elif v > 3.4:
                    print(f"⚠️  HIGH 3.3V RAIL: {v}V (unusual)")
                else:
                    print(f"✓ GOOD 3.3V RAIL: {v}V")
            
        elif choice == "8":
            print("Goodbye!")
            break
            
        else:
            print("❌ Invalid choice. Please select 1-8.")
    
    print("GPIO tester finished safely.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nProgram interrupted. All GPIO pins cleaned up safely.")
        sys.exit(0)