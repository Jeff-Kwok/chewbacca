#!/usr/bin/env python3
import time
from motorfunctions import MotorFunctions

mf = MotorFunctions()

mf.set_all_direction_pins({
    # [DIR_A, DIR_B, PWM_CH]
    "nrr": [0, 1, 8],    # right rear
    "nrl": [2, 3, 11],   # left rear
    "nfr": [4, 5, 9],    # right front
    "nfl": [6, 7, 10],   # left front
})

def pulse_wheel(name: str, speed: float, on_s: float = 1.5, off_s: float = 0.6):
    print(f"\n=== {name} : +{speed} ===")
    mf.drive_wheel(name, +speed)
    time.sleep(on_s)
    mf.drive_wheel(name, 0.0)
    time.sleep(off_s)

    print(f"=== {name} : -{speed} ===")
    mf.drive_wheel(name, -speed)
    time.sleep(on_s)
    mf.drive_wheel(name, 0.0)
    time.sleep(off_s)

def test_each_wheel(speed: float = 50.0):
    # one-by-one wheel check
    for wheel in ["nfr", "nfl", "nrr", "nrl"]:
        pulse_wheel(wheel, speed)

def sweep_pwm_channels(speed: float = 50.0):
    """
    If you're unsure which PWM channel goes to which motor ESC/driver,
    this forces each wheel entry to use a different PWM channel and pulses.
    (Uses the SAME direction pins, only swaps PWM channel number.)
    """
    print("\n\n=== PWM CHANNEL SWEEP ===")
    base = {
        "nrr": [0, 1, 8],
        "nrl": [2, 3, 11],
        "nfr": [4, 5, 9],
        "nfl": [6, 7, 10],
    }

    # Try these channels (edit if you want)
    chans_to_try = [8, 9, 10, 11, 12, 13, 14, 15]

    for ch in chans_to_try:
        # temporarily set ALL wheels to the same PWM channel "ch"
        tmp = {k: [v[0], v[1], ch] for k, v in base.items()}
        mf.set_all_direction_pins(tmp)

        print(f"\n--- Forcing ALL wheels to PWM channel {ch} ---")
        # pulse just one wheel command; if the wrong motor moves, you found wiring mismatch
        pulse_wheel("nfr", speed, on_s=1.0, off_s=0.5)

    # restore original mapping
    mf.set_all_direction_pins(base)

if __name__ == "__main__":
    try:
        print("Starting wheel test. Ctrl+C to stop.\n")
        test_each_wheel(speed=20.0)

        # optional: uncomment if you want to brute-force PWM wiring sanity
        # sweep_pwm_channels(speed=20.0)

        print("\nDone.")
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        try:
            mf.brake_all_motors(message_toggle=False)
        except Exception:
            pass
