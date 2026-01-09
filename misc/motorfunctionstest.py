from motorfunctions import MotorFunctions
motors=MotorFunctions()
def main():
    motors.set_all_direction_pins({
        # DIR A and DIR B and PWM must be defined such that DRIVE A 0XFFF AND DRIVE B 00000 WHEN FORWARD
        "nrr":[0,1,8],
        "nrl":[2,3,9],
        "nfr":[6,7,11],
        "nfl":[4,5,10]
    })
    motors.drive_all_wheels({
        "nrr":50,
        "nrl":50,
        "nfr":50,
        "nfl":50
    })
if __name__ == "__main__":
    main()
    #print("hello")
    #mecanum_direction_pins = {"nrr":[0,1] , "nrl":[2,3], "nfr":[6,7], "nfl":[4,5]}