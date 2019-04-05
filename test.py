#!/usr/bin/env python3
'''test'''
import time
import robotarmlib as r

###########################################################################
##### Testing robotarmlib                                                 #
#####                                                                     #
##### Author: HUNG Lok Shun                                               #
##### Email: lshung@connect.hku.hk                                        #
##### License: CC BY-NC 3.0                                               #
##### Last modified: 2018-10-11 23:00                                     #
##### Python version: 3.6+ (3.7.0)                                        #
###########################################################################


def main():
    '''main'''
    print("Testing robotarmlib")
    time.sleep(0.5)

    arm = r.RobotArm(
        attachment=r.ATTACHMENT_GRIPPER,
        use_default_serial_port=True,
        start_connection=True,
        verbose=True
    )

    # time.sleep(1.5)
    # print(f"{arm.is_connection_open()} {arm.serial_port.is_open}")

    # arm.gcode_attachment_high()
    # time.sleep(0.5)
    # arm.gcode_attachment_low()
    # time.sleep(0.5)

    # arm.gcode_send("M17")
    # time.sleep(0.5)

    # arm.gcode_send("G1 X0.0 Y19.5 Z134.0")
    # time.sleep(0.5)

    ##################################################################

    arm.gcode_pos(r.POS_HOME)
    time.sleep(0.5)

    arm.gcode_pos(r.POS_BOTTOM)
    arm.gcode_z_coarse_neg()
    time.sleep(0.1)
    arm.gcode_z_coarse_neg()
    time.sleep(0.1)
    arm.gcode_z_coarse_neg()
    time.sleep(0.1)
    arm.gcode_z_coarse_neg()
    time.sleep(0.1)
    arm.gcode_z_coarse_neg()
    time.sleep(0.1)
    arm.gcode_z_coarse_neg()
    time.sleep(0.1)
    arm.gcode_z_fine_neg()
    time.sleep(0.5)

    arm.gcode_attachment_high()
    time.sleep(0.5)

    arm.gcode_pos(r.POS_END_STOP)
    time.sleep(0.5)

    arm.gcode_attachment_low()
    time.sleep(0.5)

    print("all done")
    arm.close_connection()


if __name__ == "__main__":
    main()
