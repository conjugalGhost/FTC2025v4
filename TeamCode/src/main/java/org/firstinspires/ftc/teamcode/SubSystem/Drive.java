package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Drive {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // Constants for drive math (4:1 gearing, 4" wheels)
    private static final double TICKS_PER_REV = 28.0;        // REV HD Hex encoder CPR
    private static final double GEAR_RATIO = 4.0;            // motor revs per wheel rev
    private static final double WHEEL_DIAMETER_IN = 4.0;     // mecanum wheel diameter
    private static final double WHEEL_CIRCUMFERENCE_FT = Math.PI * WHEEL_DIAMETER_IN / 12.0;
    private static final double TICKS_PER_WHEEL_REV = TICKS_PER_REV * GEAR_RATIO; // 112
    private static final double FT_PER_TICK = WHEEL_CIRCUMFERENCE_FT / TICKS_PER_WHEEL_REV;

    public Drive(HardwareMap hardwareMap) {
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight");

        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        PIDFCoefficients drivePIDF = new PIDFCoefficients(10.0, 0.0, 0.0, 2.0);
        frontLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, drivePIDF);
        frontRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, drivePIDF);
        backLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, drivePIDF);
        backRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, drivePIDF);
    }

    /** TeleOp drive with gamepad sticks (robot-centric mecanum) */
    public void driveWithGamepad(Gamepad gamepad) {
        double y  = -gamepad.left_stick_y;  // forward/back
        double x  = gamepad.left_stick_x;   // strafe
        double rx = gamepad.right_stick_x;  // rotation

        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(bl),
                Math.max(Math.abs(fr), Math.abs(br))));
        if (max > 1.0) {
            fl /= max; bl /= max; fr /= max; br /= max;
        }

        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    /** Simple power set for all motors */
    public void setDrivePower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    /** Simple in-place turn (manual control) */
    public void turn(double power) {
        double p = Math.max(-1.0, Math.min(1.0, power));
        frontLeft.setPower(p);
        backLeft.setPower(p);
        frontRight.setPower(-p);
        backRight.setPower(-p);
    }

    /** Stop all motors */
    public void stop() {
        setDrivePower(0);
    }

    // Conversion helpers for auton/telemetry
    public double ticksToFeet(int ticks) {
        return ticks * FT_PER_TICK;
    }

    public int feetToTicks(double feet) {
        return (int)(feet / FT_PER_TICK);
    }

    public double ticksPerSecToFtPerSec(double ticksPerSec) {
        return ticksPerSec * FT_PER_TICK;
    }

    // Accessors for auton routines
    public DcMotorEx getFrontLeft() { return frontLeft; }
    public DcMotorEx getFrontRight() { return frontRight; }
    public DcMotorEx getBackLeft() { return backLeft; }
    public DcMotorEx getBackRight() { return backRight; }
}