package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    public Drive(HardwareMap hardwareMap) {
        try {
            frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
            backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
            backRight = hardwareMap.get(DcMotorEx.class, "backRight");

            // Normalize directions so +power = forward
            frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
            backLeft.setDirection(DcMotorEx.Direction.FORWARD);
            frontRight.setDirection(DcMotorEx.Direction.REVERSE);
            backRight.setDirection(DcMotorEx.Direction.REVERSE);
        } catch (Exception e) {
            // Handle missing motors gracefully
        }
    }

    // --- Core helpers ---

    public void setDrivePower(double power) {
        if (frontLeft != null) frontLeft.setPower(power);
        if (backLeft != null) backLeft.setPower(power);
        if (frontRight != null) frontRight.setPower(power);
        if (backRight != null) backRight.setPower(power);
    }

    public void stop() {
        if (frontLeft != null) frontLeft.setPower(0.0);
        if (backLeft != null) backLeft.setPower(0.0);
        if (frontRight != null) frontRight.setPower(0.0);
        if (backRight != null) backRight.setPower(0.0);
    }

    // --- Corrected turn method ---
    public void turn(double power) {
        // Positive power = clockwise turn
        if (frontLeft != null) frontLeft.setPower(power);
        if (backLeft != null) backLeft.setPower(power);
        if (frontRight != null) frontRight.setPower(-power);
        if (backRight != null) backRight.setPower(-power);
    }

    // --- Accessors for AutonBase ---
    public DcMotorEx getFrontLeft() { return frontLeft; }
    public DcMotorEx getFrontRight() { return frontRight; }
    public DcMotorEx getBackLeft() { return backLeft; }
    public DcMotorEx getBackRight() { return backRight; }

    // Utility to set run mode for all motors
    public void setRunModeAll(DcMotorEx.RunMode mode) {
        if (frontLeft != null) frontLeft.setMode(mode);
        if (frontRight != null) frontRight.setMode(mode);
        if (backLeft != null) backLeft.setMode(mode);
        if (backRight != null) backRight.setMode(mode);
    }
}