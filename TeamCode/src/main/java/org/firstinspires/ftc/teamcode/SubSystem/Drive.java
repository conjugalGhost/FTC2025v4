package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    // --- Trim Factors to balance motor performance ---
    // Decrease these (e.g., 0.95) for motors that are "too fast"
    private double trimFL = 1.0;
    private double trimFR = 1.0;
    private double trimBL = 1.0;
    private double trimBR = 1.0; // backRight is reported slow, others might need reduction

    public Drive(HardwareMap hardwareMap) {
        try {
            frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
            backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
            backRight = hardwareMap.get(DcMotorEx.class, "backRight");

            // Specifically flipping front motors to match the back motors
            frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
            backLeft.setDirection(DcMotorEx.Direction.REVERSE);
            frontRight.setDirection(DcMotorEx.Direction.REVERSE);
            backRight.setDirection(DcMotorEx.Direction.FORWARD);

            // Ensure motors are in a mode that responds to setPower() immediately
            setRunModeAll(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            setZeroPowerBehaviorAll(DcMotorEx.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            // Handle missing motors gracefully
        }
    }

    public void setTrim(double fl, double fr, double bl, double br) {
        this.trimFL = fl;
        this.trimFR = fr;
        this.trimBL = bl;
        this.trimBR = br;
    }

    public void setZeroPowerBehaviorAll(DcMotorEx.ZeroPowerBehavior behavior) {
        if (frontLeft != null) frontLeft.setZeroPowerBehavior(behavior);
        if (frontRight != null) frontRight.setZeroPowerBehavior(behavior);
        if (backLeft != null) backLeft.setZeroPowerBehavior(behavior);
        if (backRight != null) backRight.setZeroPowerBehavior(behavior);
    }

    // --- Core helpers with Trim applied ---

    public void setMotorPowers(double flP, double frP, double blP, double brP) {
        if (frontLeft != null)  frontLeft.setPower(flP * trimFL);
        if (frontRight != null) frontRight.setPower(frP * trimFR);
        if (backLeft != null)   backLeft.setPower(blP * trimBL);
        if (backRight != null)  backRight.setPower(brP * trimBR);
    }

    public void setDrivePower(double power) {
        setMotorPowers(power, power, power, power);
    }

    public void stop() {
        setMotorPowers(0, 0, 0, 0);
    }

    // --- Corrected turn method with trim ---
    public void turn(double power) {
        // Positive power = clockwise turn
        setMotorPowers(power, -power, power, -power);
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