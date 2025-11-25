package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;
    private double targetVelocity = 0;

    // Constants for REV HD Hex direct drive with 90mm wheels
    private static final double TICKS_PER_REV = 28.0;       // encoder CPR
    private static final double WHEEL_DIAMETER_MM = 90.0;   // shooter wheel
    private static final double WHEEL_DIAMETER_IN = WHEEL_DIAMETER_MM / 25.4;
    private static final double WHEEL_CIRCUMFERENCE_FT = Math.PI * WHEEL_DIAMETER_IN / 12.0;
    private static final double SLIP_FACTOR = 0.85;         // compression/slip

    public Shooter(HardwareMap hardwareMap) {
        leftShooter  = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");

        if (leftShooter != null) {
            leftShooter.setDirection(DcMotorEx.Direction.REVERSE);
            leftShooter.setVelocityPIDFCoefficients(50.0, 0.0, 0.0, 14.0);
        }
        if (rightShooter != null) {
            rightShooter.setDirection(DcMotorEx.Direction.FORWARD);
            rightShooter.setVelocityPIDFCoefficients(50.0, 0.0, 0.0, 14.0);
        }
    }

    /** Set shooter velocity in ticks/sec */
    public void setVelocity(double velocity) {
        targetVelocity = velocity;
        if (leftShooter != null) leftShooter.setVelocity(targetVelocity);
        if (rightShooter != null) rightShooter.setVelocity(targetVelocity);
    }

    public void stop() {
        targetVelocity = 0;
        if (leftShooter != null) leftShooter.setPower(0);
        if (rightShooter != null) rightShooter.setPower(0);
    }

    /** Accessors */
    public double getLeftVelocity() { return leftShooter != null ? leftShooter.getVelocity() : 0; }
    public double getRightVelocity() { return rightShooter != null ? rightShooter.getVelocity() : 0; }

    /** Derived physics values */
    public double getAverageVelocityTicks() {
        return (getLeftVelocity() + getRightVelocity()) / 2.0;
    }

    public double getRPM() {
        return (getAverageVelocityTicks() / TICKS_PER_REV) * 60.0;
    }

    public double getExitVelocityFtPerSec() {
        double rpm = getRPM();
        double surfaceSpeed = (rpm / 60.0) * WHEEL_CIRCUMFERENCE_FT;
        return surfaceSpeed * SLIP_FACTOR;
    }

    /** Telemetry */
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Target Vel (ticks/sec)", targetVelocity);
        telemetry.addData("Left Vel", getLeftVelocity());
        telemetry.addData("Right Vel", getRightVelocity());
        telemetry.addData("Shooter RPM", "%.0f", getRPM());
        telemetry.addData("Exit Velocity (ft/s)", "%.1f", getExitVelocityFtPerSec());
    }
}