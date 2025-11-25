package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;
    private double targetPower = 0;

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
        }
        if (rightShooter != null) {
            rightShooter.setDirection(DcMotorEx.Direction.FORWARD);
        }
    }

    /** Set shooter power (open-loop, 0.0–1.0) */
    public void setPower(double power) {
        targetPower = power;
        if (leftShooter != null) leftShooter.setPower(targetPower);
        if (rightShooter != null) rightShooter.setPower(targetPower);
    }

    /** Convenience wrappers for preset shooter power (70% forward/reverse) */
    public void shootForward() {
        setPower(0.7);   // 70% forward
    }

    public void shootReverse() {
        setPower(-0.7);  // 70% reverse
    }

    public void stop() {
        targetPower = 0;
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
        telemetry.addData("Target Power", targetPower);
        telemetry.addData("Left Vel (ticks/sec)", getLeftVelocity());
        telemetry.addData("Right Vel (ticks/sec)", getRightVelocity());
        telemetry.addData("Shooter RPM", "%.0f", getRPM());
        telemetry.addData("Exit Velocity (ft/s)", "%.1f", getExitVelocityFtPerSec());
    }
}