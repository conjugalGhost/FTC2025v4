package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Subsystem for the Robot Shooter (Flywheel Launcher).
 * Optimized for a single REV HD Hex motor setup.
 */
public class Shooter {
    private DcMotorEx shooter;
    
    private double targetVelocityTicksPerSec = 0;
    private double targetPower = 0;

    // Constants for REV HD Hex motors (28 CPR) at 4x quadrature = 112 ticks/rev
    private static final double TICKS_PER_REV = 112.0;
    private static final double WHEEL_DIAMETER_IN = 3.54;   // ~90mm
    private static final double WHEEL_CIRCUMFERENCE_FT = Math.PI * WHEEL_DIAMETER_IN / 12.0;
    private static final double SLIP_FACTOR = 0.85;         // Compression/slip factor for exit velocity

    public Shooter(HardwareMap hardwareMap) {
        // Try to find the motor under the name "shooter"
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        if (shooter != null) {
            shooter.setDirection(DcMotorEx.Direction.REVERSE);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /** Set shooter velocity in ticks per second (closed-loop) */
    public void setVelocity(double ticksPerSec) {
        targetVelocityTicksPerSec = ticksPerSec;
        targetPower = 0;
        if (shooter != null) {
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setVelocity(targetVelocityTicksPerSec);
        }
    }

    /** Set shooter velocity in RPM */
    public void setTargetRPM(double rpm) {
        double ticksPerSec = (rpm / 60.0) * TICKS_PER_REV;
        setVelocity(ticksPerSec);
    }

    /** Set shooter power (open-loop, 0.0–1.0) */
    public void setPower(double power) {
        targetPower = power;
        targetVelocityTicksPerSec = 0; 
        if (shooter != null) {
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter.setPower(targetPower);
        }
    }

    /** Presets for shooter speed */
    public void shootForward() {
        setTargetRPM(2400); // Target RPM for scoring
    }

    public void shootReverse() {
        setTargetRPM(-1000); // Reverse for clearing jams
    }

    public void stop() {
        targetVelocityTicksPerSec = 0;
        targetPower = 0;
        if (shooter != null) shooter.setPower(0);
    }

    /** Accessors */
    public double getVelocity() { 
        return shooter != null ? shooter.getVelocity() : 0; 
    }
    
    /** Compatibility methods for existing TeleOp and Logger code */
    public double getLeftVelocity() { return getVelocity(); }
    public double getRightVelocity() { return getVelocity(); }

    public double getRPM() {
        return (getVelocity() / TICKS_PER_REV) * 60.0;
    }

    public double getTargetRPM() {
        return (targetVelocityTicksPerSec / TICKS_PER_REV) * 60.0;
    }
    
    public double getTargetPower() {
        return targetPower;
    }

    public double getExitVelocityFtPerSec() {
        double rpm = getRPM();
        double surfaceSpeed = (rpm / 60.0) * WHEEL_CIRCUMFERENCE_FT;
        return surfaceSpeed * SLIP_FACTOR;
    }

    public boolean isAtTargetVelocity() {
        if (targetVelocityTicksPerSec == 0) return false;
        double error = Math.abs(targetVelocityTicksPerSec - getVelocity());
        return error < (TICKS_PER_REV * 2); // 2 revs/sec tolerance
    }

    /** Telemetry */
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("=== SHOOTER (SINGLE MOTOR) ===");
        telemetry.addData("Target RPM", "%.0f", getTargetRPM());
        telemetry.addData("Current RPM", "%.0f", getRPM());
        telemetry.addData("At Target", isAtTargetVelocity());
        telemetry.addData("Exit Velocity (ft/s)", "%.1f", getExitVelocityFtPerSec());
        if (shooter == null) {
            telemetry.addLine("WARNING: Shooter motor 'shooter' not found!");
        }
    }
}
