package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;
    private double targetVelocity = 0;

    public Shooter(HardwareMap hardwareMap) {
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
    }

    public void shootForward() {
        targetVelocity = 2800; // example target velocity
        leftShooter.setVelocity(targetVelocity);
        rightShooter.setVelocity(targetVelocity);
    }

    public void stop() {
        targetVelocity = 0;
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }

    public double getTargetVelocity() { return targetVelocity; }
    public double getLeftVelocity() { return leftShooter.getVelocity(); }
    public double getRightVelocity() { return rightShooter.getVelocity(); }

    public boolean isReady() {
        if (targetVelocity <= 0) return false;
        double tolerance = targetVelocity * 0.05;
        return Math.abs(getLeftVelocity() - targetVelocity) <= tolerance &&
                Math.abs(getRightVelocity() - targetVelocity) <= tolerance;
    }

    // ✅ Add this method so TeleOp can call it
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Left Velocity", getLeftVelocity());
        telemetry.addData("Right Velocity", getRightVelocity());
        telemetry.addData("Shooter Ready", isReady());
    }
}