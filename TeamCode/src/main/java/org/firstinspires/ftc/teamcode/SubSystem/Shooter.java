package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;
    private double targetVelocity = 0;

    public Shooter(HardwareMap hardwareMap) {
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
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

    public void shootForward() {
        targetVelocity = 2200;
        if (leftShooter != null) leftShooter.setVelocity(targetVelocity);
        if (rightShooter != null) rightShooter.setVelocity(targetVelocity);
    }

    public void shootReverse() {
        targetVelocity = -2200;
        if (leftShooter != null) leftShooter.setVelocity(targetVelocity);
        if (rightShooter != null) rightShooter.setVelocity(targetVelocity);
    }

    public void stop() {
        targetVelocity = 0;
        if (leftShooter != null) leftShooter.setPower(0);
        if (rightShooter != null) rightShooter.setPower(0);
    }

    public double getLeftVelocity() { return leftShooter != null ? leftShooter.getVelocity() : 0; }
    public double getRightVelocity() { return rightShooter != null ? rightShooter.getVelocity() : 0; }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Target Vel", targetVelocity);
        telemetry.addData("Left Vel", getLeftVelocity());
        telemetry.addData("Right Vel", getRightVelocity());
    }
}