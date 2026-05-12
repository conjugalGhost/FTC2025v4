package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Feeder {
    private DcMotorEx feeder;

    // 125:1 Gearbox (3x 5:1 cartridges)
    // 28 (shaft) * 125 = 3500 ticks per revolution
    // 3500 ticks = 360 degrees
    private static final int STEP_TICKS = 3500;
    private int state = 0; // 1 = forward, -1 = reverse, 0 = stopped

    public Feeder(HardwareMap hardwareMap) {
        try {
            feeder = hardwareMap.get(DcMotorEx.class, "feeder");
            feeder.setDirection(DcMotorEx.Direction.FORWARD);
            feeder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            feeder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            feeder.setTargetPosition(0);
            feeder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            feeder.setPower(0.8); // High torque but slightly limited speed for accuracy
        } catch (Exception e) {
            feeder = null;
        }
    }

    public void resetEncoders() {
        if (feeder != null) {
            feeder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            feeder.setTargetPosition(0);
            feeder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
        state = 0;
    }

    // Forward feed
    public void advanceOneStep() {
        if (feeder == null) return;
        int target = feeder.getCurrentPosition() + STEP_TICKS;
        feeder.setTargetPosition(target);
        feeder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        feeder.setPower(0.8);
        state = 1;
    }

    // Reverse feed
    public void reverseOneStep() {
        if (feeder == null) return;
        int target = feeder.getCurrentPosition() - STEP_TICKS;
        feeder.setTargetPosition(target);
        feeder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        feeder.setPower(0.8);
        state = -1;
    }

    public void stop() {
        if (feeder != null) feeder.setPower(0.0);
        state = 0;
    }

    public int getState() { return state; }

    /** Compatibility methods for Logger and other subsystems */
    public int getLeftPosition() { return getPosition(); }
    public int getRightPosition() { return getPosition(); }
    public int getAveragePosition() { return getPosition(); }

    public int getPosition() {
        return feeder != null ? feeder.getCurrentPosition() : 0;
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("=== FEEDER (SINGLE MOTOR) ===");
        telemetry.addData("State", state == 1 ? "Forward" : state == -1 ? "Reverse" : "Stopped");
        if (feeder != null) {
            telemetry.addData("Position (ticks)", feeder.getCurrentPosition());
        } else {
            telemetry.addLine("WARNING: Feeder motor 'feeder' not found!");
        }
    }
}
