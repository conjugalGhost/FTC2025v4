package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Feeder {
    private DcMotorEx feeder;

    private static final int STEP_TICKS = 144; // ~180°
    private int state = 0; // 1 = forward, -1 = reverse, 0 = stopped

    public Feeder(HardwareMap hardwareMap) {
        try {
            feeder = hardwareMap.get(DcMotorEx.class, "feeder");
            feeder.setDirection(DcMotorEx.Direction.FORWARD);
            feeder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            feeder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            feeder = null;
        }
    }

    public void resetEncoders() {
        if (feeder != null) {
            feeder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            feeder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        state = 0;
    }

    // Forward feed
    public void advanceOneStep() {
        int startPos = feeder != null ? feeder.getCurrentPosition() : 0;

        if (feeder != null) feeder.setPower(1.0);

        ElapsedTime timer = new ElapsedTime();
        while (feeder != null && Math.abs(feeder.getCurrentPosition() - startPos) < STEP_TICKS) {
            if (timer.seconds() > 1.0) break;
        }

        stop();
        state = 1;
    }

    // Reverse feed
    public void reverseOneStep() {
        int startPos = feeder != null ? feeder.getCurrentPosition() : 0;

        if (feeder != null) feeder.setPower(-1.0);

        ElapsedTime timer = new ElapsedTime();
        while (feeder != null && Math.abs(feeder.getCurrentPosition() - startPos) < STEP_TICKS) {
            if (timer.seconds() > 1.0) break;
        }

        stop();
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
