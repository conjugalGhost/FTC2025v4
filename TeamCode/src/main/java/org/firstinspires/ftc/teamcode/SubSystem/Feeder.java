package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Feeder {
    private DcMotorEx feederLeft;
    private DcMotorEx feederRight;

    private int leftTarget = 0;
    private int rightTarget = 0;

    private static final int STEP_TICKS = 144; // ~180°

    // Feeder state: 1 = forward, -1 = reverse, 0 = stopped
    private int state = 0;

    public Feeder(HardwareMap hardwareMap) {
        try {
            feederLeft = hardwareMap.get(DcMotorEx.class, "feederLeft");
            feederLeft.setDirection(DcMotorEx.Direction.REVERSE);
        } catch (Exception e) {
            feederLeft = null;
        }

        try {
            feederRight = hardwareMap.get(DcMotorEx.class, "feederRight");
            feederRight.setDirection(DcMotorEx.Direction.REVERSE);
        } catch (Exception e) {
            feederRight = null;
        }
    }

    public void resetEncoders() {
        if (feederLeft != null) {
            feederLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            feederLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        if (feederRight != null) {
            feederRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            feederRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        leftTarget = 0;
        rightTarget = 0;
        state = 0;
    }

    // Step forward (advance 180°)
    public void advanceOneStep() {
        leftTarget -= STEP_TICKS;
        rightTarget += STEP_TICKS;

        if (feederLeft != null) {
            feederLeft.setTargetPosition(leftTarget);
            feederLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            feederLeft.setPower(1.0);
        }
        if (feederRight != null) {
            feederRight.setTargetPosition(rightTarget);
            feederRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            feederRight.setPower(1.0);
        }

        state = 1;
    }

    // Step reverse (back 180°)
    public void reverseOneStep() {
        leftTarget += STEP_TICKS;
        rightTarget -= STEP_TICKS;

        if (feederLeft != null) {
            feederLeft.setTargetPosition(leftTarget);
            feederLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            feederLeft.setPower(1.0);
        }
        if (feederRight != null) {
            feederRight.setTargetPosition(rightTarget);
            feederRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            feederRight.setPower(1.0);
        }

        state = -1;
    }

    public void stop() {
        if (feederLeft != null) feederLeft.setPower(0.0);
        if (feederRight != null) feederRight.setPower(0.0);
        state = 0;
    }

    /** Return feeder state (1=forward, -1=reverse, 0=stopped) */
    public int getState() {
        return state;
    }

    // --- Encoder accessors for logging ---
    public int getLeftPosition() {
        return feederLeft != null ? feederLeft.getCurrentPosition() : 0;
    }

    public int getRightPosition() {
        return feederRight != null ? feederRight.getCurrentPosition() : 0;
    }

    public int getAveragePosition() {
        int left = feederLeft != null ? feederLeft.getCurrentPosition() : 0;
        int right = feederRight != null ? feederRight.getCurrentPosition() : 0;
        return (left + right) / 2;
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("=== FEEDER ===");
        if (feederLeft != null) {
            telemetry.addData("Left Pos (ticks)", feederLeft.getCurrentPosition());
            telemetry.addData("Left Target", leftTarget);
        } else {
            telemetry.addData("Left Motor", "Not Found");
        }
        if (feederRight != null) {
            telemetry.addData("Right Pos (ticks)", feederRight.getCurrentPosition());
            telemetry.addData("Right Target", rightTarget);
        } else {
            telemetry.addData("Right Motor", "Not Found");
        }

        telemetry.addData("Feeder State", state == 1 ? "Forward" :
                state == -1 ? "Reverse" : "Stopped");
    }
}