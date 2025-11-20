package org.firstinspires.ftc.teamcode.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Feeder {
    private DcMotorEx feederMotor;

    public Feeder(HardwareMap hardwareMap) {
        feederMotor = hardwareMap.get(DcMotorEx.class, "feeder");
    }

    public void feedForward() {
        feederMotor.setPower(1.0);
    }

    public void feedReverse() {
        feederMotor.setPower(-1.0);
    }

    public void stop() {
        feederMotor.setPower(0.0);
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("=== FEEDER ===")
                .addData("Motor Position", feederMotor.getCurrentPosition())
                .addData("Motor Power", feederMotor.getPower());
    }
}
