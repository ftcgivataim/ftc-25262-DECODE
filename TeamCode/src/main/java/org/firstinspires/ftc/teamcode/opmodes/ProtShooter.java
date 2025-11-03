package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "protShooter")
public class ProtShooter extends OpMode {

    static float A_POWER = 0.75f;
    static float X_POWER = 0.9f;

    DcMotor rightMotor;
    DcMotor leftMotor;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rightMotor = hardwareMap.get(DcMotor.class, "right");
        leftMotor = hardwareMap.get(DcMotor.class,"left");

        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        float power;

        if (gamepad1.a) {
            power = A_POWER;
        } else if (gamepad1.x) {
            power = X_POWER;
        } else if (gamepad1.right_trigger != 0)
            power = gamepad1.right_trigger;
        else
            power = 0;

        rightMotor.setPower(power);
        leftMotor.setPower(power);

        telemetry.addData("Power", power);
        telemetry.update();
    }
}
