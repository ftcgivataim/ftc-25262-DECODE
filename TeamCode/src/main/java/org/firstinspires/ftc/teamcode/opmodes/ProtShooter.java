package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name = "protShooter")
public class ProtShooter extends OpMode {

    static float A_POWER = 0.55f;
    static float X_POWER = 0.9f;
    static float B_POWER = 1f;

    DcMotor rightMotor;
    DcMotor leftMotor;
    DcMotor sweeperMotor;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rightMotor = hardwareMap.get(DcMotor.class, "right");
        leftMotor = hardwareMap.get(DcMotor.class,"left");
        sweeperMotor = hardwareMap.get(DcMotor.class,"sweeper");

        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);

        sweeperMotor.setDirection(DcMotor.Direction.FORWARD);


        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sweeperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    @Override
    public void loop() {
//        TelemetryPacket packet = new TelemetryPacket();

        float power;
        float sweepPower;

        if (gamepad1.a) {
            power = A_POWER;
        }
        else if (gamepad1.x) {
            power = X_POWER;
        }
        else if (gamepad1.right_trigger != 0)
            power = gamepad1.right_trigger;
        else
            power = 0;

        if(gamepad1.b)
        {
            sweepPower = B_POWER;
        }
        else
        {
            sweepPower = 0;
        }




        rightMotor.setPower(power);
        leftMotor.setPower(power);
        sweeperMotor.setPower(sweepPower);

        telemetry.addData("Power", power);
        telemetry.addData("sweepPower", sweepPower);
        telemetry.update();
    }
}
