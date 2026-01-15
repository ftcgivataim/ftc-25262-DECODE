package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "protShooter")
public class ProtShooter extends OpMode {

    public static float A_POWER = 0.55f;
    public static float X_POWER = 0.9f;
    public static float B_POWER = 1f;
    public static float Y_POWER = 1f;
    public static boolean inverseRight = false;
    public static boolean inverseLeft = true;
    public static boolean inverseConv = true;
    public static boolean inverseSweeper = true;


    DcMotorEx rightMotor;
    DcMotorEx leftMotor;
    DcMotorEx convMotor;
    DcMotor sweeperMotor;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rightMotor = hardwareMap.get(DcMotorEx.class, "right");
        leftMotor = hardwareMap.get(DcMotorEx.class,"left");
        sweeperMotor = hardwareMap.get(DcMotor.class,"sweeper");
        convMotor = hardwareMap.get(DcMotorEx.class,"conv");

        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        sweeperMotor.setDirection(DcMotor.Direction.FORWARD);
        convMotor.setDirection(DcMotor.Direction.FORWARD);

        if (inverseLeft){
            leftMotor.setDirection(DcMotor.Direction.REVERSE);
        }
        if (inverseRight){
            rightMotor.setDirection(DcMotor.Direction.REVERSE);

        }
        if (inverseConv){
            convMotor.setDirection(DcMotor.Direction.REVERSE);
        }
        if (inverseSweeper){
            sweeperMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }


        rightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sweeperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        convMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

//        telemetry.speak("It's a me, Mario! Here we go!");
//        telemetry.update();
    }

    @Override
    public void loop() {
//        TelemetryPacket packet = new TelemetryPacket();

        float power;
        float sweepPower;
        float convpower;

        //shooter power. or A or X or right trigger.
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

        //sweeper power. only B.
        if(gamepad1.b) {
            sweepPower = B_POWER;
        }
        else {
            sweepPower = 0;
        }

        //conv power. only Y.
        if(gamepad1.y) {
            convpower = Y_POWER;
        }
        else {
            convpower = 0;
        }


        rightMotor.setPower(power);
        leftMotor.setPower(power);
        sweeperMotor.setPower(sweepPower);
        convMotor.setPower(convpower);

        double leftRPM = leftMotor.getVelocity();
        double rightRPM = rightMotor.getVelocity();


        telemetry.addData("Power", power);
        telemetry.addData("sweepPower", sweepPower);
        telemetry.addLine();
        telemetry.addData("Left RPM",leftRPM);
        telemetry.addData("Right RPM",rightRPM);


        telemetry.update();
    }
}
