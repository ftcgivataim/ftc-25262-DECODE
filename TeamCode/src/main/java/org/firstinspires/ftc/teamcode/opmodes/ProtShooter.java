package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@TeleOp(name = "protShooter")
public class ProtShooter extends OpMode {

    public static double kP = 50;
    public static double kD = 5;
    public static double kI = 0;
    public static double kF = 16;

    public static double A_POWER = 0.5; //checked for shooting far.
    public static double X_POWER = 900.0; //in ticks per second
    public static double B_POWER = 1.0;
    public static double Y_POWER = 0.2;

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

        rightMotor = hardwareMap.get(DcMotorEx.class, "shooterRight");
        leftMotor = hardwareMap.get(DcMotorEx.class,"shooterLeft");
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

        PIDFCoefficients coefficients = new PIDFCoefficients(kP, kI, kD, kF);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);


        telemetry.addData("coef",rightMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));

//        telemetry.speak("It's a me, Mario! Here we go!");
        telemetry.update();
    }

    @Override
    public void loop() {
//        TelemetryPacket packet = new TelemetryPacket();

        double power;
        double sweepPower;
        double convpower;

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

        double leftRPM = leftMotor.getVelocity();
        double rightRPM = rightMotor.getVelocity();

        rightMotor.setVelocity(power);
        leftMotor.setVelocity(power);


        sweeperMotor.setPower(sweepPower);
        convMotor.setPower(convpower);


        telemetry.addData("Power", power);
        telemetry.addData("sweepPower", sweepPower);
        telemetry.addLine();
        telemetry.addData("Left RPM",leftRPM);
        telemetry.addData("Right RPM",rightRPM);

        telemetry.addData("coef",rightMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));

        telemetry.update();
    }
}
