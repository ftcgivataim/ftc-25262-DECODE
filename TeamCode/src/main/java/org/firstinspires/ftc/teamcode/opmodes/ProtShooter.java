package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

@Config
@TeleOp(name = "protShooter")
public class ProtShooter extends OpMode {

    public static float A_POWER = 0.55f;
    public static float shootPower;
    public static boolean inverseRight = false;
    public static boolean inverseLeft = true;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    DcMotorEx rightMotor;
    DcMotorEx leftMotor;
    DcMotor sweeperMotor;
    DcMotorEx frontLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx backRightMotor;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rightMotor = hardwareMap.get(DcMotorEx.class, "right");
        leftMotor = hardwareMap.get(DcMotorEx.class,"left");
        sweeperMotor = hardwareMap.get(DcMotor.class,"sweeper");

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeftMotor  = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");

        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);


        if (inverseLeft){
            leftMotor.setDirection(DcMotor.Direction.REVERSE);
        }
        if (inverseRight){
            rightMotor.setDirection(DcMotor.Direction.REVERSE);

        }

        sweeperMotor.setDirection(DcMotor.Direction.FORWARD);


        rightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sweeperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        telemetry.speak("It's a me, Mario! Here we go!");
//        telemetry.update();

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.update();


    }

    @Override
    public void loop() {
//        TelemetryPacket packet = new TelemetryPacket();

        float power;
        float sweepPower;

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double frontLeftPower = y + x + rx;
        double backLeftPower = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower = y + x - rx;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }

        if (gamepad1.right_trigger != 0) {
            power = gamepad1.right_trigger;
        }
        else
            power = 0;

        if(gamepad1.left_trigger != 0)
        {
            sweepPower = gamepad1.left_trigger;
        }
        else
        {
            sweepPower = 0;
        }





        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        rightMotor.setPower(power);
        leftMotor.setPower(power);
        sweeperMotor.setPower(sweepPower);

        double leftRPM = leftMotor.getVelocity();
        double rightRPM = rightMotor.getVelocity();

        Pose2D pos = odo.getPosition();


        telemetry.addData("Power", power);
        telemetry.addData("sweepPower", sweepPower);
        telemetry.addLine();
        telemetry.addData("Left RPM",leftRPM);
        telemetry.addData("Right RPM",rightRPM);

        telemetry.addData("Status", "Running");
        telemetry.addData("Front Left Power", frontLeftPower);
        telemetry.addData("Back Left Power", backLeftPower);
        telemetry.addData("Front Right Power", frontRightPower);
        telemetry.addData("Back Right Power", backRightPower);

        telemetry.update();
    } 
}
