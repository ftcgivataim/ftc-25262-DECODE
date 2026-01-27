package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Helpers;
import org.firstinspires.ftc.teamcode.sensors.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.UnifiedActions;
import org.firstinspires.ftc.teamcode.subsystems.conv.Conv;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;


@Config
@TeleOp(name = "PrimaryOpMode")
public class PrimaryOpMode extends LinearOpMode {

    private final FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    public static class Params {
        public double speedMult = 0.7;
        public double turnMult = 1;

        public double backMotorMult = 1;
        public double frontMotorMult = 1;

        public double shooterSpeed = 900.0;

        public double kP = 1;
        public double kI = 0;
        public double kD = 1;

        public boolean isBlue = true;

        public Pose2D startingPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);


    }

    public static Params PARAMS = new Params();

    public enum IntakeState {
        STOPPED,
        LOADING,
        UNLOADING
    }

    // Create a variable to track the current state

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        TelemetryPacket packet = new TelemetryPacket();


        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        DcMotorEx rightMotor = hardwareMap.get(DcMotorEx.class, "shooterRight");
        DcMotorEx leftMotor = hardwareMap.get(DcMotorEx.class,"shooterLeft");

        // Reset the motor encoder so that it reads zero ticks
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //For manual control without Actions

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //shooter motor directions
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        //stop shooter motors if they are not used
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Retrieve the IMU from the hardware map
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();
        odo.setPosition(PARAMS.startingPose);
        odo.update();

        PIDController pid = new PIDController(PARAMS.kP, PARAMS.kI, PARAMS.kD);
        pid.setSetPoint(Helpers.getAngleToBasket(odo.getPosition(), PARAMS.isBlue));

        Conv conv = new Conv(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, odo.getPosition());

        UnifiedActions unifiedActions = new UnifiedActions(conv, shooter, intake);


        boolean goalLock = false;

        double prevAngle = 0;

        int counter = 0;

        IntakeState currentIntakeState = IntakeState.STOPPED;

        Action activeIntakeAction = null;
        // Track the active shooter action so we can check if it is running
        Action activeShooterAction = null;


        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {
            counter++;

            /* ##################################################
                            Inputs and Initializing
               ################################################## */

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            goalLock = gamepad1.b ^ goalLock;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.


            /* ##################################################
                        Movement Controls Calculations
               ################################################## */

            odo.update();
            Pose2D pose = odo.getPosition();

            double goalAngle = Helpers.getAngleToBasket(pose, PARAMS.isBlue);


            double botHeading = pose.getHeading(AngleUnit.RADIANS);

            if (botHeading < 0){
                botHeading += 2*Math.PI;
            }

            double error = calcError(botHeading, goalAngle,telemetry); // Use unwrapping here


            double rotX = x * Math.cos(botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(botHeading);

            rotX *= PARAMS.speedMult;
            rotY *= PARAMS.speedMult;

            if (goalLock) {
                rx = pid.calculate(0,error);
            } else {
                rx *= PARAMS.turnMult;
            }


            //intake
            /*if (gamepad1.a) {
                if (!intakeMode) {
                    runningActions.add(new ParallelAction(
                            intake.spinUp(),
                            conv.load()
                    ));
                    intakeMode = true;
                } else {
                    runningActions.add(new ParallelAction(
                            intake.stop(),
                            conv.stop()
                    ));
                    intakeMode = false;
                }
               }*/



            if (gamepad1.left_trigger != 0) {
                PARAMS.speedMult = 1;
            }
            else if (gamepad1.left_bumper) {
                PARAMS.speedMult = 0.5;
            }
            else {
                PARAMS.speedMult = 0.7;
            }

            /* ##################################################
                           SHOOTER LOGIC
               ################################################## */

            // Check if Trigger is pressed
            if (gamepad1.right_trigger > 0.1) {
                // Check if we are ALREADY shooting.
                // We do this by checking if our tracker variable is still in the running list.
                boolean isAlreadyShooting = activeShooterAction != null && runningActions.contains(activeShooterAction);

                if (!isAlreadyShooting) {
                    // Create a FRESH action
                    activeShooterAction = unifiedActions.shoot();
                    runningActions.add(activeShooterAction);
                }
            }

            // Check if Y is pressed (Emergency Stop / Reset)
            if (gamepad1.y) {
                // 1. Cancel the shooting action if it's currently running
                if (activeShooterAction != null) {
                    runningActions.remove(activeShooterAction);
                    activeShooterAction = null; // clear the tracker
                }

                // 2. Add the stop command
                // We don't need to track this one because it's usually instant
                runningActions.add(unifiedActions.stopShot());
            }


            /* ##################################################
                           INTAKE CONTROL LOGIC
               ################################################## */
            boolean isPressingUnload = gamepad1.x;
            boolean isPressingLoad   = gamepad1.a;

            // 1. UNLOAD
            if (isPressingUnload) {
                if (currentIntakeState != IntakeState.UNLOADING) {
                    // Stop previous action if it exists
                    if (activeIntakeAction != null) {
                        runningActions.remove(activeIntakeAction);
                    }
                    runningActions.add(unifiedActions.stopLoad()); // Safety stop

                    // Create NEW action and store it
                    activeIntakeAction = unifiedActions.unLoad();
                    runningActions.add(activeIntakeAction);

                    currentIntakeState = IntakeState.UNLOADING;
                }
            }

            // 2. LOAD
            else if (isPressingLoad) {
                if (currentIntakeState != IntakeState.LOADING) {
                    if (activeIntakeAction != null) {
                        runningActions.remove(activeIntakeAction);
                    }
                    runningActions.add(unifiedActions.stopLoad());

                    // Create NEW action and store it
                    activeIntakeAction = unifiedActions.load();
                    runningActions.add(activeIntakeAction);

                    currentIntakeState = IntakeState.LOADING;
                }
            }

            // 3. STOP
            else {
                if (currentIntakeState != IntakeState.STOPPED) {
                    // Remove the specifically running action
                    if (activeIntakeAction != null) {
                        runningActions.remove(activeIntakeAction);
                    }

                    // Run the stop action
                    runningActions.add(unifiedActions.stopLoad());

                    currentIntakeState = IntakeState.STOPPED;
                    activeIntakeAction = null;
                }
            }


            /* BINDINGS SO FAR - SHOW DRIVERS
            - frontarm forwards/backwards: right/left triggers - gamepad 1
            - sweeper action: a - gamepad 1
            - rotatorarm up/down: y/a - gamepad 1
            - slide up/down: y/a - gamepad 2
            - claw open/close: b/x gamepad 2
            - elevator up/down: right/left bumpers: gamepad 2
             */

             /* ######################################################
                   Runs Autonomous Actions in TeleOp - TODO: Enable
                ###################################################### */

//           // update running actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);

            /* ##################################################
                     Applying the Calculations to the Motors
               ################################################## */

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = PARAMS.frontMotorMult * (rotY + rotX + rx) / denominator;
            double backLeftPower = PARAMS.backMotorMult * (rotY - rotX + rx) / denominator;
            double frontRightPower = PARAMS.frontMotorMult * (rotY - rotX - rx) / denominator;
            double backRightPower = PARAMS.backMotorMult * (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


            /* ##################################################
                             TELEMETRY ADDITIONS
               ################################################## */


            telemetry.addData("Pose", odo.getPosition());
            telemetry.addData("botHeading", Math.toDegrees(botHeading));
            telemetry.addData("goalAngle", Math.toDegrees(Helpers.getAngleToBasket(odo.getPosition(), PARAMS.isBlue)));
            telemetry.addData("prevAngle", Math.toDegrees(prevAngle));
            telemetry.addData("error", error);
            telemetry.addData("shooterSpeed", PARAMS.shooterSpeed);
            telemetry.addData("rightSpeed", rightMotor.getVelocity());
            telemetry.addData("leftSpeed", leftMotor.getVelocity());
            telemetry.addData("counter", counter);
            telemetry.addData("len:", runningActions.size());
            telemetry.addData("intakeMode", currentIntakeState.toString());




            telemetry.update();

            prevAngle = botHeading;

        }


    }

    private double calcError(double current, double target, Telemetry t) {
        double eDistRight = (current - target) % (2 * Math.PI);
        double eDistLeft = (target - current) % (2 * Math.PI);
        if (eDistRight > eDistLeft) {
            t.addData("left", eDistLeft);
            return -eDistLeft;
        }

        t.addData("right", eDistRight);
        return eDistRight;
    }

    private void prepareToShoot() {

    }

}