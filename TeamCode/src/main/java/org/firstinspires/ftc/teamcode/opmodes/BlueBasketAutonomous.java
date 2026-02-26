package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrive;

/**
 * Blue Basket Autonomous OpMode
 * 
 * This autonomous program:
 * 1. Drives to shooting position
 * 2. Turns towards the blue basket
 * 3. Shoots the game element
 * 4. Parks in the blue observation zone
 * 
 * DECODE Field: 144x144 inches, coordinates from -72 to +72
 * Blue Alliance is on the left side (negative X)
 * Blue Basket is in the corner at approximately (-72, 72)
 */

@Config
@Autonomous(name = "idk")
public class BlueBasketAutonomous extends LinearOpMode {

    // Starting position - Blue alliance starting zone
    public static double START_X = -12;
    public static double START_Y = -60;
    public static double START_HEADING = Math.toRadians(90);  // Facing towards positive Y

    // Shooting position - area with clear shot to basket
    public static double SHOOT_X = -36;
    public static double SHOOT_Y = -12;

    // Blue basket is at corner (-72, 72), we aim towards it
    // Angle from shooting position to basket: approximately 115 degrees
    public static double BASKET_HEADING = Math.toRadians(115);

    // Blue parking/observation zone
    public static double PARK_X = -48;
    public static double PARK_Y = -60;
    public static double PARK_HEADING = Math.toRadians(180);

    // Shooter settings
    public static double SHOOTER_POWER = 0.8;
    public static long SHOOTER_SPINUP_TIME_MS = 1500;  // Time to spin up shooter
    public static long SHOOTER_SHOOT_TIME_MS = 1000;   // Time to run sweeper/feeder
    public static double SWEEPER_POWER = 1.0;

    private DcMotorEx rightShooter;
    private DcMotorEx leftShooter;
    private DcMotor sweeper;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize telemetry with FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize drive system with starting pose
        Pose2d startPose = new Pose2d(START_X, START_Y, START_HEADING);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Initialize shooter motors
        try {
            rightShooter = hardwareMap.get(DcMotorEx.class, "right");
            leftShooter = hardwareMap.get(DcMotorEx.class, "left");
            sweeper = hardwareMap.get(DcMotor.class, "sweeper");

            // Configure shooter motors
            rightShooter.setDirection(DcMotor.Direction.FORWARD);
            leftShooter.setDirection(DcMotor.Direction.REVERSE);
            rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            leftShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            telemetry.addData("Warning", "Shooter motors not found, shooting will be skipped");
            telemetry.update();
        }

        // Build the trajectory
        TrajectoryActionBuilder trajectoryBuilder = drive.actionBuilder(startPose);

        // Step 1: Drive to shooting position (strafeTo for straight line movement)
        Action driveToShoot = trajectoryBuilder
                .strafeTo(new Vector2d(SHOOT_X, SHOOT_Y))
                .build();

        // Step 2: Turn to face blue basket
        TrajectoryActionBuilder turnBuilder = drive.actionBuilder(new Pose2d(SHOOT_X, SHOOT_Y, START_HEADING));
        Action turnToBasket = turnBuilder
                .turnTo(BASKET_HEADING)
                .build();

        // Step 3: Park in blue zone
        TrajectoryActionBuilder parkBuilder = drive.actionBuilder(new Pose2d(SHOOT_X, SHOOT_Y, BASKET_HEADING));
        Action driveToPark = parkBuilder
                .strafeTo(new Vector2d(PARK_X, PARK_Y))
                .turnTo(PARK_HEADING)
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Starting Position", "X: %.1f, Y: %.1f, Heading: %.1f°", 
                START_X, START_Y, Math.toDegrees(START_HEADING));
        telemetry.update();

        // Wait for start
        waitForStart();

        if (isStopRequested()) return;

        telemetry.addData("Status", "Running Autonomous");
        telemetry.update();

        // Execute Step 1: Drive to shooting position
        telemetry.addData("Step", "1 - Driving to shooting position");
        telemetry.update();
        Actions.runBlocking(driveToShoot);

        if (isStopRequested()) return;

        // Execute Step 2: Turn to face basket
        telemetry.addData("Step", "2 - Turning to basket");
        telemetry.update();
        Actions.runBlocking(turnToBasket);

        if (isStopRequested()) return;

        // Execute Step 3: Shoot
        telemetry.addData("Step", "3 - Shooting");
        telemetry.update();
        performShot();

        if (isStopRequested()) return;

        // Execute Step 4: Park
        telemetry.addData("Step", "4 - Parking");
        telemetry.update();
        Actions.runBlocking(driveToPark);

        telemetry.addData("Status", "Autonomous Complete!");
        telemetry.update();

        // Keep running until stopped
        while (opModeIsActive()) {
            sleep(100);
        }
    }

    /**
     * Performs the shooting sequence:
     * 1. Spin up shooter motors
     * 2. Run sweeper to feed game element
     * 3. Stop all shooter motors
     */
    private void performShot() {
        if (rightShooter == null || leftShooter == null) {
            telemetry.addData("Shot", "Skipped - motors not available");
            telemetry.update();
            sleep(500);  // Brief pause to simulate shooting
            return;
        }

        // Spin up shooter motors
        telemetry.addData("Shooter", "Spinning up...");
        telemetry.update();
        rightShooter.setPower(SHOOTER_POWER);
        leftShooter.setPower(SHOOTER_POWER);
        sleep(SHOOTER_SPINUP_TIME_MS);

        if (isStopRequested()) {
            stopShooter();
            return;
        }

        // Feed the game element
        telemetry.addData("Shooter", "Feeding...");
        telemetry.update();
        if (sweeper != null) {
            sweeper.setPower(SWEEPER_POWER);
        }
        sleep(SHOOTER_SHOOT_TIME_MS);

        // Stop everything
        stopShooter();
        telemetry.addData("Shooter", "Complete");
        telemetry.update();
    }

    /**
     * Stops all shooter motors safely
     */
    private void stopShooter() {
        if (rightShooter != null) rightShooter.setPower(0);
        if (leftShooter != null) leftShooter.setPower(0);
        if (sweeper != null) sweeper.setPower(0);
    }
}
