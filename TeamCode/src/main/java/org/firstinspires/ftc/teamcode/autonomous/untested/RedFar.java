package org.firstinspires.ftc.teamcode.autonomous.untested;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Helpers;
import org.firstinspires.ftc.teamcode.Stats;
import org.firstinspires.ftc.teamcode.subsystems.UnifiedActions;
import org.firstinspires.ftc.teamcode.subsystems.conv.Conv;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

@Config
@Autonomous(name = "Red Far Auto")
public class RedFar extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(63, 9, Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

//        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        Conv conv = new Conv(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, Helpers.GOAL_POSE_RED);

        UnifiedActions unifiedActions = new UnifiedActions(conv, shooter, intake);

        VelConstraint baseVelConstraint = (robotPose, _path, _disp) -> 10.0;


        double goalHeading = Math.atan2(72 - 52, 9 - 72) + Math.PI;

        TrajectoryActionBuilder driveToShootingPose = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(52, -9), goalHeading);

        TrajectoryActionBuilder driveToCollect = driveToShootingPose.fresh().
                strafeTo(new Vector2d(56, 56));

        TrajectoryActionBuilder driveToSample2 = driveToCollect.fresh().
                strafeTo(new Vector2d(32, -60), baseVelConstraint);

        TrajectoryActionBuilder driveToScore = driveToSample2.fresh()
                .strafeToLinearHeading(new Vector2d(32, -40), Math.PI * 3 / 2)
                .strafeToLinearHeading(new Vector2d(60, -9), goalHeading);


        Action prepareShotAndMove = new ParallelAction(
                unifiedActions.stopShot(),
                driveToCollect.build(),
                unifiedActions.load(0.08)
        );

        Action spinUpAndMoveToScore = new ParallelAction(
                driveToScore.build(),
                unifiedActions.unLoad(0.2)
        );


        Action scorePreload = new SequentialAction(
                driveToShootingPose.build(),
                unifiedActions.shoot(925),
                unifiedActions.stopShot()
        );

        Action scoreSample1 = new SequentialAction(
                driveToSample2.build(),
                unifiedActions.stopLoad(),
                spinUpAndMoveToScore,
                conv.stop(),
                unifiedActions.UnLoadShooter(0.2),
                unifiedActions.shoot(),
                unifiedActions.stopShot()
        );


        Action fullAutoRoutine = new SequentialAction(
                scorePreload,
                driveToCollect.build()
        );

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(fullAutoRoutine);
        Stats.currentPose = drive.localizer.getPose();

    }
}
