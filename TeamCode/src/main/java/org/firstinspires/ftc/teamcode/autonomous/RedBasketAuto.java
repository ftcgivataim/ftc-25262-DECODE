package org.firstinspires.ftc.teamcode.autonomous;

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
@Autonomous(name = "Red Basket Auto")
public class RedBasketAuto extends LinearOpMode {

    // 1. Define Constants for Tuning (allows Dashboard use)
    public static double UNLOAD_TIME_SHORT = 0.1;
    public static double UNLOAD_TIME_LONG = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-56, 56, Math.toRadians(315));
        Vector2d shootingPose = new Vector2d(-24, 34);


        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        Conv conv = new Conv(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, Helpers.GOAL_POSE_BLUE);

        UnifiedActions unifiedActions = new UnifiedActions(conv, shooter, intake);



        double goalHeading = Math.atan2(-34 + 72, 24 - 72) + Math.PI;

        TrajectoryActionBuilder driveToShootingPose = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(shootingPose, goalHeading);

        TrajectoryActionBuilder driveToCollect = driveToShootingPose.fresh().
                strafeToLinearHeading(new Vector2d(-3,30), Math.PI/2 + 0.13);

        TrajectoryActionBuilder  driveToSample2 = driveToCollect.fresh().
                strafeTo(new Vector2d(-3,60));

        TrajectoryActionBuilder  driveToScore =  driveToSample2.fresh()
                .strafeToLinearHeading(new Vector2d(-3,44), Math.PI/2)
                .strafeToLinearHeading(shootingPose, goalHeading);

        TrajectoryActionBuilder driveToSubmersible =  driveToScore.fresh().
                waitSeconds(7).
                strafeToLinearHeading(new Vector2d(32,50), 0);



        Action prepareShotAndMove = new ParallelAction(
                unifiedActions.stopShot(),
                unifiedActions.UnLoadShooter(0.2),
                driveToCollect.build(),
                unifiedActions.load(0.08)

        );

        Action spinUpAndMoveToScore = new ParallelAction(
                driveToScore.build(),
                unifiedActions.unLoad(0.2),
                unifiedActions.UnLoadShooter(1)
        );


        Action scorePreload = new SequentialAction(
                driveToShootingPose.build(),
                unifiedActions.shoot(),
                unifiedActions.stopShot()
        );

        Action scoreSample1 = new SequentialAction(
                driveToSample2.build(),
                unifiedActions.stopLoad(),
                spinUpAndMoveToScore,
                conv.stop(),
                new ParallelAction(
                        unifiedActions.UnLoadShooter(0.2),
                        unifiedActions.unLoad(1)
                ),
                unifiedActions.shoot(),
                unifiedActions.stopShot()
        );

        Action park = driveToSubmersible.build();


        Action fullAutoRoutine = new SequentialAction(
                scorePreload,
                prepareShotAndMove,
                scoreSample1
        );

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(fullAutoRoutine);
        Stats.currentPose = drive.localizer.getPose();



    }
}
