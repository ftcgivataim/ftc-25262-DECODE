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

import org.firstinspires.ftc.teamcode.Stats;
import org.firstinspires.ftc.teamcode.sensors.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.UnifiedActions;
import org.firstinspires.ftc.teamcode.subsystems.conv.Conv;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

@Config
@Autonomous(name = "Red Wall Auto")
public class RedWallAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(63, 9, Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        Conv conv = new Conv(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, odo.getPosition());

        UnifiedActions unifiedActions = new UnifiedActions(conv, shooter, intake);


        TrajectoryActionBuilder driveToShootingPose = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-18,18), Math.atan2(72-18,18-72));

        TrajectoryActionBuilder driveToCollect = driveToShootingPose.fresh().
                strafeToLinearHeading(new Vector2d(-12,40), Math.PI/2 + 0.13);

        TrajectoryActionBuilder driveToSample2 = driveToCollect.fresh().
                strafeTo(new Vector2d(-12,56));

        TrajectoryActionBuilder driveToScore = driveToSample2.fresh()
                .strafeToLinearHeading(new Vector2d(-12,24), Math.PI/2)
                .strafeToLinearHeading(new Vector2d(-18,18), Math.atan2(72-18,18-72));

        TrajectoryActionBuilder driveToSubmersible = driveToScore.fresh().
                strafeToLinearHeading(new Vector2d(32,50), Math.PI);

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
                unifiedActions.shoot(),
                conv.stop()
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

        Action park = driveToSubmersible.build();


        Action fullAutoRoutine = new SequentialAction(
                scorePreload,
                prepareShotAndMove,
                scoreSample1,
                park
        );

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(fullAutoRoutine);
        Stats.currentPose = drive.localizer.getPose();



//        Action tot = new SequentialAction(
//                tab1.build(),
//                conv.unLoad(),
//                tab1.fresh().waitSeconds(0.5).build(),
//                conv.stop(),
//                unifiedActions.shoot(),
//                unifiedActions.stopShot(),
//                tab2.build(),
//                unifiedActions.load(),
//                tab3.build(),
//                unifiedActions.stopLoad(),
//                tab4.build(),
//                unifiedActions.shoot(),
//                unifiedActions.stopShot(),
//                tabClose.build()
//        );
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        Actions.runBlocking(
//                tot
//        );


    }
}