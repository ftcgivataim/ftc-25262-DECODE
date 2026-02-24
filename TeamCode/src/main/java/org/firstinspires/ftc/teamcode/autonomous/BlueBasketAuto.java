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
@Autonomous(name = "Blue Basket Auto")
public class BlueBasketAuto extends LinearOpMode {

    // 1. Define Constants for Tuning (allows Dashboard use)
    public static double UNLOAD_TIME_SHORT = 0.1;
    public static double UNLOAD_TIME_LONG = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-54, -54, Math.toRadians(45));
        Vector2d shootingPose = new Vector2d(-18, -18);


        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

////        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        Conv conv = new Conv(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, Helpers.GOAL_POSE_BLUE);

        UnifiedActions unifiedActions = new UnifiedActions(conv, shooter, intake);

        VelConstraint baseVelConstraint = (robotPose, _path, _disp) -> 10.0;


        double goalHeading = Math.atan2(18 - 72, 18 - 72) + Math.PI;

        TrajectoryActionBuilder driveToShootingPose = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(shootingPose, goalHeading);

        TrajectoryActionBuilder driveToCollect = driveToShootingPose.fresh().
                strafeToLinearHeading(new Vector2d(-12,-40), Math.PI*3/2 - 0.13);

        TrajectoryActionBuilder  driveToSample2 = driveToCollect.fresh().
                strafeTo(new Vector2d(-12,-60), baseVelConstraint);

        TrajectoryActionBuilder  driveToScore =  driveToSample2.fresh()
                .strafeToLinearHeading(new Vector2d(24,-44), Math.PI*3/2)
                .strafeToLinearHeading(shootingPose, goalHeading);

        TrajectoryActionBuilder driveToSubmersible =  driveToScore.fresh().
                strafeToLinearHeading(new Vector2d(32,-50), 0);





        Action prepareShotAndMove = new ParallelAction(
                unifiedActions.stopShot(),
                driveToCollect.build(), // Renamed from tab2
                unifiedActions.load(),
                unifiedActions.UnLoadShooter(0.5)
        );

        Action spinUpAndMoveToScore = new ParallelAction(
                driveToScore.build(), // Renamed from tab4
                unifiedActions.unLoad(0.2),
                unifiedActions.UnLoadShooter(1)
        );

//// 3. Define Logical Phases
        Action scorePreload = new SequentialAction(
                driveToShootingPose.build(), // Renamed from tab1
                unifiedActions.shoot(),
                conv.stop()
        );

        Action scoreSample1 = new SequentialAction(
                new ParallelAction(
                        driveToSample2.build(), // Renamed from tab3
                        unifiedActions.UnLoadShooter(2.5)
                ),
                unifiedActions.stopLoad(),
                spinUpAndMoveToScore,
                conv.stop(),
                unifiedActions.UnLoadShooter(0.2),
                unifiedActions.shoot(),
                unifiedActions.stopShot()
        );

        Action park = driveToSubmersible.build();

//// 4. The Final Sequence is now readable like English
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

//
//        Action tot = new SequentialAction(
//                driveToShootingPose.build(),
//                unifiedActions.unLoad(0.1),
//                conv.stop(),
//                unifiedActions.shoot(),
//                new ParallelAction(unifiedActions.stopShot(), driveToCollect.build(),unifiedActions.load()),
//                driveToSample2.build(),
//                unifiedActions.stopLoad(),
//                new ParallelAction( driveToScore.build(), shooter.spinUp(),unifiedActions.unLoad(0.3)),
//                conv.stop(),
//                unifiedActions.shoot(),
//                unifiedActions.stopShot(),
//                driveToSubmersible.build()
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
