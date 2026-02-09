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
import org.firstinspires.ftc.teamcode.sensors.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.UnifiedActions;
import org.firstinspires.ftc.teamcode.subsystems.conv.Conv;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

@Config
@Autonomous(name = "Blue Basket Auto")
public class BlueBasketAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-54, -54, Math.toRadians(45));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

//        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        Conv conv = new Conv(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, Helpers.GOAL_POSE_BLUE);

        UnifiedActions unifiedActions = new UnifiedActions(conv, shooter, intake);

        VelConstraint baseVelConstraint = (robotPose, _path, _disp) -> 10.0;


        double goalHeading = Math.atan2(18 - 72, 18 - 72) + Math.PI;
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-18,-18), goalHeading);
        TrajectoryActionBuilder tab2 = tab1.fresh().
                strafeToLinearHeading(new Vector2d(24,-40), Math.PI*3/2 - 0.13);
        TrajectoryActionBuilder tab3 = tab2.fresh().
                strafeTo(new Vector2d(24,-72), baseVelConstraint);
        TrajectoryActionBuilder tab4 = tab3.fresh()
                .strafeToLinearHeading(new Vector2d(24,-44), Math.PI*3/2)
                .strafeToLinearHeading(new Vector2d(-18,-18), goalHeading);
        TrajectoryActionBuilder tabClose = tab4.fresh().
                strafeToLinearHeading(new Vector2d(32,-50), 0);

        Action tot = new SequentialAction(
                tab1.build(),
                unifiedActions.unLoad(0.1),
                conv.stop(),
                unifiedActions.shoot(),
                new ParallelAction(unifiedActions.stopShot(), tab2.build(),unifiedActions.load()),
                tab3.build(),
                unifiedActions.stopLoad(),
                new ParallelAction(tab4.build(), shooter.spinUp(),unifiedActions.unLoad(0.3)),
                conv.stop(),
                unifiedActions.shoot(),
                unifiedActions.stopShot(),
                tabClose.build()
        );

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                tot
        );


    }
}
