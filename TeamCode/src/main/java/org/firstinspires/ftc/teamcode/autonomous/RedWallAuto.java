package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.sensors.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.UnifiedActions;
import org.firstinspires.ftc.teamcode.subsystems.conv.Conv;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

@Config
@Autonomous(name = "Red Auto")
public class RedWallAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(63, -9, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        Conv conv = new Conv(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, odo.getPosition());

        UnifiedActions unifiedActions = new UnifiedActions(conv, shooter, intake);

        VelConstraint baseVelConstraint = (robotPose, _path, _disp) -> 10.0;


        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-18,18), Math.atan2(72-18,18-72));
        TrajectoryActionBuilder tab2 = tab1.fresh().
                strafeToLinearHeading(new Vector2d(36,24), Math.PI*3/2);
        TrajectoryActionBuilder tab3 = tab2.fresh().
                strafeTo(new Vector2d(36,56), baseVelConstraint);
        TrajectoryActionBuilder tab4 = tab3.fresh()
                .strafeToLinearHeading(new Vector2d(36,24), Math.PI*3/2)
                .strafeToLinearHeading(new Vector2d(-18,18), Math.atan2(72-18,18-72));
        TrajectoryActionBuilder tabClose = tab4.fresh().
                strafeToLinearHeading(new Vector2d(36,32), Math.PI);

        Action tot = new SequentialAction(
                tab1.build(),
                conv.prepareToShoot(),
                unifiedActions.shoot(),
                unifiedActions.stopShot(),
                tab2.build(),
                unifiedActions.load(),
                tab3.build(),
                unifiedActions.stopLoad(),
                tab4.build(),
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