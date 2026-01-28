package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrive;

@Config
@Autonomous(name = "Learning Auto")

public class LearningAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        //Claw claw = new Claw(hardwareMap);
        //LiftActions lift = new LiftActions(hardwareMap);

        //TODO - Build Movement Trajectory

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(20, 0))
                .strafeTo(new Vector2d(0, 20))
                .strafeTo(new Vector2d(-20, 0))
                .strafeTo(new Vector2d(0, -20))
                .waitSeconds(2);

        Action tab1Action = tab1.build();



        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Starting Position", initialPose);
            telemetry.update();
        }

        // Add Abillty to chose certain trajectory based on sensor input - Not needed right now
        Action trajectoryActionChosen;
        trajectoryActionChosen = tab1.build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                tab1Action
        );
    }
}
