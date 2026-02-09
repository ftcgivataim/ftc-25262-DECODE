package org.firstinspires.ftc.teamcode.subsystems.shooter;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.sensors.GoBildaPinpointDriver;

public class Shooter {
    private final DcMotorEx right;
    private final DcMotorEx left;
//    private final GoBildaPinpointDriver odo;

    private final Pose2D goalPose;

    public Shooter(HardwareMap hardwareMap, Pose2D goalPose) {
        right = hardwareMap.get(DcMotorEx.class, "shooterRight");
        left = hardwareMap.get(DcMotorEx.class, "shooterLeft");

        right.setDirection(DcMotorEx.Direction.FORWARD);
        left.setDirection(DcMotorEx.Direction.REVERSE);

        left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients coefficients = new PIDFCoefficients(50.0,0.0,5.0,16.0);

        right.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,coefficients);
        left.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,coefficients);

//        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        this.goalPose = goalPose;

    }

    private static double calcDist(Pose2D pose) {
        double x = pose.getX(DistanceUnit.CM);
        double y = pose.getY(DistanceUnit.CM);
        return 0;
    }

    private static double getVelTPS(Pose2D pose) {
        return 900;
    }



    public class SpinUp implements Action {
        double velTPS = 900;
        boolean initialized = false;


        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
//            odo.update();
//            Pose2D pose = odo.getPosition();
//            double velTPS = Shooter.getVelTPS(pose);

            if (!initialized) {
                right.setVelocity(velTPS);
                left.setVelocity(velTPS);
                initialized = true;
            }

            double velLeft = left.getVelocity();
            double velRight = right.getVelocity();
            packet.put("leftVel", velLeft);
            packet.put("rightVel", velRight);

            if (right.isOverCurrent() || left.isOverCurrent()){
                packet.put("OverCurrent!!!", "FUCK");
                left.setPower(0);
                right.setPower(0);
                return false;
            }

            return velLeft < velTPS || velRight < velTPS;
        }


    }

//    public class SpinUpSecondStage implements Action {
//
//        private final GoBildaPinpointDriver odo;
//
//        public SpinUpSecondStage(GoBildaPinpointDriver odo) {
//            this.odo = odo;
//        }
//
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            odo.update();
//            Pose2D pose = odo.getPosition();
//            double velTPS = Shooter.getVelTPS(pose);
//
//            left.setVelocity(velTPS);
//            right.setVelocity(velTPS);
//
//            double velLeft = left.getVelocity();
//            double velRight = right.getVelocity();
//            packet.put("leftVel", velLeft);
//            packet.put("rightVel", velRight);
//            return velLeft != velTPS && velRight != velTPS;
//        }
//
//
//    }


    public Action spinUp() {
        return new SpinUp();
    }

//    public Action spinUpSecondStage() {
//        return new SpinUp(odo);
//    }

//    public class Stop implements Action {
//
//        private boolean initialized = false;
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if (!initialized) {
//                right.setPower(0);
//                left.setPower(0);
//                initialized = true;
//            }
//
//
//            double velLeft = left.getVelocity();
//            double velRight = right.getVelocity();
//            telemetryPacket.put("leftVel", velLeft);
//            telemetryPacket.put("rightVel", velRight);
//            return velLeft != 0 || velRight != 0;
//        }
//    }


    public Action stop() {
        return new InstantAction(() -> {right.setPower(0);left.setPower(0);});
    }


}
