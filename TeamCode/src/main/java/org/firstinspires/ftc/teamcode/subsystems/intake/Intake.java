package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final DcMotorEx intakeMotor;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "sweeper");
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }

//    public class SpinUp implements Action {
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            convMotor.setPower(1);
//            return false;
//        }
//    }

    public Action spinUp(){
        return new InstantAction(() -> intakeMotor.setPower(1));
    }

    public Action spinDown(){
        return new InstantAction(() -> intakeMotor.setPower(-0.2));
    }

//    public class Stop implements Action {
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            convMotor.setPower(0);
//            return false;
//        }
//    }

    public Action stop(){
        return new InstantAction(() -> intakeMotor.setPower(0));
    }

}
