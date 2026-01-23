package org.firstinspires.ftc.teamcode.subsystems.conv;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Conv {

    private final DcMotorEx convMotor;

    public Conv(HardwareMap hardwareMap) {
        convMotor = hardwareMap.get(DcMotorEx.class, "conv");
        convMotor.setDirection(DcMotorEx.Direction.REVERSE);

    }

    public class Load implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                convMotor.setPower(0.5);
                initialized = true;
            }

            double vel = convMotor.getVelocity();
            packet.put("shooterVelocity", vel);
            return vel < 480.0;
        }
    }

    public Action load(){
        return new Load();
    }

    public class PrepareToShoot implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                convMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                convMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                convMotor.setPower(0);
                convMotor.setTargetPosition(-100);
                convMotor.setPower(0.1);
                initialized = true;
            }

            return convMotor.isBusy();
        }
    }

    public Action prepareToShoot(){
        return new PrepareToShoot();
    }

    public Action stop(){
        return new InstantAction(() -> convMotor.setPower(0));
    }
}
