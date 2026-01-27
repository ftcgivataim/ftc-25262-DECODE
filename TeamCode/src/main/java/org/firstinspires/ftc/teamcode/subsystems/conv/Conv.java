package org.firstinspires.ftc.teamcode.subsystems.conv;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
                convMotor.setDirection(DcMotor.Direction.REVERSE);
                convMotor.setPower(0.3);
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

    public class UnLoad implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                convMotor.setDirection(DcMotor.Direction.FORWARD);
                convMotor.setPower(0.3);
                initialized = true;
            }

            double vel = Math.abs(convMotor.getVelocity());
            packet.put("shooterVelocity", vel);
            return vel < 480.0;
        }
    }

    public Action unLoad(){
        return new UnLoad();
    }

    public class PrepareToShoot implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                convMotor.setDirection(DcMotor.Direction.FORWARD);
                convMotor.setPower(1);
                initialized = true;
            }

            telemetryPacket.put("Pushing back!", true);
            if (convMotor.getCurrentPosition() < convMotor.getTargetPosition() - 20) {
                convMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetryPacket.put("Pushed successfuly!", true);
                return false;
            }

            return false;
        }
    }

    public Action prepareToShoot(){
        return new PrepareToShoot();
    }

    public Action stop(){
        return new InstantAction(() -> convMotor.setPower(0));
    }
}
