package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.subsystems.conv.Conv;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

public class UnifiedActions {
    private final Conv conv;
    private final Shooter shooter;
    private final Intake intake;


    public UnifiedActions(Conv conv, Shooter shooter, Intake intake) {
        this.conv = conv;
        this.shooter = shooter;
        this.intake = intake;
    }

    public Action shoot(){
        return new SequentialAction(
                shooter.spinUp(794),
                new SleepAction(0.1),
                new ParallelAction(intake.spinUp(),conv.load(0.08)),
                new SleepAction(3.5)
        );
    }
    public Action shoot(int vel){
        return new SequentialAction(
                shooter.spinUp(vel),
                new SleepAction(0.1),
                new ParallelAction(intake.spinUp(),conv.load(0.08)),
                new SleepAction(3.5)
        );
    }

    public Action UnLoadShooter(double unloadDuration){
        return new SequentialAction(
                shooter.unloadShooter(-1),
                new SleepAction(unloadDuration),
                shooter.stop()
        );
    }

    public Action stopShot(){
        return new ParallelAction(
                shooter.stop(),
                new ParallelAction(conv.stop(), intake.stop())
        );
    }
    public Action load(double power){
        return new ParallelAction(
                intake.spinUp(),
                conv.load(power)
        );
    }
    public Action unLoad(double x){
        return new SequentialAction(
                new ParallelAction(intake.spinDown(), conv.unLoad()),
                new SleepAction(x),
                intake.stop(),
                conv.stop()
                );
    }

    public Action stopLoad(){
        return new ParallelAction(
                intake.stop(),
                conv.stop()
        );
    }
}
