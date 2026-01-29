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
                shooter.spinUp(),
                conv.load(),
                new SleepAction(5.0)
        );
    }

    public Action stopShot(){
        return new ParallelAction(
                shooter.stop(),
                conv.stop()
        );
    }
    public Action load(){
        return new ParallelAction(
                intake.spinUp(),
                conv.load()
        );
    }
    public Action unLoad(){
        return new SequentialAction(
                conv.unLoad(),
                new SleepAction(0.5)
                );
    }

    public Action stopLoad(){
        return new ParallelAction(
                intake.stop(),
                conv.stop()
        );
    }
}
