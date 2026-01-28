package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

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
                conv.load()
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
        return conv.unLoad();
    }

    public Action stopLoad(){
        return new ParallelAction(
                intake.stop(),
                conv.stop()
        );
    }
}
