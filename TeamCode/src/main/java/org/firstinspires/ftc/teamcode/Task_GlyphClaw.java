package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by harsh.joshi on 31-Dec-2017.
 */

public class Task_GlyphClaw extends IOPModeTaskBase {
    private boolean taskSatisfied = false;
    private OPModeConstants opModeConstants = null;
    private HardwareMap hardwareMap;
    private OPModeConstants.GlyphClawPosition clawPosition;

    public Task_GlyphClaw(HardwareMap hardwareMap, OPModeConstants.GlyphClawPosition clawPosition)
    {
        this.hardwareMap = hardwareMap;
        this.clawPosition = clawPosition;
        taskSatisfied = false;
    }
    @Override
    public boolean GetTaskStatus() {
        return taskSatisfied;
    }

    @Override
    public void PerformTask(Telemetry telemetry, double elapsedTime) {
        if (elapsedTime > OPModeConstants.ReleaseGlyph) {
            taskSatisfied = true;
            return;
        }
        Servo leftClaw = hardwareMap.servo.get("left_claw");
        Servo rightClaw = hardwareMap.servo.get("right_claw");
        if(opModeConstants.getGlyphClawPosition() == OPModeConstants.GlyphClawPosition.OPEN)
        {
            leftClaw.setPosition(0.35);
            rightClaw.setPosition(0.65);
        }
        if(opModeConstants.getGlyphClawPosition() == OPModeConstants.GlyphClawPosition.CLOSE)
        {
            leftClaw.setPosition(0.65);
            rightClaw.setPosition(0.35);
        }
        if(clawPosition != OPModeConstants.GlyphClawPosition.UNKNOWN)
        {
            taskSatisfied = true;
        }
    }

    @Override
    public void Init() {
        opModeConstants = OPModeConstants.getInstance();
    }

    @Override
    public void Reset() {
        taskSatisfied = false;
    }
}
