package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.MotorType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

/**
 * Created by harsh.joshi on 24-Dec-2017.
 */
@Autonomous(name="Glyph Claw Test", group="Autonomous")
public class Testing_GlyphClaw extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OPModeConstants opModeConstants = OPModeConstants.getInstance();
        Task_GlyphClaw closeClaw = new Task_GlyphClaw(hardwareMap, OPModeConstants.GlyphClawPosition.CLOSE);
        closeClaw.Init();
        closeClaw.PerformTask(telemetry,getRuntime());
        closeClaw.Reset();
        sleep(2000);
        Task_GlyphClaw openClaw = new Task_GlyphClaw(hardwareMap, OPModeConstants.GlyphClawPosition.OPEN);
        openClaw.Init();
        openClaw.PerformTask(telemetry,getRuntime());
        openClaw.Reset();
        waitForStart();


    }
}
