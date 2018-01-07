package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.MotorType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import java.util.LinkedList;

/**
 * Created by akanksha.joshi on 24-Dec-2017.
 */
@Autonomous(name="Glyph Maneuver Test", group="Autonomous")
public class Testing_GlyphManeuver extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        robotPath();

        Task_GlyphManeuver glyphManeuver = new Task_GlyphManeuver(hardwareMap);
        glyphManeuver.Init();
        while(glyphManeuver.GetTaskStatus() == false){
            glyphManeuver.PerformTask(telemetry, getRuntime());
            sleep(100);
        }

    }
    private void robotPath(){
        DriveInstructionsHelper firstAction = new DriveInstructionsHelper(OPModeConstants.DriveInstructions.FORWARD, 24.0d);
        DriveInstructionsHelper secondAction = new DriveInstructionsHelper(OPModeConstants.DriveInstructions.REVERSE, 24.0d);
        DriveInstructionsHelper thirdAction = new DriveInstructionsHelper(OPModeConstants.DriveInstructions.TURN, 90d);
        DriveInstructionsHelper fourthAction = new DriveInstructionsHelper(OPModeConstants.DriveInstructions.TURN, -90d);
        DriveInstructionsHelper fifthAction = new DriveInstructionsHelper(OPModeConstants.DriveInstructions.FORWARD, 6.0d);
        LinkedList initPair = new LinkedList<DriveInstructionsHelper>();
        initPair.add(firstAction);
        initPair.add(secondAction);
        initPair.add(thirdAction);
        initPair.add(fourthAction);
        initPair.add(fifthAction);
        OPModeConstants.getInstance().setDrivePath(initPair);
    }
}
