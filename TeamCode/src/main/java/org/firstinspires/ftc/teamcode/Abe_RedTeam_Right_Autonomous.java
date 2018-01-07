package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.io.IOException;
import java.util.LinkedList;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.OPModeConstants;


/**
 * Created by Akanksha.Joshi on 23-Dec-2017.
 */
@Autonomous(name="Red Team Right", group="Autonomous")
public class Abe_RedTeam_Right_Autonomous extends LinearOpMode{

    public OPModeConstants opModeConstants = null;
    public JewelDetectorFacade jewelDetectorFacade= null;
    public JewelDetector jewelDetector= null;
    private JewelDriveMode jewelDriveMode = null;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean jewelRemoved = false;


    @Override
    public void runOpMode() throws InterruptedException {
        Task_ResetAll resetAll = new Task_ResetAll(hardwareMap);
        resetAll.PerformTask(telemetry,0);
        opModeConstants = OPModeConstants.getInstance();
        telemetry.setAutoClear(false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        opModeConstants.setSelectedTeam(OPModeConstants.SelectedTeam.RED_TEAM);
        opModeConstants.setOrientation(OPModeConstants.Orientation.FRONT_FACING);
        OPModeConstants.DEBUG =false;

        // Get the camera going
        Task_JewelOrder jewelOrder = new Task_JewelOrder(hardwareMap);
        jewelOrder.Init();

        waitForStart();
        resetStartTime();

        Task_GlyphClaw pickGlyph = new Task_GlyphClaw(hardwareMap, OPModeConstants.GlyphClawPosition.CLOSE);
        pickGlyph.Init();
        while(pickGlyph.GetTaskStatus() == false){
            pickGlyph.PerformTask(telemetry, getRuntime());
            sleep(100);
        }
        pickGlyph.Reset();

        Task_LiftGlyph liftGlyph = new Task_LiftGlyph(hardwareMap);
        liftGlyph.Init();
        while(liftGlyph.GetTaskStatus() ==false)
        {
            liftGlyph.PerformTask(telemetry,getRuntime());
            sleep(200);
        }
        liftGlyph.Reset();

        while(jewelOrder.GetTaskStatus()==false) {
            jewelOrder.PerformTask(telemetry, getRuntime());
            sleep(100);
        }
        telemetry.addData("Jewel Order ",opModeConstants.getDetectedOrder().toString());
        telemetry.update();

        Task_GetCryptoKey getCryptoKey = new Task_GetCryptoKey(hardwareMap,telemetry);
        getCryptoKey.Init();
        while(getCryptoKey.GetTaskStatus() == false)
        {
            getCryptoKey.PerformTask(telemetry, getRuntime());
            sleep(100);

        }
        telemetry.addData("Crypto Location ",opModeConstants.getCryptoLocation());
        telemetry.update();
        Task_JewelArm jewelArm = new Task_JewelArm(hardwareMap, OPModeConstants.jewelKickerArmPosition.ACTION);
        jewelArm.Init();
        while(jewelArm.GetTaskStatus()==false)
        {
            jewelArm.PerformTask(telemetry, getRuntime());
            sleep(100);
        }
        telemetry.addData("Fire Sequence ",opModeConstants.getFireSequence().toString());
        telemetry.update();
        Task_JewelRemove jewelRemove = new Task_JewelRemove(hardwareMap);
        jewelRemove.Init();
        while(jewelRemove.GetTaskStatus() == false) {
            jewelRemove.PerformTask(telemetry, getRuntime());
            sleep(100);
        }

        jewelArm = new Task_JewelArm(hardwareMap, OPModeConstants.jewelKickerArmPosition.REST);
        jewelArm.Init();
        while(jewelArm.GetTaskStatus()==false)
        {
            jewelArm.PerformTask(telemetry, getRuntime());
            sleep(100);
        }

        //Robot path is where we set the drive action
        robotPath(opModeConstants.getCryptoLocation());
        Task_GlyphManeuver glyphManeuver = new Task_GlyphManeuver(hardwareMap);
        glyphManeuver.Init();
        while(glyphManeuver.GetTaskStatus() == false){
            glyphManeuver.PerformTask(telemetry, getRuntime());
            sleep(100);
        }
        glyphManeuver.Reset();

        Task_GlyphClaw glyphClaw = new Task_GlyphClaw(hardwareMap, OPModeConstants.GlyphClawPosition.OPEN);
        glyphClaw.Init();
        while(glyphClaw.GetTaskStatus() == false){
            glyphClaw.PerformTask(telemetry, getRuntime());
            sleep(100);
        }

        Task_LowerGlyph lowerGlyph = new Task_LowerGlyph(hardwareMap);
        lowerGlyph.Init();
        while(lowerGlyph.GetTaskStatus() == false){
            lowerGlyph.PerformTask(telemetry, getRuntime());
            sleep(100);
        }

        robotPush();
        Task_GlyphManeuver pushTask = new Task_GlyphManeuver(hardwareMap);
        pushTask.Init();
        while(pushTask.GetTaskStatus() == false){
            pushTask.PerformTask(telemetry, getRuntime());
            sleep(100);
        }
        pushTask.Reset();

        telemetry.addData("Tasks Completed In ", getRuntime());
        telemetry.update();
        sleep((30 - (int)getRuntime())*1000);
        //TODO -- Make sure to set motor power to 0 and encoder values to "DO NOT USE ENCODERS" in teleop
    }
    private void robotPath(RelicRecoveryVuMark vuMark){
        DriveInstructionsHelper firstAction = new DriveInstructionsHelper(OPModeConstants.DriveInstructions.FORWARD, 12.0d);
        DriveInstructionsHelper secondAction = new DriveInstructionsHelper(OPModeConstants.DriveInstructions.TURN, 90d);
        LinkedList initPair = new LinkedList<DriveInstructionsHelper>();
        initPair.add(firstAction);
        DriveInstructionsHelper vuMarkPosition = null;
        switch (vuMark){
            case CENTER:
                vuMarkPosition = new DriveInstructionsHelper(OPModeConstants.DriveInstructions.FORWARD, 6.0d);
                break;
            case LEFT:
                vuMarkPosition = new DriveInstructionsHelper(OPModeConstants.DriveInstructions.FORWARD, 0.0d);
                break;
            case RIGHT:
                vuMarkPosition = new DriveInstructionsHelper(OPModeConstants.DriveInstructions.FORWARD, 12.0d);
                break;
            default:
                vuMarkPosition = new DriveInstructionsHelper(OPModeConstants.DriveInstructions.FORWARD, 0.0d);
                break;
        }
        initPair.add(vuMarkPosition);
        initPair.add(secondAction);
        opModeConstants.setDrivePath(initPair);
    }
    private void robotPush(){
        DriveInstructionsHelper pushAction = new DriveInstructionsHelper(OPModeConstants.DriveInstructions.FORWARD, 6.0d);
        LinkedList initPair = new LinkedList<DriveInstructionsHelper>();
        initPair.add(pushAction);
        opModeConstants.setDrivePath(initPair);
    }

}


