package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.io.IOException;
import org.firstinspires.ftc.teamcode.OPModeConstants;


/**
 * Created by Akanksha.Joshi on 23-Dec-2017.
 */
@Autonomous(name="Jewel Task Red Team", group="Autonomous")
public class RedTeam_JewelTest extends LinearOpMode{

    public OPModeConstants opModeConstants = null;
    public JewelDetectorFacade jewelDetectorFacade= null;
    public JewelDetector jewelDetector= null;
    private JewelDriveMode jewelDriveMode = null;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean jewelRemoved = false;

    @Override
    public void runOpMode() throws InterruptedException {

        performAutonomousInitialization();

        telemetry.addData("Status", "Initialized");
        opModeConstants.setSelectedTeam(OPModeConstants.SelectedTeam.RED_TEAM);
        opModeConstants.setOrientation(OPModeConstants.Orientation.LEFT_SIDE);
        Servo leftArm = hardwareMap.servo.get("left_arm");
        telemetry.setAutoClear(false);
        ResetToInitialPosition();

        waitForStart();
        // Top OPMode LOOP
        while(opModeConstants.getDetectedOrder() == JewelDetector.JewelOrder.UNKNOWN) {
                JewelDetector.JewelOrder jewelOrder = jewelDetectorFacade.getJewelOrder(jewelDetector);
                opModeConstants.setDetectedOrder(jewelOrder);
                sleep(100);
        }
        OPModeConstants.JewelDetectionDisabled = true;
        jewelDetectorFacade.stop(jewelDetector);
        leftArm.setPosition(OPModeConstants.jewelArmActive);
        while(jewelRemoved==false)
        {

            OPModeConstants.FireSequence fireSequence = opModeConstants.getFireSequence();
            if( fireSequence != OPModeConstants.FireSequence.UNKNOWN)
            {
                telemetry.addData("Wheel Firing Position", opModeConstants.getFireSequence());
                telemetry.update();
                jewelRemoved =  jewelDriveMode.performJewelRemovalTask(fireSequence,hardwareMap,telemetry);
            }
            sleep(1000);
        }
        leftArm.setPosition(OPModeConstants.jewelArmInactive);
        telemetry.addData("Arm Back in Resting Position", OPModeConstants.jewelArmInactive);
        telemetry.update();
        sleep(25000);
    }

    public void performAutonomousInitialization() {
        /*Init All Singletons*/
        jewelDetectorFacade = JewelDetectorFacade.getInstance();
        jewelDriveMode = JewelDriveMode.jewelDriveModeInstance();
        opModeConstants = OPModeConstants.getInstance();
        jewelDetector = new JewelDetector();
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        jewelDetector.areaWeight = 0.05;
        jewelDetector.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA;
        jewelDetector.debugContours = false;
        jewelDetector.maxDiffrence = 15;
        jewelDetector.ratioWeight = 15;
        jewelDetector.minArea = 700;
        jewelDetector.speed = JewelDetector.JewelDetectionSpeed.BALANCED;
        jewelDetector.rotateMat = true;
        jewelDetector.enable();
        opModeConstants = OPModeConstants.getInstance();
        jewelDetectorFacade = JewelDetectorFacade.getInstance();
    }
    void ResetToInitialPosition() {
        jewelDetectorFacade.Reset();
        jewelDriveMode.Reset();
        opModeConstants.Reset();
    }
}


