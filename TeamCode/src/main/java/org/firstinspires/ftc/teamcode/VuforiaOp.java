package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.*;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Akanksha.Joshi on 2017-10-22.
 */

public class VuforiaOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "ARu0pnX/////AAAAGU7XB2WmyU5kv3ajbeT3HhRf7ACI" +
                "3GFxKNWK2EwbVeTa1hXzuQbGfR7dsY1LMIs4pT04HYGuNnVhAp1tSsaqFNgkshKf98ER/MSIecMKhl4NQv" +
                "FmW6WKA7xOMLJZKYe/T7bE1K/PJm53beyC2iqr9RXb6Yo8YLnTtHdMOv/Tj3jn6hW0drXzqmTGBW54BKz+fT" +
                "0z27n9P+O/73olp4OYrRYR3tdr81en2sZaHYISZdF8ryKhPIgTIg98+C8WowtFXrB1C2b40dc78NXxWxCgEHt" +
                "5YIDEXFTSLjB7wfr8FogqiPjBRGoiPwDrY9oRTpsXKMeZmDccEnjNOukwQWx/D6dPSL5lG1jECsVGWDHnzfaO";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");

        waitForStart();

        beacons.activate();

        while (opModeIsActive()){
            for(VuforiaTrackable beac : beacons){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

                if(pose != null){
                    VectorF translation = pose.getTranslation();

                    telemetry.addData(beac.getName() + "-Translation", translation);

                    double degreestToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2))); //translation.get(0), translation.get(2) for landscape

                    telemetry.addData(beac.getName() + "-Degrees", degreestToTurn);
                }
            }
            telemetry.update();
        }
    }
}
