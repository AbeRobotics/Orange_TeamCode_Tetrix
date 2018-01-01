package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.EnumMap;
import java.util.LinkedList;
import java.util.Map;

/**
 * Created by harsh.joshi on 31-Dec-2017.
 */

public class Task_GlyphManeuver extends IOPModeTaskBase {
    private boolean taskSatisfied = false;
    private OPModeConstants opModeConstants = null;
    private HardwareMap hardwareMap;
    LinkedList<DriveInstructionsHelper> drivePath = null;
    public Task_GlyphManeuver(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }
    @Override
    public boolean GetTaskStatus() {
        return taskSatisfied;
    }

    @Override
    public void PerformTask(Telemetry telemetry, double elapsedTime) {
        if (drivePath.size() > 2 && elapsedTime > OPModeConstants.GlyphManeuver) {
            taskSatisfied = true;
            return;
        }
        if (drivePath.size() < 3 && elapsedTime > OPModeConstants.PushGlyph) {
            taskSatisfied = true;
            return;
        }
        boolean availableTask = true;
        while (availableTask){
            DriveInstructionsHelper firstInstruction = drivePath.peekFirst();
            if (firstInstruction == null){
                availableTask = false;
            }
            else{
                OPModeDriveHelper driveHelper = OPModeDriveHelper.getInstance();
                driveHelper.Init(telemetry, hardwareMap);
                OPModeConstants.DriveInstructions action = firstInstruction.action;
                double value = firstInstruction.value;
                switch (action){
                    case FORWARD:
                        driveHelper.MoveForward(value);
                        break;
                    case REVERSE:
                        driveHelper.MoveBackward(value);
                        break;
                    case TURN:
                        driveHelper.Turn((int)value, opModeConstants.getAutoSpeed());
                        break;
                    case UNKNOWN:
                        driveHelper.ResetDriveEncoders();
                        driveHelper.SetAllStop();
                        break;
                    default:
                        driveHelper.ResetDriveEncoders();
                        driveHelper.SetAllStop();
                        break;
                }
            }
            taskSatisfied = true;
        }
    }

    @Override
    public void Init() {
        opModeConstants = OPModeConstants.getInstance();
        drivePath = opModeConstants.getDrivePath();
    }

    @Override
    public void Reset() {
        Map initEnumMap = new EnumMap<OPModeConstants.DriveInstructions, Integer>(OPModeConstants.DriveInstructions.class);
        initEnumMap.put(OPModeConstants.DriveInstructions.UNKNOWN, 0);
        LinkedList initPair = new LinkedList<EnumMap<OPModeConstants.DriveInstructions, Integer>>();
        initPair.add(initEnumMap);
        opModeConstants.setDrivePath(initPair);
        taskSatisfied = false;
    }
}
