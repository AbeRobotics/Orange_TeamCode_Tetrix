package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.JewelDetector;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Akanksha.Joshi on 23-Dec-2017.
 */

public class OPModeConstants {
    private static OPModeConstants opModeConstants;
    public static double jewelArmActive = 0.52d;
    public static double jewelArmInactive = 0.00d;
    public static double degreesToInch = 0.209d;
    public static boolean JewelDetectionDisabled = false;
    public static final double ticksPerInch = 114.67;
    public static final double gearRatio = 0.5;
    private OPModeConstants()
    {
        setFireSequence(FireSequence.UNKNOWN);
        setSelectedTeam(SelectedTeam.UNKNOWN);
        setDetectedOrder(JewelDetector.JewelOrder.UNKNOWN);
        setCryptoLocation(RelicRecoveryVuMark.UNKNOWN);
        setOrientation(Orientation.UNKNOWN);
        setAutoSpeed(AutonomousSpeed.HIGH);
        jewelArmActive = 0.52d;
        jewelArmInactive = 0.00d;
        JewelDetectionDisabled = false;
    }
    public static OPModeConstants getInstance()
    {
        if(opModeConstants==null)
        {
            opModeConstants = new OPModeConstants();
        }
        return opModeConstants;
    }

    public enum SelectedTeam
    {
        RED_TEAM,
        BLUE_TEAM,
        UNKNOWN
    }
    public enum FireSequence
    {
        FORWARD,
        BACKWARD,
        UNKNOWN
    }
    public enum jewelKickerArmPosition
    {
        REST,
        ACTION
    }
    public enum Orientation
    {
        FRONT_FACING,
        BACK_FACING,
        UNKNOWN
    }
    public enum AutonomousSpeed
    {
        SLOW,
        MEDIUM,
        HIGH
    }
    private RelicRecoveryVuMark cryptoLocation;
    private JewelDetector.JewelOrder jewel_order;
    private SelectedTeam selectedTeam;
    private FireSequence fireSequence;
    private jewelKickerArmPosition jewelKickerArmPosition;
    private Orientation orientation;
    private AutonomousSpeed autoSpeed;

    public AutonomousSpeed getAutoSpeed()
    {
        return autoSpeed;
    }
    public void setAutoSpeed(AutonomousSpeed speed)
    {
        autoSpeed = speed;
    }
    public void setOrientation (Orientation orientation) {this.orientation = orientation;}
    public Orientation getOrientation(){return orientation;}
    public void setCryptoLocation(RelicRecoveryVuMark cryptoLocation)
    {
        this.cryptoLocation = cryptoLocation;
    }
    public RelicRecoveryVuMark getCryptoLocation()
    {
        return cryptoLocation;
    }
    public void setDetectedOrder (JewelDetector.JewelOrder order){
        jewel_order = order;
    }
    public JewelDetector.JewelOrder getDetectedOrder ()
    {
        return jewel_order;
    }
    public void setSelectedTeam(SelectedTeam team)
    {
        selectedTeam = team;
    }
    public SelectedTeam getSelectedTeam()
    {
        return selectedTeam;
    }
    public void setFireSequence(FireSequence sequence)
    {
        fireSequence= sequence;
    }
    public FireSequence getFireSequence()
    {
        if((getSelectedTeam()==SelectedTeam.BLUE_TEAM) &&(getDetectedOrder() == JewelDetector.JewelOrder.BLUE_RED))
            {
                setFireSequence(FireSequence.FORWARD);
            }
        if((getSelectedTeam()==SelectedTeam.BLUE_TEAM) &&(getDetectedOrder() == JewelDetector.JewelOrder.RED_BLUE))
        {
            setFireSequence(FireSequence.BACKWARD);
        }
        if((getSelectedTeam()==SelectedTeam.RED_TEAM) &&(getDetectedOrder() == JewelDetector.JewelOrder.RED_BLUE))
        {
            setFireSequence(FireSequence.FORWARD);
        }
        if((getSelectedTeam()==SelectedTeam.RED_TEAM) &&(getDetectedOrder() == JewelDetector.JewelOrder.BLUE_RED))
        {
            setFireSequence(FireSequence.BACKWARD);
        }
        return fireSequence;
    }
    public void setJewelKickerArmPosition(jewelKickerArmPosition position)
    {
        jewelKickerArmPosition = position;
    }
    public jewelKickerArmPosition getJewelKickerArmPosition()
    {
        return jewelKickerArmPosition;
    }
    public void Reset()
    {
        opModeConstants = null;
    }
}
