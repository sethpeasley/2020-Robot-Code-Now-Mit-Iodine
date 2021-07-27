/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.util.Color;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants
{
    // DriveTrain CAN ids
    public static final class CAN_Addresses
    {
        public static int rightDriveACAN = 10;
        public static int leftDriveACAN = 11;
        public static int rightDriveBCAN = 13;
        public static int leftDriveBCAN = 14;
        
        // climber CAN
        public static int kClimbFXCAN = 15;
    }

    public static final class DriveTrainContants
    {
        public static double k_RampRate = .25;
        public static int k_ContinuousCurrentLimit = 40;
        public static int k_PeakCurrentLimit = 45;
        public static int k_PeakCurrentDuration = 50;
    }

    // Autonomous Constants
    public static final double k_EncoderTicksPerFoot = 2607.595;
    public static final double k_p = 3.5;// 3.5
    public static final double k_d = 0;
    public static final double k_v = .08;// .08
    public static final double k_a = .06;// .06

    public static double k_PTurn = .05;
    public static double k_ITurn = 0;
    public static double k_DTurn = .1;
    public static double k_turnEpsilon = 3.0;

    public static double k_PdriveTurn = .06;
    public static double k_IdriveTurn = 0;
    public static double k_DdriveTurn = 0.0;

    public static double k_PDrive = 1.5;
    public static double k_IDrive = 0.0;
    public static double k_DDrive = 4.0;
    public static double k_driveEbsilon = 2.0;

    public static double k_PHold = -.0001;

    public static double k_IntakeSpeed = .65;
    public static int k_IntakeMotorCAN = 1; // this is for the intake mechanism
    public static int k_FlipMotorCAN = 2;
    public static int k_IntakeTicksPerRotation = 2048;
    public static int k_IntakeUpPosition = 0;
    public static int k_IntakeDownPosition = 0;
    public static int k_IntakeAbsoluteInput = 0;

    public static double k_PIntake = 0;
    public static double k_IIntake = 0;
    public static double k_DIntake = 0;
    public static double k_intakeEpsilon = 0;

    public static int k_InputTOFCAN = 2;
    public static int k_OutputTOFCAN = 1;
    public static int k_BackupTOFCAN = 3; // not used unless one breaks
    public static int k_IndexMotorCAN = 3; // temporary CAN ID


    // Indexer Doubles in Millimeters
    public static double k_CellIncomingValueLow = 30.0; // 30 mm
    public static double k_CellIncomingValueHigh = 90.0; // 110 mm
    public static double k_CellOutgoingValueLow = 30.0; // 30 mm
    public static double k_CellOutgoingValueHigh = 90.0; // 110 mm

    // Indexer Sampling Period In Milliseconds
    public static int k_IndexerSamplingPeriod = 40; // 40 ms

    // Indexer Preloaded Power Cells / Maximum Power cell amount
    public static int k_CellsPreloaded = 3;

    public static double k_IndexerStowingMotorPower = .8;
    public static double k_IndexerShootingMotorPower = .8;

    public static int k_AutoDriveTime = 3; // This is for our simple auto example
    public static int k_AutoTimeoutSeconds = 5; // This is for our simple auto example

    public static int k_CPMotorPort = 4;

    public static int k_ShooterACAN = 5;
    public static int k_ShooterBCAN = 6;
    public static int k_TurnTurretCAN = 12;

    public static double k_PShooter = 0.02; // TUNED
    public static double k_IShooter = 0.0; // TUNED
    public static double k_DShooter = 0.001; // TUNED
    public static int k_shooterEpsilon = 1; // TUNED

    public static double k_shooterMaxOutput = 0.85; // TUNED
    public static double k_shooterRampTime = 0.25; // TUNED
    public static int k_ForwardSoftLimitValue = 0;
    public static int k_ReverseSoftLimitValue = 0;

    public static double k_Prpm = 0.00035; // TUNED
    public static double k_Irpm = 0.0; // TUNED
    public static double k_Drpm = 0.0; // TUNED
    public static double k_rpmFF = 0.000175; // TUNED
    public static double k_rpmMaxOutput = 1.0; // TUNED
    public static double k_rpmMinOutput = 0.0; // TUNED
    public static double k_rpmRampTime = 0.35; // TUNED

    public static int k_MaxCPTicks = 8;
    public static int k_MinColorReadingUntilAccepted = 5;
    public static final double k_ControlPanelSpeed = 0.5;

    // Pid Loop For Climb
    public static final double k_PClimb = 0.00006;
    public static final double k_IClimb = 0;
    public static final double k_DClimb = 0.0001;
    public static final double k_climbEpsilon = 100;
    public static final double k_ClimbMaxOutput = 0.8;

    public static final int k_raiseClimbButton = 1;
    public static final int k_optestbutton = 10;

    public static Color k_BlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    public static Color k_GreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    public static Color k_RedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    public static Color k_YellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    // Driver Controller Bindings
    public static final class HIDConstants
    {
        // Driver Controller port
        public static int k_DriverPort = 0;
        public static int k_OperatorPort = 1;

        // Driver Controller Axis's

        // left stick axis's
        public static int k_LeftStickX =  0;
        public static int k_LeftStickY = 1;

        // Right Stick Axis's
        public static int k_RightStickX = 4;
        public static int k_RightStickY = 5;

        // Triggers
        public static int k_RightTrigger = 3;
        public static int k_LeftTrigger = 2;

        // Buttons Bindings
        public static int k_A = 1;
        public static int k_B = 2;
        public static int k_X = 3;
        public static int k_Y = 4;

        public static int k_LB = 5;
        public static int k_RB = 6;

        public static int k_Select = 7;
        public static int k_Start = 8;

        public static int k_LeftStickPress = 9;
        public static int k_RightStickPress = 10;
    }
}
