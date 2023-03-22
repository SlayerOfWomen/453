package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class Constants {
    public static final class DriveConstants{
        //X drive
        public static final int kLeftFrontPort = 2;
        public static final int kLeftRearPort = 1;
        public static final int kRightFrontPort = 3;
        public static final int kRightRearPort = 0;

        //Swap drive
        public static final int kLeft1Port = 1;
        public static final int kLeft2Port = 3;
        public static final int kRight1Port = 2;
        public static final int kRight2Port = 4;

        public static final int ktankSolLeftOut = 0;
        public static final int ktankSolLeftIn = 1;
        public static final int ktankSolRightOut = 3;
        public static final int ktankSolRightIn = 2;

        public static final double k_AutoCorrectSpeed = 0.25;
        public static final double k_AutoCorrectDist = 1.0;
        public static final double k_AutoCorrectTurn = 0.1;

        public static final double kTrackWidth = 0.68;
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 0.68;
        // Distance between centers of front and back wheels on robot

        public static final int kGearRatio = 16;

        public static final MecanumDriveKinematics kDriveKinematics =
            new MecanumDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kEncoderCPR = 2048;
        public static final double kWheelDiameterMeters = 0.1524;
        public static final double kEncoderDistancePerPulse =
            (kWheelDiameterMeters * Math.PI) / ((double) kEncoderCPR * kGearRatio);
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The SysId tool provides a convenient method for obtaining these values for your robot.
        public static final SimpleMotorFeedforward kFeedforward =
        new SimpleMotorFeedforward(1, 0.8, 0.15);

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPFrontLeftVel = 0.5;
        public static final double kPRearLeftVel = 0.5;
        public static final double kPFrontRightVel = 0.5;
        public static final double kPRearRightVel = 0.5;

        public static final double kNormSpeedMult = .6;
        public static final double kTurboSpeedMult = 1.0;

        //Button Mapping
        public static final int kTurboButtonMap = 3;
        public static final int kIntakeInMap = 9;
        public static final int kIntakeOutMap = 11;
        public static final int kLifterUpMap = 10;
        public static final int kLifterDownMap = 12;
        public static final int kClimberUpMap = 7;
        public static final int kClimberDownMap = 8;
        public static final int kSwapDriveMap = 7;
    }

    public static final class IOConstants {
        // IO Ports and Buttons
        public static final int driverJoyPort = 0;
        public static final int talonFXPort = 1;
        public static final int talonSRXPort = 5;
        public static final int pigeonPort = 0;
        public static final int pcmPort = 0;
        public static final int pdpPort = 0;
        public static int kOperatorControllerPort = 1;
        public static int kDriverControllerPort = 0;
        public static int armAxisNum = 1;
        public static int kArmManualControlButton = 4;
    }
    

    public static final class ArmConstants {
        // PID gains may have to be adjusted based on the responsiveness of control loop.
        public static final int kSlotIdx = 0;
        // Talon FX supports multiple (cascaded) PID loops. For now we just want the primary one.
        public static final int kPIDLoopIdx = 0;
        // The timeout for the PID loops
        public static final int kTimeoutMs = 1;
        // Sets the direction of the sensor. True sets the sensor to positive when outputting
        public static final boolean kSensorPhase = true;
        // Invert motor or not
        public static boolean kMotorInvert = false;
        // Brake or coast the motor
        public static NeutralMode kNeutralMode = NeutralMode.Brake;

        //Solendoid
        public static int kForwardChannel = 4;
        public static int kReverseChannel = 5;
        public static int pOutButton = 6;
        public static int pInButton = 4;

        //Arm Position Constants
        public static double kBottomPosition = 0;
        public static double kMiddlePosition = 2500;
        public static double kTopPosition = 5000;
        public static final int resetArmButton = 7;
        public static final  int bottomPosButton = 8;
        public static double armAxisThreshold = 0.1;
        public static final int middlePosButton = 10;
        public static final int topPosButton = 12;
        public static double kArmSpeedMultiplier = -250;

        // PID gains may have to be adjusted based on the responsiveness of control loop. PID is used to correct sensor input.
        /* It is composed of three terms: proportional, integral, and derivative.
        * The proportional term is the error multiplied by a constant.
        * The integral term is the sum of the errors multiplied by a constant.
        * The derivative term is the change in error multiplied by a constant.
        * The sum of these three terms is the output of the PID loop.
        * The constants are called the gains.
        */

        // kp is the proportional gain, which is used to correct sensor input
        public static final double kP = 0.7;
        // ki is the integral gain, which is used to correct steady state errors
        public static final double kI = 0.0006;
        // kd is the derivative gain, which is used to reduce the amount of overshoot
        public static final double kD = 0.7;
        // kf is the feed forward gain, which is used to calculate the output of the motor
        public static final double kF = 0;
        // kIzone is the integral zone, which is the range of error where the integral gain
        public static final int kIzone = 0;
        // kPeakOutput is the maximum output of the PID loop, used to set the limit of the motor
        public static final double kPeakOutput = 0.5;
}
}
