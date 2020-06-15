/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class Constants {

    private static Constants instance;
	
	public static Constants getConstants() {		
		if ( instance == null ) {
				instance = new Constants();
		}
		return instance;
    }
    
    private String gameData = null;

    //Talon
    public static final int kLongCANTimeoutMs = 100;
    public static final double kLongCANTimeoutSec = 0.01;
    public static final double kDriveVoltageRampRate = 0.0;

    public static final double joyDeadZone = 0.12;


    //DriveTrain
    public static final double kWheelDiameter = 0.19; // meters
    public static final double kWheelCircuit = Math.PI*kWheelDiameter; // circuit in meters
    public static final double kTrackWidthMeters = 0.5;

    public static final double openLoopkPLeft = 0.104;
    public static final double openLoopkILeft = 0.00034;
    public static final double openLoopkDLeft = 0.000245;
    public static final double openLoopkPRight = 0.104;
    public static final double openLoopkIRight = 0.00035;
    public static final double openLoopkDRight = 0.000255;
    public static final double openLoopErrorTolerance = 0.05;
    
    public static final double kS = 1.02; // min voltage to run motors
    public static final double kV = 1.39; // velocity * seconds per meter
    public static final double kA = 0.383; // velocity * squared seconds per meter
    public static final double kP = 14;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public final double maxVelocityMetersPerSecond = 0.1;
    public final double maxAccelerationMetersPerSecondSqr = 0.01;
    public static final double kRamseteTuningB = 2;
    public static final double kRamseteTuningZeta = 0.7;

    //Intake Constants
    public double intakeSwitch = 0;    

    //Shooter Constants
    public static final double kPShootLeft = 0.0207; 
    public static final double kIShootLeft = 0.0045; //15
    public static final double kDShootLeft = 0.00015; 
    public static final double kFShootLeft = 0.0; 
    public static final double kPShootRight = 0.017; //0.22
    public static final double kIShootRight = 0.0015; //0.001
    public static final double kDShootRight = 0.00018;  
    public static final double kFShootRight = 0.0; 

    public static final double maxShootVelocity = 60.0;
    public static final double maxShootAcceleration = 45.0;
    public static final double kShooterLeft_kV = 0.0141;
    public static final double kShooterLeft_kA = 0.00365;
    public static final double kShooterRight_kV = 0.0148;
    public static final double kShooterRight_kA = 0.00179;
    
    public static final double kMaxShooterOutput = 0.95;
    
    public static final double maxShootSpeed =  54.0; //RPS
    public static final double allowedShooterError = 5.5;

    //Transporter Constants
    public static final double sparkTicksToMove = 4;

    public static final double kPSpark = 1;
    public static final double kISpark = 0.0;
    public static final double kDSpark = 0.0;
    public static final double kIzSpark = 0.0;
    public static final double kFFSpark = 0.0;
    public static final double kSparkMaxOutput = 0.15;
    public static final double kSparkMinOutput = 0;

    public static final double detectedColorMin = 10.0;
    
    public static final double twoBallsSpeedTransporter = 0.2;
    public static final double threeBallsSpeedTransporter = 0.2;

    //Aiming Constants
    public static double maxUpPotentimeterValue = 0.30;
    public static double minDownPotentimeterValue = 0.48;

    public static double aimingkP = 1.12358132134;
    public static double aimingkI = 0;
    public static double aimingkD = 0;

    public final static double aimMinYAllowError = 0.015;
    public final static double aimMinPotenciometrAllowError = 0.015;

    //LimeLight

    public static double yOffsetAllowedError = 0.5;
    public static double yAimSetpoint = -1.0;
    public static double xOffsetAllowedErrorStageOne = 1.2;
    public static double xOffsetAllowedErrorStageTwo = 0.8;
    public static double xOffsetAllowedErrorStageThree = 0.4;

    public static final double kPLimeAim = 0.028;
    public static final double kILimeAim = 0.0004;
    public static final double kDLimeAim = 0.0008;

    public static final double kPLimeAimZoom2 = 0.056;
    public static final double kILimeAimZoom2 = 0.0004;
    public static final double kDLimeAimZoom2 = 0.00008;

    public static final double kPLimeAimZoom3 = 0.3;
    public static final double kILimeAimZoom3 = 0.0004;
    public static final double kDLimeAimZoom3 = 0.00008;

    public boolean isShooting = false;

    public static final double kLimeAimY = 0.35;
    public static final double kLimeAimStageTwoY = 0.1;
    public static final double kLimeAimTargetY = 9.0;
    public static final double kYAllowedError = 2.0;
    public static final double yAmingTargetAngle = -16.5; //-20

    //turn controller
    public static final double minOutputTurn = 0.255;

    public static final double turnAllowedError = 3;
    public static final double turnkP = 0.03;
    public static final double turnkI = 0.0004;
    public static final double turnkD = 0.0008;

    public static final double driveAnglekP = 0.02;
    public static final double turnMaxVelocity = 30;
    public static final double turnMaxAcceleration = 30;
    public static final double maxTurnOutput = 0.15;
    
    public String getGameData() {
        return gameData;
    }
    public void setGameData(String gameData) {
        this.gameData = gameData;
    }

    public void setShootingFlag(boolean flag){
        isShooting = flag;
    }

    public boolean getShootingFlag(){
        return isShooting;
    }
} 
