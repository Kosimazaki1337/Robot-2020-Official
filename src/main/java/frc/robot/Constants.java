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

    public static final double openLoopkPLeft = 0.15;
    public static final double openLoopkILeft = 0.0018;
    public static final double openLoopkDLeft = 0.0;
    public static final double openLoopkPRight = 0.15;
    public static final double openLoopkIRight = 0.0018;
    public static final double openLoopkDRight = 0.0;
    public static final double openLoopErrorTolerance = 0.05;
    public static final double joyDeadZone = 0.12;


    //DriveTrain
    public static final double kWheelDiameter = 0.19; // meters
    public static final double kWheelCircuit = Math.PI*kWheelDiameter; // circuit in meters
    public static final double kTrackWidthMeters = 0.5;
    
    public static final double kS = 0.0; // min voltage to run motors
    public static final double kV = 0.0; // velocity * seconds per meter
    public static final double kA = 0.0; // velocity * squared seconds per meter
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double maxVelocityMetersPerSecond = 4.0;
    public static final double maxAccelerationMetersPerSecondSqr = 1.0;
    public static final double kRamseteTuningB = 0.0;
    public static final double kRamseteTuningZeta = 0.0;

    //Intake Constants
    public double intakeSwitch = 0;

    //Shooter Constants
    public static final double kPShootLeft = 0.017; 
    public static final double kIShootLeft = 0.0015; 
    public static final double kDShootLeft = 0.0001; 
    public static final double kFShootLeft = 0.0; 
    public static final double kPShootRight = 0.013; //0.22
    public static final double kIShootRight = 0.0015; //0.001
    public static final double kDShootRight = 0.00016;  
    public static final double kFShootRight = 0.0; 

    public static final double maxShootSpeed =  70.0; //RPS
    public static final double maxShootAcceleration = 8.0;
    public static final double kShooterLeft_kV = 0.0141;
    public static final double kShooterLeft_kA = 0.00365;
    public static final double kShooterRight_kV = 0.0148;
    public static final double kShooterRight_kA = 0.00179;


    public static final double allowedShooterError = 0.5;


    //Transporter Constants
    public int ballCounter = 0;
    public static final double sparkTicksToMove = 4;

    public static final double kPSpark = 1;
    public static final double kISpark = 0.0;
    public static final double kDSpark = 0.0;
    public static final double kIzSpark = 0.0;
    public static final double kFFSpark = 0.0;
    public static final double kSparkMaxOutput = 0.15;
    public static final double kSparkMinOutput = 0;

    public static final double detectedColorMin = 8.0;
    
    public static final double twoBallsSpeedTransporter = 0.21;
    public static final double oneBallsSpeedTransporter = 0.1;

    //Aiming Constants
    public static double maxUpPotentimeterValue = 0.28;
    public static double minDownPotentimeterValue = 0.42;

    public static double aimingkP = 0;
    public static double aimingkI = 0;
    public static double aimingkD = 0;

    public boolean isShooting = false;
    
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
