/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Controllers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.motion.ProfilePoint;
import frc.robot.motion.SpeedPID;
import frc.robot.motion.TrapezoidalMotionProfile;

public class TurnController implements DrivetrainController {
    Constants constants;
    double startAngle;
	
	double startT;
	TrapezoidalMotionProfile profile;
	SpeedPID speedPID;
    PIDController speedController;
    
    double allowedError;
	
	double linearActual, linearSetpoint, linearError;
	public TurnController(TrapezoidalMotionProfile profile, double allowedError, double kP, double kI, double kD) {
        constants = Constants.getConstants();
        
        this.allowedError = allowedError;
        startAngle = Robot.driveTrain.getAngle();
        startT = Timer.getFPGATimestamp();
		
		this.profile = profile;
		refreshConstants();


        speedController = new PIDController(kP, kI, kD);
	}

	/*
	 * Configure this controller to begin following the profile, making t=0 the current moment.
	 * @param theta The desired angle for drivestraight correction
	 */
	
	@Override
	public boolean update() {
		double t = Timer.getFPGATimestamp() - startT;
		ProfilePoint point = profile.getAtTime(t);
		
		double feedforward = (point.vel * 0.018) + (point.acc * 0);
		
		linearActual = Robot.driveTrain.getAngle();
		linearSetpoint = point.pos;
		double error = (linearActual-startAngle) - point.pos;
		double output = (error * 0.05) + feedforward;

		if(output > 0.7){
			output = 0.7;
		}else if(output < -0.7){
			output = -0.7;
        }
        

        if(output < constants.minOutputTurn && output >= 0){
			output = constants.minOutputTurn;
		}else if(output > -constants.minOutputTurn && output <= 0){
			output = -constants.minOutputTurn;
		}
		
		linearError = error;
		Robot.driveTrain.setSpeed(output, output);
		
		return Math.abs(error) < Math.abs(allowedError);
	}

	public void reset() {
		startT = Timer.getFPGATimestamp();
	}

	@Override
	public void refreshConstants() {
	}

	@Override
	public double getLinearError() {
		return linearError;
	}

	@Override
	public double getLinearActual() {
		return linearActual;
	}

	@Override
	public double getLinearSetpoint() {
		return linearSetpoint;
	}

	@Override
	public double getAngularError() {
		return 0;
	}

	@Override
	public double getAngularActual() {
		return 0;
	}

	@Override
	public double getAngularSetpoint() {
		return 0;
	}
}