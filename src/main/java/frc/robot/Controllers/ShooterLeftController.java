/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Controllers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.motion.ProfilePoint;
import frc.robot.motion.SpeedPID;
import frc.robot.motion.TrapezoidalMotionProfile;

public class ShooterLeftController implements DrivetrainController {
	Constants constants;
	
	double startT;
	TrapezoidalMotionProfile profile;
	SpeedPID speedPID;
	
	double linearActual, linearSetpoint, linearError;
	public ShooterLeftController(TrapezoidalMotionProfile profile) {
		constants = Constants.getConstants();
		
		
		this.profile = profile;
		refreshConstants();

		speedPID = new SpeedPID(Constants.kPShootLeft, Constants.kIShootLeft, Constants.kDShootLeft, Constants.kFShootLeft);
	}

	/*
	 * Configure this controller to begin following the profile, making t=0 the current moment.
	 * @param theta The desired angle for drivestraight correction
	 */
	
	@Override
	public boolean update() {
		double t = Timer.getFPGATimestamp() - startT;
		ProfilePoint point = profile.getAtTime(t);
		
		double feedforward = (point.vel * constants.kShooterLeft_kV) + (point.acc * constants.kShooterLeft_kA);
		
		linearActual = Robot.shooter.getLSpeed();
		linearSetpoint = point.pos;
		double error = linearSetpoint-linearActual;
		//double output = speedPID.calculate(-linearSetpoint, -linearActual) + feedforward;
		double output = (error * 0.17) + feedforward;
		if(output > 0.75){
			output = 0.75;
		}else if(output < -0.75){
			output = -0.75;
		}
		
		linearError = error;
		Robot.shooter.setLeftSpeed(output);
		SmartDashboard.putNumber("Output LeftShooterController", output);
		
		return t >= profile.getDuration() && Math.abs(error) < constants.allowedShooterError;
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
