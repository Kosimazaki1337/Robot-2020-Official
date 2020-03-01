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

public class ShooterRightController implements DrivetrainController {
	Constants constants;
	
	double startT;
	TrapezoidalMotionProfile profile;
	SpeedPID speedPID;
	PIDController speedController;
	
	double linearActual, linearSetpoint, linearError;
	public ShooterRightController(TrapezoidalMotionProfile profile) {
		constants = Constants.getConstants();
		
		
		this.profile = profile;
		refreshConstants();


		speedController = new PIDController(Constants.kPShootRight, Constants.kIShootRight, Constants.kDShootRight, 0.001);
	}

	/*
	 * Configure this controller to begin following the profile, making t=0 the current moment.
	 * @param theta The desired angle for drivestraight correction
	 */
	
	@Override
	public boolean update() {
		double t = Timer.getFPGATimestamp() - startT;
		ProfilePoint point = profile.getAtTime(t);
		
		double feedforward = (point.vel * constants.kShooterRight_kV) + (point.acc * constants.kShooterRight_kA);
		
		linearActual = Robot.shooter.getRSpeed();
		linearSetpoint = point.pos;
		double error = linearSetpoint-linearActual;
		SmartDashboard.putNumber("linearActualRight", linearActual);
		SmartDashboard.putNumber("error", error);
		double output = speedController.calculate(linearActual, linearSetpoint) + point.pos*0.01;

		if(output > 0.75){
			output = 0.75;
		}else if(output < -0.75){
			output = -0.75;
		}
		
		linearError = error;
		Robot.shooter.setRightSpeed(output);
		SmartDashboard.putNumber("Output RightShooterController", output);
		
		return Math.abs(error) < constants.allowedShooterError; //t >= profile.getDuration() &&
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