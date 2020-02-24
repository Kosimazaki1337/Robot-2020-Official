/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Controllers;

public interface DrivetrainController {
	/* 
	 * Run one cycle of this controller
	 * @returns true if on target, false if not
	 */
	public boolean update();
	
	public double getLinearError();
	public double getLinearActual();
	public double getLinearSetpoint();
	
	public double getAngularError();
	public double getAngularActual();
	public double getAngularSetpoint();
	
	public void reset();
	
	/*
	 * Reload constants that may have changed at runtime
	 * Intended to be used while tuning
	 */
	public void refreshConstants(); 
}
