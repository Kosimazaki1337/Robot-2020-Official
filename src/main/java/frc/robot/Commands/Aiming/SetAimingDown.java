/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Aiming;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class SetAimingDown extends CommandBase {

  private PIDController controller;
  private double currentAimValue;
  private double target;
  double error;

  public SetAimingDown() {
    addRequirements(Robot.aiming);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller = new PIDController(0.9, 0.6, 0.04);
    target = Constants.minDownPotentimeterValue;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAimValue = Robot.aiming.getPotentometerPosition();
    //Robot.aiming.setAimSpeed(controller.calculate(currentAimValue, target));
    Robot.aiming.setAimSpeed(0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.aiming.setAimSpeed(0.0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    error = target - currentAimValue;
    return currentAimValue >= 0.47;
  }
}
