/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Aiming;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class SetAimingDown extends CommandBase {

  private PIDController controller;
  private double currentAimValue;
  private double target;
  boolean up;
  private double startT;
  public SetAimingDown(boolean up) {
    this.up = up;
    addRequirements(Robot.aiming);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startT = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(up){
      Robot.aiming.setAimSpeed(-0.3);
    }else{
      Robot.aiming.setAimSpeed(0.3);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.aiming.setAimSpeed(0.0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double newT = Timer.getFPGATimestamp();
    if(up){
      return newT-startT > 0.8 || Robot.aiming.getPotentometerPosition() > 0.4;
    } else {
      return newT-startT > 0.8 || Robot.aiming.getPotentometerPosition() < 0.4;
    }
  }
}
