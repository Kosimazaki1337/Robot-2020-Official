/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Controllers.ShooterLeftController;
import frc.robot.Controllers.ShooterRightController;
import frc.robot.motion.SpeedPID;
import frc.robot.motion.TrapezoidalMotionProfile;

public class Shoot extends CommandBase {
  
  SpeedPID shootPIDLeft;
  SpeedPID shootPIDRight;

  Constants constants;
  TrapezoidalMotionProfile rProfile;
  TrapezoidalMotionProfile lProfile;
  ShooterLeftController leftController;
  ShooterRightController rightController;
  
  double oldTime;
  double sumTime;

  public Shoot() {
    addRequirements(Robot.shooter);

    constants = new Constants();
  }

  @Override
  public void initialize() {
    rProfile = new TrapezoidalMotionProfile(constants.maxShootSpeed, constants.maxShootSpeed, 1.0);
    lProfile = new TrapezoidalMotionProfile(constants.maxShootSpeed, constants.maxShootSpeed, 1.0);

    leftController = new ShooterLeftController(lProfile);
    rightController = new ShooterRightController(rProfile);
    leftController.reset();
    rightController.reset();

    oldTime = Timer.getFPGATimestamp();
    sumTime = 0;
    constants.setShootingFlag(true);
    SmartDashboard.putBoolean("isS", constants.isShooting);
  }

  @Override
  public void execute() {
    //Robot.aiming.holdPosition();
    //leftController.update();
    rightController.update();
    if(Robot.shooter.getRSpeed() >= 60){
      //Robot.transporter.setPower(0.25);
    }
  }

  @Override
  public void end(boolean interrupted) {
    Robot.shooter.setShootSpeed(0, 0);
    constants.setShootingFlag(false);
  }

  @Override
  public boolean isFinished() {
    double newTime = Timer.getFPGATimestamp();
    sumTime += newTime - oldTime;
    oldTime = newTime;
    return sumTime >= 3;
  }
}
