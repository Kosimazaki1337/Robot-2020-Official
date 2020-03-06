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
import frc.robot.Subsystems.Intake.Flag;
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

  boolean button = false;
  boolean shootFlag = false;
  double shootingTime;

  public Shoot(double shootngTime) {
    this.shootingTime = shootingTime;
    addRequirements(Robot.shooter);
    addRequirements(Robot.leds);
    addRequirements(Robot.limelight);

    button = false;
    constants = new Constants();
  }

  public Shoot() {
    shootingTime = 0;
    button = true;
    addRequirements(Robot.shooter);
    addRequirements(Robot.leds);
    addRequirements(Robot.limelight);

    constants = new Constants();
  }

  @Override
  public void initialize() {


    rProfile = new TrapezoidalMotionProfile(constants.maxShootSpeed, constants.maxShootVelocity, constants.maxShootAcceleration);
    lProfile = new TrapezoidalMotionProfile(constants.maxShootSpeed, constants.maxShootVelocity, constants.maxShootAcceleration);

    leftController = new ShooterLeftController(lProfile);
    rightController = new ShooterRightController(rProfile);
    leftController.reset();
    rightController.reset();

    oldTime = Timer.getFPGATimestamp();
    sumTime = 0;
    
    Robot.shooter.changeShootState(true);
  }

  @Override
  public void execute() {
    rightController.update();
    leftController.update();

    if(Math.abs(Robot.shooter.getRSpeed()) >= Math.abs(constants.maxShootSpeed-Constants.allowedShooterError) && Math.abs(Robot.shooter.getLSpeed()) >= Math.abs(constants.maxShootSpeed-Constants.allowedShooterError)){
      Robot.shooter.changeShootState(true);
      Robot.intake.setPower(0.30);
    } else if((Robot.shooter.getRSpeed() == 0 || Robot.shooter.getLSpeed() == 0) && (Math.abs(Robot.shooter.getRSpeed()) >= Math.abs(constants.maxShootSpeed-Constants.allowedShooterError) || Math.abs(Robot.shooter.getLSpeed()) >= Math.abs(constants.maxShootSpeed-Constants.allowedShooterError))){
      Robot.shooter.changeShootState(true);
      Robot.intake.setPower(0.30);
    }else {
      Robot.shooter.changeShootState(false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shootFlag = false;
    Robot.shooter.setShootSpeed(0, 0);
    Robot.shooter.changeShootState(false);
    Robot.intake.changeIntakeFlag(Flag.START);
    Robot.intake.stopBalls();
    Robot.leds.turnOFF();
    Robot.transporter.resetBalls();
    Robot.limelight.changePipeline(0);
  }

  @Override
  public boolean isFinished() {
    double newTime = Timer.getFPGATimestamp();
    sumTime += newTime - oldTime;
    oldTime = newTime;
    SmartDashboard.putNumber("SumTime", sumTime);
    if(button == true){
      return false;
    }else{
      return sumTime >= shootingTime || !Robot.limelight.isTargetVisible();
    }
  }
}
