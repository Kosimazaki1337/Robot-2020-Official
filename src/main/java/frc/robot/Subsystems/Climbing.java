/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Climbing extends SubsystemBase {
  private VictorSP climbMotorLeft, climbMotorRight;
  private Servo climbLeftServo, climbRightServo;

  public Climbing() {
    climbMotorLeft = new VictorSP(2);
    climbMotorRight = new VictorSP(3);

    climbLeftServo = new Servo(4);
    climbRightServo = new Servo(5);

    climbMotorLeft.setInverted(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("leftServo", climbLeftServo.getAngle());
    SmartDashboard.putNumber("rightServo", climbRightServo.getAngle());

    joyServo();
    
  }

  public void setPower(double speed){
    climbMotorRight.set(speed);
    climbMotorLeft.set(speed);
  }

  public void setServoAngle(double angle){
    climbRightServo.setAngle(angle);
    climbLeftServo.setAngle(angle);
  }

  public void joyServo(){
    climbLeftServo.set(Robot.oi.getOperatorJoystick().getRawAxis(1)/2);
    climbRightServo.set(Robot.oi.getOperatorJoystick().getRawAxis(1)/2);
  }

}
