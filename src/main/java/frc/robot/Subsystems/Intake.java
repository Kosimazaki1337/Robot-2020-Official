/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
  public WPI_VictorSPX intakeMotor;

  public DoubleSolenoid intakeSolenoid;
  public Compressor compressor;

  private double intakeSpeed = 0.5;

  public boolean isAutonomous = false;

  public enum Flag{
    START, STOP
  }

  public Flag state = Flag.START;

  public Intake(){
    intakeMotor = new WPI_VictorSPX(PortMap.kIntakeMotor);

    intakeMotor.configFactoryDefault();

    intakeSolenoid = new DoubleSolenoid(PortMap.kIntakeSolenoidA, PortMap.kIntakeSolenoidB);
    compressor = new Compressor();
    //compressor.stop();
  }

  @Override
  public void periodic() {

      if(Robot.oi.getOperatorJoystick().getRawAxis(2) > 0.1){
        intakeMotor.set(-Robot.oi.getOperatorJoystick().getRawAxis(2)/2);
        isAutonomous = false;
      }else if(Robot.oi.getOperatorJoystick().getRawAxis(3) > 0.1){
        intakeMotor.set(Robot.oi.getOperatorJoystick().getRawAxis(3)/1.8);
        isAutonomous = false;
      }else if(isAutonomous){
        intakeMotor.set(0.55);
      }
        else{
        intakeMotor.set(0);
      }

    
  }

  public void setPower(double speed){
    if(Robot.transporter.isBallDown()){
      intakeMotor.set(ControlMode.PercentOutput, speed/2.5);
    } else {
      intakeMotor.set(ControlMode.PercentOutput, speed);
    }
  }

  public void stopBalls(){
    intakeMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void intakeUp(){
    intakeSolenoid.set(Value.kForward);
  }

  public void intakeDown(){
    intakeSolenoid.set(Value.kReverse);
  }

  public void intakeOff(){
    intakeSolenoid.set(Value.kOff);
  }

  public Flag getIntakeFlag(){
    return state;
  }

  public void changeIntakeFlag(Flag flag){
    state = flag;
  }

  public void changeIntakeSpeed(double speed){
    intakeSpeed = speed;
  }
}
