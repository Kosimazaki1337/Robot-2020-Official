/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.Commands.Aiming.OpAim;
import frc.robot.motion.SpeedPID;

public class Aiming extends SubsystemBase {
  private WPI_VictorSPX aimMotor;
  private AnalogPotentiometer potentiometer;
  private SpeedPID aimPID;

  private double positionToHold = 0;

  private double positionToHold2 = 0;


  Constants constants;

  public Aiming(){
    aimMotor = new WPI_VictorSPX(PortMap.kAimMotor);

    potentiometer = new AnalogPotentiometer(PortMap.kPotentiometer);
    constants = new Constants().getConstants();

    aimMotor.configFactoryDefault();

    aimPID = new SpeedPID(0.2, 0.001, 0.01, 0.0);
  }

  @Override
  public void periodic() {
    opAim();
    getShootingState();
    //checkPotentiometerPosition();
  }

  public void stopMotor(){
    aimMotor.set(ControlMode.PercentOutput, 0);
  }

  public void setAimSpeed(double speed){

    if(speed > 0.25) {
      speed = 0.25;
    }else if(speed < -0.6) {
      speed = -0.6;
    }

    double multipler = 1.1;
    double minOutputTurn = 0.05;
    if(speed < minOutputTurn && speed >= 0){
			speed = minOutputTurn;
		}else if(speed > -minOutputTurn && speed <= 0){
			speed = -minOutputTurn*multipler;
		}


    if (potentiometer.get() < 0.48 && speed > 0){
      aimMotor.set(ControlMode.PercentOutput, -speed);
    }else if(potentiometer.get() > 0.30 && speed < 0){
      aimMotor.set(ControlMode.PercentOutput, -speed);
    }else{
      aimMotor.set(0);
    }

    SmartDashboard.putNumber("aimSpeed", speed);
    
  }

  public void opAim(){
    if (Robot.oi.getOperatorJoystick().getRawAxis(2) != 0){
      Robot.aiming.setAimSpeed(Robot.oi.getOperatorJoystick().getRawAxis(2)/1.5);
    }else if (Robot.oi.getOperatorJoystick().getRawAxis(3) != 0){
      Robot.aiming.setAimSpeed(-Robot.oi.getOperatorJoystick().getRawAxis(3)/2);
    }else Robot.aiming.setAimSpeed(0.0);
  }

  public double getAngle(){
    positionToHold = potentiometer.get();
    double positionRounded = Math.round(positionToHold*1000);
    positionToHold = positionRounded/10;
    return positionToHold;
  }

  public boolean getShootingState(){
    return constants.getShootingFlag();
  }

  public double getPotentometerPosition(){
    return potentiometer.get();
  }

  public void logs(){
    SmartDashboard.putNumber("aiming_ActualPosition", potentiometer.get());
    SmartDashboard.putNumber("AimMotorPercent", aimMotor.getMotorOutputPercent());
  }

  public void setpositionToHold2(){
    positionToHold2 = getPotentometerPosition();
  }

  public double getPositionToHold2(){
    return positionToHold2;
  }
}
