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
 // private SpeedPID aimPID;

  private double positionToHold = 0;

  private double positionToHold2 = 0.42;

  private boolean isAuto = false;

  Constants constants;

  public Aiming(){
    aimMotor = new WPI_VictorSPX(PortMap.kAimMotor);

    potentiometer = new AnalogPotentiometer(PortMap.kPotentiometer);
    constants = new Constants().getConstants();

    aimMotor.configFactoryDefault();

    //aimPID = new SpeedPID(0.2, 0.001, 0.01, 0.0);
  }

  @Override
  public void periodic() {

    double increment = 0.003;

    SmartDashboard.putBoolean("IsAuto", isAuto);

    // error = 
    //Robot.aiming.setAimSpeed(-error * Constants.getConstants().aimingkP);

    if(isAuto){
        positionLoop();
    } else {
        if (Robot.oi.getOperatorJoystick().getRawButton(6)){
          ourSet(-0.23);
        } else if (Robot.oi.getOperatorJoystick().getRawButton(5)){
          ourSet(0.24);
        } else {
          ourSet(0);
        }
    }
      SmartDashboard.putNumber("positionToHold2", positionToHold2);
  }
  
  
  public void stopMotor(){
    aimMotor.set(ControlMode.PercentOutput, 0);
  }

  private void positionLoop(){
          // 0.30 na gorze
      // 0.48 na dole
      double actualPosition = Robot.aiming.getPotentometerPosition();
      
      // positionToHold2 == target == setPoint
      double setPoint = Robot.aiming.getPositionToHold2();
  
      double error = actualPosition - setPoint; // 0.48 - 0.38 = 0.1
      // 0.42 - 0.38 = 0.04

      if(Math.abs(error) < Constants.getConstants().aimMinPotenciometrAllowError){
        ourSet(0);          
      } else {
        if(error > 0){
          ourSet(0.38);
        } else if(error < 0){
          ourSet(0.12);
        }
      }

  }

  public void setAimSpeed(double speed){

    if(speed > 0.35) {
      speed = 0.35;
    }else if(speed < -0.1) {
      speed = -0.1;
    }

    double multipler = 2.35;
    double minOutputTurn = 0.13;

    if(speed < minOutputTurn/3 && speed >= 0){
			speed = minOutputTurn/3;
		}else if(speed > -minOutputTurn*multipler && speed <= 0){
      speed = -minOutputTurn*multipler;
		}

    //MAXOUTPUT
    if(speed > 0.22) {
      speed = 0.22;
    }
    if(speed < -0.05) {
      speed = -0.05;
    }

    
    
    aimMotor.set(speed);    
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
  
  public void setpositionToHold2(double pos){
    positionToHold2 = pos;
  }

  public double getPositionToHold2(){
    return positionToHold2;
  }
  private void ourSet(double _speed){
    if (potentiometer.get() > 0.48){
      // ogranicznik dolu
      if(_speed < 0){
        _speed = 0;
      }
    }else if(potentiometer.get() < 0.30){
      if(_speed > 0){
        _speed = 0;
      } 
    }
    aimMotor.set(_speed);
  }


  public void setMode(boolean _isAuto){
    isAuto = _isAuto;
  }
}
    

