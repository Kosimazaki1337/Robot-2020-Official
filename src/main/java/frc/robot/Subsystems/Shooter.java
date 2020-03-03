/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {
  public WPI_TalonSRX lMaster, rMaster;
  public WPI_VictorSPX lSlave, rSlave;

  Constants constants;

  boolean shoot = false;

  public Shooter(){
    lMaster = new WPI_TalonSRX(PortMap.kLMasterShooter);
    rMaster = new WPI_TalonSRX(PortMap.kRMasterShooter);
    lSlave = new WPI_VictorSPX(PortMap.kLSlaveShooter);
    rSlave = new WPI_VictorSPX(PortMap.kRSlaveShooter);

    configureMaster(lMaster, false);
    configureMaster(rMaster, false);

    lSlave.follow(lMaster);
    rSlave.follow(rMaster);

    constants = new Constants();

    resetEncoderSHR();

  }

  @Override
  public void periodic() {
    //steer();
  }

  private void configureMaster(WPI_TalonSRX talon, boolean left){
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
    final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice
            .CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
    if (sensorPresent != ErrorCode.OK) {
        DriverStation.reportError("Could not detect " + (left ? "left" : "right") + " encoder: " + sensorPresent, false);
    }
 
    talon.setSensorPhase(true);
    talon.enableVoltageCompensation(true);
    talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
    talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs);
    talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);
    talon.configClosedloopRamp(4.0, Constants.kLongCANTimeoutMs);
    talon.configNeutralDeadband(0.04, 0);
}

  public void steer(){
    lMaster.set(ControlMode.PercentOutput, Robot.oi.getDriverJoystick().getRawAxis(1)/2);
    rMaster.set(ControlMode.PercentOutput, Robot.oi.getDriverJoystick().getRawAxis(1)/2);
  }

  public void setShootSpeed(double left, double right){
    lMaster.set(ControlMode.PercentOutput, left);
    rMaster.set(ControlMode.PercentOutput, right);
  }

  public void setLeftSpeed(double speed){
    Robot.leds.ledShooterSpeedLeft();
    lMaster.set(ControlMode.PercentOutput, speed);
  }

  public void setRightSpeed(double speed){
    Robot.leds.ledShooterSpeedRight();
    rMaster.set(ControlMode.PercentOutput, speed);
  }

  public void stopMotors(){
    lMaster.set(ControlMode.PercentOutput, 0.0);
    rMaster.set(ControlMode.PercentOutput, 0.0);
  }

  public double getLeftSpeed(){
    return lMaster.getMotorOutputPercent();
  }

  public double getRightSpeed(){
    return rMaster.getMotorOutputPercent();
  }

  public void resetEncoderSHR(){
    lMaster.setSelectedSensorPosition(0);
    rMaster.setSelectedSensorPosition(0);
  }

  public double getLSpeed(){
    return -(((double)lMaster.getSelectedSensorVelocity()/4096)*10)/3;
  }

  public double getRSpeed(){
    return (((double)rMaster.getSelectedSensorVelocity()/4096)*10)/3;
  }

  public void changeShootState(boolean state){
    shoot = state;
  }

  public boolean getShootState(){
    return shoot;
  }

  public void logs(){
    SmartDashboard.putNumber("shooter_RightEncoder", lMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("shooter_LeftEncoder", rMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("shooter_LeftRPS", getLSpeed());
    SmartDashboard.putNumber("shooter_RightRPS", getRSpeed());
  }
}
