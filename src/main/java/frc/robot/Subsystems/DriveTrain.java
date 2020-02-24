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
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;

import frc.robot.Commands.Drive.*;

public class DriveTrain extends SubsystemBase {

  public enum ControlState {
    OPEN_LOOP, PATH_FOLLOWING, AIMING
  }

  private final Constants constants;

  private WPI_TalonSRX lMaster, rMaster;
  private WPI_VictorSPX lSlaveM, lSlaveB, rSlaveM, rSlaveB;

  private AHRS gyro;

  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveOdometry mOdometry;
  private Pose2d pose;

  private SimpleMotorFeedforward feedforward;

  private PIDController leftPidController;
  private PIDController righPidController;

  public ControlState mState = ControlState.OPEN_LOOP;

  public DriveTrain() {
    lMaster = new WPI_TalonSRX(PortMap.kLMasterDrive);
    rMaster = new WPI_TalonSRX(PortMap.kRMasterDrive);

    lSlaveM = new WPI_VictorSPX(PortMap.kLSlaveMDrive);
    lSlaveB = new WPI_VictorSPX(PortMap.kLSlaveBDrive);
    rSlaveM = new WPI_VictorSPX(PortMap.kRSlaveMDrive);
    rSlaveB = new WPI_VictorSPX(PortMap.kRSlaveBDrive);

    configureMaster(lMaster, false);
    configureMaster(rMaster, false);

    lSlaveB.follow(lMaster);
    lSlaveM.follow(lMaster);
    rSlaveB.follow(rMaster);
    rSlaveM.follow(rMaster);

    gyro = new AHRS(PortMap.gyroPort);

    constants = new Constants().getConstants();

    kinematics = new DifferentialDriveKinematics(constants.kTrackWidthMeters);
    mOdometry = new DifferentialDriveOdometry(getHeading(), new Pose2d());
    feedforward = new SimpleMotorFeedforward(constants.kS, constants.kV, constants.kA);

    leftPidController = new PIDController(constants.kP, constants.kI, constants.kD);
    righPidController = new PIDController(constants.kP, constants.kI, constants.kD);
    stopAndReset();
  }

  public void stopAndReset(){
    gyro.reset();
    setSpeed(0.0, 0.0);
    resetEncoders();
  }


  public void configureMaster(WPI_TalonSRX talon, boolean invert){
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
    final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice
            .CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
    if (sensorPresent != ErrorCode.OK) {
        DriverStation.reportError("Could not detect " + (invert ? "right" : "left") + " encoder: " + sensorPresent, false);
    }
    talon.setSensorPhase(true);
    talon.enableVoltageCompensation(true);
    talon.configVoltageCompSaturation(12.0, constants.kLongCANTimeoutMs);
    talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, constants.kLongCANTimeoutMs);
    talon.configVelocityMeasurementWindow(1, constants.kLongCANTimeoutMs);
    talon.configClosedloopRamp(constants.kDriveVoltageRampRate, constants.kLongCANTimeoutMs);
    talon.configNeutralDeadband(0.04, 0);
  }

  @Override
  public void periodic() {
    pose = mOdometry.update(getHeading(), getLeftDistance(), getRightDistance());

    if(mState == ControlState.OPEN_LOOP){
      setDefaultCommand(new OpenLoopDrive());
    }

  }

  public void setSpeed(double leftSpeed, double rightSpeed){
    lMaster.set(ControlMode.PercentOutput, leftSpeed);
    rMaster.set(ControlMode.PercentOutput, rightSpeed);
  }

  public void resetEncoders(){
    lMaster.setSelectedSensorPosition(0);
    rMaster.setSelectedSensorPosition(0);
  }

  public double getLeftSpeed(){
    return (Math.PI*0.19)*(lMaster.getSelectedSensorVelocity())/4096*10;
  }

  public double getRightSpeed(){
    return -(Math.PI*0.19)*(rMaster.getSelectedSensorVelocity())/4096*10;
  }

  public DifferentialDriveWheelSpeeds getSpeeds(){
    return new DifferentialDriveWheelSpeeds(
    getLeftSpeed(), getRightSpeed());
  }

  public int getLeftDistance(){
    return lMaster.getSelectedSensorPosition();
  }

  public int getRightDistance(){
    return rMaster.getSelectedSensorPosition();
  }

  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public SimpleMotorFeedforward getFeedForward(){
    return feedforward;
  }

  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }

  public Pose2d getPose(){
    return pose;
  }

  public PIDController getLeftPidController(){
    return leftPidController;
  }

  public PIDController getRightPidController(){
    return righPidController;
  }

  public void changeControlState(ControlState state){
    mState = state;
  }

  public void logs() {
    SmartDashboard.putNumber("dirveTrain_leftEncoderTicks", getLeftDistance());
    SmartDashboard.putNumber("dirveTrain_rightEncoderTicks", getRightDistance());
    SmartDashboard.putNumber("driveTrain_SlaveSpeed", lSlaveM.getMotorOutputVoltage());
    SmartDashboard.putNumber("driveTrain_LeftSideVoltage", lMaster.getMotorOutputPercent());
    SmartDashboard.putNumber("driveTrain_RightSideVoltage", rMaster.getMotorOutputPercent());
    SmartDashboard.putNumber("driveTrain_LeftSideVelocity", getLeftSpeed());
    SmartDashboard.putNumber("driveTrain_RightSideVelocity", getRightSpeed());
    SmartDashboard.putNumber("gyroAngle", gyro.getAngle());
  }
}
