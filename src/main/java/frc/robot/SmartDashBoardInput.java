/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class SmartDashBoardInput {
    public void logs(){
        Robot.driveTrain.logs();
        Robot.limelight.logs();
        Robot.shooter.logs();
        Robot.aiming.logs();
        Robot.transporter.logs();

    }
}
