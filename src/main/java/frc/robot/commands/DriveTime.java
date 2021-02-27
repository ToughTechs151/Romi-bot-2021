// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveTime extends CommandBase {
  private final double m_duration;
  private final double m_speedleft1;
  private final double m_speedleft2;
  private final double m_speedright1;
  private final double m_speedright2;
  private final Drivetrain m_drive;
  private double currspeedleft;
  private double currspeedright;
  private long currtime;
  private long m_startTime;

  /**
   * Creates a new DriveTime. This command will drive your robot for a desired
   * speed and time.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param time  How much time to drive in seconds
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveTime(double speedright1, double speedright2, double speedleft1, double speedleft2, double time,
      Drivetrain drive) {
    m_speedleft1 = speedleft1;
    m_speedleft2 = speedleft2;
    m_speedright1 = speedright1;
    m_speedright2 = speedright2;
    m_duration = time * 1000;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.tankDrive(m_speedleft1,m_speedright1);
    m_startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currtime = System.currentTimeMillis() - m_startTime;
    currspeedright = (((m_speedright2 - m_speedright1) / (m_duration*m_duration)) * (currtime * currtime)) + m_speedright1;
    currspeedleft = (((m_speedleft2 - m_speedleft1) / (m_duration*m_duration)) * (currtime * currtime)) + m_speedleft1;
    m_drive.tankDrive(currspeedleft, currspeedright);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.tankDrive(currspeedleft, currspeedright);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - m_startTime) >= m_duration;
  }
}
