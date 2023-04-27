// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /* the next two lines declare the right side motor controller objects */
  CANSparkMax m_rightMotor1;
  CANSparkMax m_rightMotor2;

  /* the next two lines declare the left side motor controller objects */
  CANSparkMax m_leftMotor1;
  CANSparkMax m_leftMotor2;

  /* declares a DifferentialDrive object */
  DifferentialDrive m_differentialDrive; 

  /* declares a new SlewRateLimiter object 
    we are using this to limit the acceleration of the drivetrain which results in smooth driving */
  SlewRateLimiter m_rateLimit = new SlewRateLimiter(0, 0, 0); //TODO

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    //initialize and configure right motor controllers
    m_rightMotor1 = new CANSparkMax(0, MotorType.kBrushless);
    m_rightMotor2 = new CANSparkMax(1, MotorType.kBrushless);
    m_rightMotor2.follow(m_rightMotor1, false);

    //initialize and configure left motor controllers
    m_leftMotor1 = new CANSparkMax(2, MotorType.kBrushless);
    m_leftMotor2 = new CANSparkMax(3, MotorType.kBrushless);
    m_leftMotor2.follow(m_leftMotor1, false);

    //intialize DifferentialDrive class with "leader" motor controller
    m_differentialDrive = new DifferentialDrive(m_rightMotor1, m_leftMotor1); 
  }

  public CommandBase arcadeDriveCommand(double forward, double rotation) {
    return run(() -> m_differentialDrive.arcadeDrive(m_rateLimit.calculate(forward), rotation));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
