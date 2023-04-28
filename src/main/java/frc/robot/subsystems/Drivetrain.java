// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /*
   * Declares four CANSparkMax objects, each corresponding to a specific motor.
   * We will use these objects to give directions our motors.
   * Learn more about motor controllers here:
   * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/using-
   * motor-controllers.html#can-motor-controllers
   */
  CANSparkMax m_rightMotor1;
  CANSparkMax m_rightMotor2;

  CANSparkMax m_leftMotor1;
  CANSparkMax m_leftMotor2;

  /*
   * Declares a DifferentialDrive object.
   * Learn more about DifferentialDrive here:
   * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-
   * drive-classes.html#differential-drive-robots
   */
  DifferentialDrive m_differentialDrive;

  /*
   * Declares a new SlewRateLimiter object.
   * We are using this class to limit the acceleration of the drivetrain which
   * results in smooth driving.
   * Learn more about SlewRateLimiter here:
   * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/
   * slew-rate-limiter.html
   */
  SlewRateLimiter m_rateLimit = new SlewRateLimiter(0, 0, 0); // TODO

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    /*
     * Initializes the CANSparkMaxs using a unique CAN ID for each motor.
     * Most motors used in FRC (REV NEOs, CTRE Falcon 500) are brushless.
     */
    m_rightMotor1 = new CANSparkMax(0, MotorType.kBrushless);
    m_rightMotor2 = new CANSparkMax(1, MotorType.kBrushless);

    m_leftMotor1 = new CANSparkMax(2, MotorType.kBrushless);
    m_leftMotor2 = new CANSparkMax(3, MotorType.kBrushless);

    /*
     * Configuring the second motor to "follow" a "leader" motor on the same side.
     * The follower will produce the same output without any additional code
     * The invert boolean specifies if the follower should invert the output.
     * Inverting the output is useful if the motors are geared or mounted to spin
     * opposite of eachother.
     */
    m_rightMotor2.follow(m_rightMotor1, false);
    m_leftMotor2.follow(m_leftMotor1, false);

    /*
     * Initialize a DifferentialDrive using the "leading" motors on each side.
     * We do not need to include the follower motors because they will copy the
     * calculated output of the leader.
     */
    m_differentialDrive = new DifferentialDrive(m_rightMotor1, m_leftMotor1);
  }

  /*
   * This method returns a new RunCommand which contiuously executes
   * DifferentialDrive's arcadeDrive() method.
   * In our case, arcadeDrive() takes joystick inputs as parameters.
   * Learn more about Commands here:
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html
   */
  public CommandBase arcadeDriveCommand(DoubleSupplier forward, DoubleSupplier rotation) {
    CommandBase driveCommand = new RunCommand(
        () -> m_differentialDrive.arcadeDrive(m_rateLimit.calculate(forward.getAsDouble()), rotation.getAsDouble()));
    driveCommand.addRequirements(this);
    return driveCommand;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
