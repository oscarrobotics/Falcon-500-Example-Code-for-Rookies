package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Shuffleboard is the dashboard app that runs on your driver station laptop.
// A "GenericEntry" is a single live-updating cell on that dashboard — think
// of it like one labeled box that we keep overwriting with a fresh value
// every scheduler loop (every ~20ms). We create the entries ONCE up front
// (in the constructor) and then just keep calling .setString()/.setDouble()
// on the same entry objects from periodic() — creating brand new entries
// every loop would be wasteful and can even cause the dashboard to flicker.
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;

public class ActuatorSubsystem extends SubsystemBase {

  TalonFX motor1 = new TalonFX(12);

  private static final double kQuarterTurnRotations = 100;
  //JoyStick Go VROOM
  private static final double kJoystickDeadband = 0.1;

  private final PositionVoltage m_positionRequest = new PositionVoltage(0).withSlot(0);
  private final MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(1).withSlot(1);

  private String currentCommandName = "None";

  // Shuffleboard.getTab("Actuator") creates a new tab called
  // "Actuator" on the dashboard
  private final ShuffleboardTab m_actuatorTab = Shuffleboard.getTab("Actuator");

  // .add(name, defaultValue) creates a labeled widget on that tab
  //
  // .getEntry() hands us back a live handle we can keep writing new values
  // into.    | @overide  |
  // Check out| periodic()| at the bottom
  // for where the values actually get refreshed every loop.
  //
  //.add makes the widget, .getEntry is where the information is inserted into the widget
  private final GenericEntry m_activeCommandEntry =
      m_actuatorTab.add("Active Command", currentCommandName).getEntry();
  private final GenericEntry m_motorSpeedEntry =
      m_actuatorTab.add("Motor Speed (rps)", 0.0).getEntry();

    public ActuatorSubsystem() {
    // Build up one big settings object, then send it to the motor all at
    // once via the ".apply()" call at the bottom of this constructor.
    TalonFXConfiguration config = new TalonFXConfiguration();

    // --- PID Slot 0 ---
    // These three numbers control HOW AGGRESSIVELY the motor corrects
    // itself when it's not at the target position. Both PositionVoltage
    // and MotionMagicVoltage use these same Slot 0 gains internally —
    // Motion Magic just also wraps a smooth motion profile around them.
    //   kP (Proportional): how hard to push based on how far away from
    //       the target we currently are. Bigger error -> bigger push.
    //       This is normally the main gain you tune first.
    //   kI (Integral): corrects for small, persistent errors that kP
    //       alone never quite eliminates (e.g. a strong gravity load
    //       always pulling one way). Left at 0 here — many simple
    //       mechanisms don't need it, and it can cause oscillation if
    //       misused.
    //   kD (Derivative): pushes back against how FAST the position is
    //       currently changing, which helps reduce overshoot/wobble as
    //       the motor approaches the target (a bit like a shock absorber).
    //       
    //       I always found kD to be a little harder to understand, so 
    //       think of it as Approaching quickly -> brake harder. 
    //       approaching slowly -> barely touch the brake.

    //UNTUNED
    config.Slot0.kP = 25;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0.1;


    // --- Motion Magic settings ---
    // These only matter for MotionMagicVoltage (PositionVoltage ignores
    // them). They describe the "shape" of the smooth motion profile:
    //   CruiseVelocity: the top speed (in motor rotations per second)
    //       the profile is allowed to reach in the middle of the move.
    //
    //   Acceleration: how quickly (in rotations per second, per second)
    //       it's allowed to speed up to reach cruise speed, and slow
    //       down again as it nears the target.
    //
    //   Jerk: how quickly the ACCELERATION itself is allowed to change.
    //         (You'll learn about how its a derivative of acceleration 
    //          in physics, calculus, or right now!)
    //
    //       A nonzero jerk limit smooths out the "corners" where the
    //       profile transitions from speeding up to cruising, and from
    //       cruising to slowing down, instead of changing acceleration
    //       instantly. Setting this to 0 disables jerk limiting (i.e. the
    //       acceleration can change instantly, a plain trapezoid shape).

//Manual tuning typically follows this process:
//
// 1. Set all gains to zero.
// 2. Determine Kg if using an elevator or arm.
// 3. Select the appropriate Static Feedforward Sign for your closed-loop type.
// 4. Increase Ks until just before the motor moves.
// 5. If using velocity setpoints, increase Kv until the output velocity 
//    closely matches the velocity setpoints.
//
// 6. Increase Kp until the output starts to oscillate around the setpoint.
// 7. Increase Kd as much as possible without introducing jittering to the response.
config.Slot1.kG = 0;
config.Slot1.kS = 5; // Add Voltage output to overcome static friction
config.Slot1.kV = 0; // A velocity target of 1 rps results in 0.12 V output
config.Slot1.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
config.Slot1.kP = 3; // A position error of 2.5 rotations results in 12 V output
config.Slot1.kI = 0; // no output for integrated error
config.Slot1.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
    
    // UNTUNED! (Presets in documentation!)
    config.MotionMagic.MotionMagicCruiseVelocity = 500; // rotations/sec
    config.MotionMagic.MotionMagicAcceleration = 100;   // rotations/sec^2
    config.MotionMagic.MotionMagicJerk = 800;          // rotations/sec^3

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Actually send all of the settings above to the physical motor
    // controller over CAN. Nothing above this line has touched the real
    // hardware yet, this line applys the settings.
    motor1.getConfigurator().apply(config);
  }

  public void runMotor() {
    currentCommandName = "Run Motor (open-loop)";
    motor1.set(1);
  }

  public void stop() {
    currentCommandName = "Stopped";
    motor1.set(0);
  }
//JoyStick Go VROOM
  public void driveWithJoystick(double joystickValue) {
    currentCommandName = "Joystick Drive";
    double deadbanded = MathUtil.applyDeadband(joystickValue, kJoystickDeadband);
    motor1.set(deadbanded);
  }

  public void rotateNinetyDegrees() {
    currentCommandName = "Rotate 90 (PID)";
    double currentRotations = motor1.getPosition().getValueAsDouble();
    motor1.setControl(m_positionRequest.withPosition(currentRotations + kQuarterTurnRotations));
  }

  public void rotateNinetyDegreesMotionMagic() {
    currentCommandName = "Rotate 90 (Motion Magic)";
    double currentRotations = motor1.getPosition().getValueAsDouble();
    motor1.setControl(m_motionMagicRequest.withPosition(currentRotations + kQuarterTurnRotations));
  }

  public Command runMotorCommand() {
    return this.startEnd(this::runMotor, this::stop);
  }

  public Command rotateNinetyDegreesCommand() {
    return this.runOnce(this::rotateNinetyDegrees);
  }

  public Command rotateNinetyDegreesMotionMagicCommand() {
    return this.runOnce(this::rotateNinetyDegreesMotionMagic);
  }
// joyStick Go VROOM!
  public Command driveWithJoystickCommand(DoubleSupplier joystickSupplier) {
    return this.run(() -> driveWithJoystick(joystickSupplier.getAsDouble()));
  }

  @Override
  public void periodic() {
    m_activeCommandEntry.setString(currentCommandName);

    // motor1.getVelocity() reads the TalonFX's built-in encoder and reports
    // ACTUAL measured rotor speed (in rotations per second, "rps"), not
    // just whatever we last told it to do. This is the real, live speed
    // the motor is spinning at, which is more useful for diagnosing "is it
    // actually moving?" than the raw -1..1 power level would be.
    double currentSpeedRotationsPerSecond = motor1.getVelocity().getValueAsDouble();
    m_motorSpeedEntry.setDouble(currentSpeedRotationsPerSecond);
  }
}