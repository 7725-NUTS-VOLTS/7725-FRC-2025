package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//waiting for hardware to complete code
public class TestNeumatica extends SubsystemBase {
      // Compressor connected to a PCM with a default CAN ID (0)
    private final Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    private DoubleSolenoid doubleSolenoid;

  /**
   * Creates a new PneumaticsSubsystem.
   */
  public TestNeumatica() {
    super(); //add this
    try {
      doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 5);
    } catch(Exception e) {
      e.printStackTrace();
    }
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extendPiston() {
    //Should extend the piston
    doubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retractPiston() {
    //Should retract the piston
    doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
}

