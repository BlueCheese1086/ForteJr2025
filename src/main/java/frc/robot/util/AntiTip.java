// package frc.robot.util;

// import static edu.wpi.first.units.Units.Inches;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import java.util.function.Consumer;
// import java.util.function.Supplier;

// public class AntiTip extends Command {
//     private Consumer<Distance> setHeight;
//     private Supplier<Angle> pitchSupplier;
//     private Supplier<Angle> rollSupplier;

//     /**
//      * Creates a new {@link AntiTip} command.
//      * It checks the roll and pitch of the robot and slams the elevator down if it is above some threshold.
//      * 
//      * @param setHeight A function that sets the height of the robot's elevator.
//      * @param pitchSupplier A function that gets the pitch of the robot.
//      * @param rollSupplier A function that gets the roll of the robot.
//      */
//     public AntiTip(Consumer<Distance> setHeight, Supplier<Angle> pitchSupplier, Supplier<Angle> rollSupplier) {
//         this.setHeight = setHeight;
//         this.pitchSupplier = pitchSupplier;
//         this.rollSupplier = rollSupplier;
//     }

//     /**
//      * Creates a new {@link AntiTip} command
//      */
//     public AntiTip(Consumer<ChassisSpeeds> drive, Supplier<Angle> pitchSupplier, Supplier<Angle> rollSupplier) {
//         this.pitchSupplier = pitchSupplier;
//         this.rollSupplier = rollSupplier;
//     }

//     /** Called every time the scheduler runs while the command is scheduled. */
//     @Override
//     public void execute() {
//         if (pitchSupplier.get().gte(Constants.TipThreshold) || rollSupplier.get().gte(Constants.TipThreshold)) {
//             setHeight.accept(Inches.zero());
//         }
//     }
// }