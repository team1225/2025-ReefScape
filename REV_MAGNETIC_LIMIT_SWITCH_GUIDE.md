# REV Magnetic Limit Switch Integration Guide

This guide explains how to connect and use the REV Magnetic Limit Switch with your FRC robot code.

## Hardware Overview

The REV Magnetic Limit Switch is a **digital sensor** (not I2C) that detects the presence of a magnetic field. Despite having 4 pins, it only uses 3 for digital input functionality.

### Pin Configuration
- **Pin 1 (VCC)**: Power input (5V)
- **Pin 2 (GND)**: Ground
- **Pin 3 (Signal)**: Digital output signal
- **Pin 4 (Unused)**: Not connected

## Wiring Instructions

### Connecting to RoboRIO DIO Port

The REV Magnetic Limit Switch connects to a 3-pin Digital Input/Output (DIO) port on the RoboRIO:

```
REV Magnetic Limit Switch    →    RoboRIO DIO Port
Pin 1 (VCC/Power)           →    Pin 2 (5V Power) - Red wire
Pin 2 (GND/Ground)          →    Pin 3 (Ground) - Black wire  
Pin 3 (Signal)              →    Pin 1 (Signal) - White wire
Pin 4 (Unused)              →    Not connected
```

### Cable Requirements
- Use a standard 3-pin PWM cable
- Ensure proper wire colors: Red (5V), Black (Ground), White (Signal)
- Verify connections are secure

## Code Implementation

### 1. Port Configuration

The DIO port has been added to `Ports.java`:

```java
public static class Digital {
    public static final int MAGNETIC_LIMIT_SWITCH = 0; // DIO port
}
```

### 2. Sensor Class

The `MagneticLimitSwitch` class has been created with the following features:

```java
// Initialize the sensor
MagneticLimitSwitch limitSwitch = new MagneticLimitSwitch(); // Uses port from Ports.java
// OR
MagneticLimitSwitch limitSwitch = new MagneticLimitSwitch(0); // Specify DIO port directly
```

### 3. Integration with RobotContainer

The sensor is integrated into `RobotContainer.java`:

```java
private final MagneticLimitSwitch magneticLimitSwitch = new MagneticLimitSwitch();

public MagneticLimitSwitch getMagneticLimitSwitch() {
    return magneticLimitSwitch;
}
```

## Usage Examples

### Basic Usage

```java
// Get the sensor from RobotContainer
MagneticLimitSwitch limitSwitch = robotContainer.getMagneticLimitSwitch();

// Check if limit switch is pressed (magnet detected)
if (limitSwitch.isPressed()) {
    System.out.println("Magnet detected - limit reached!");
    // Stop motor or perform other actions
}

// Check if limit switch is not pressed
if (limitSwitch.isNotPressed()) {
    System.out.println("No magnet detected - safe to move");
}
```

### Motor Control with Limit Switch

```java
// Example: Move motor until limit switch is pressed
public void moveUntilLimit() {
    while (!limitSwitch.isPressed()) {
        motor.set(0.5); // 50% power
    }
    motor.set(0); // Stop when limit is reached
}

// Example: Safe movement with limit switch protection
public void safeMove(double speed) {
    if (!limitSwitch.isPressed()) {
        motor.set(speed);
    } else {
        motor.set(0); // Don't move if at limit
    }
}
```

### Using with Commands

```java
// Create a command that moves until limit switch
Command moveToLimit = new Command() {
    @Override
    public void execute() {
        if (!limitSwitch.isPressed()) {
            pivotArm.setVoltage(3.0);
        } else {
            pivotArm.setVoltage(0.0);
        }
    }
    
    @Override
    public boolean isFinished() {
        return limitSwitch.isPressed();
    }
    
    @Override
    public void end(boolean interrupted) {
        pivotArm.setVoltage(0.0);
    }
};
```

### Button Binding Example

```java
// In RobotContainer's configureBindings() method
driverController.a().whileTrue(
    Commands.run(() -> {
        if (!magneticLimitSwitch.isPressed()) {
            pivotArm.setVoltage(2.0);
        } else {
            pivotArm.setVoltage(0.0);
        }
    })
);
```

## Key Features

### State Reading Methods

```java
// Raw digital input state (true = no magnet, false = magnet present)
boolean rawState = limitSwitch.getRawDigitalInput();

// Inverted state (true = magnet present, false = no magnet) - More intuitive
boolean invertedState = limitSwitch.getInvertedState();

// Debounced state (recommended for motor control)
boolean debouncedState = limitSwitch.getDebouncedState();

// Convenience methods
boolean isPressed = limitSwitch.isPressed();        // Same as getDebouncedState()
boolean isNotPressed = limitSwitch.isNotPressed();  // Opposite of isPressed()
```

### Debouncing
The sensor includes built-in debouncing (50ms) to prevent false triggers from electrical noise:

```java
// Use debounced state for reliable motor control
boolean stableState = limitSwitch.getDebouncedState();
```

### Connection Testing
```java
// Check if sensor is connected and working
if (limitSwitch.isConnected()) {
    System.out.println("Magnetic limit switch is connected");
} else {
    System.out.println("Magnetic limit switch connection issue");
}
```

## How the Sensor Works

### Electrical Behavior
- **No Magnet Present**: DigitalInput returns `true` (circuit open, 5V)
- **Magnet Present**: DigitalInput returns `false` (circuit closed, 0V)

### Code Inversion
The `MagneticLimitSwitch` class inverts this behavior for more intuitive use:
- **No Magnet Present**: `isPressed()` returns `false`
- **Magnet Present**: `isPressed()` returns `true`

## Best Practices

1. **Use Debounced State**: Always use `isPressed()` or `getDebouncedState()` for motor control
2. **Check Before Moving**: Always check the limit switch state before moving motors
3. **Stop on Limit**: Immediately stop motors when the limit switch is activated
4. **Test Thoroughly**: Test the sensor in various conditions
5. **Use in Commands**: Integrate limit switch logic into your command structure

## Troubleshooting

### Sensor Not Responding
- Check wiring connections (VCC, GND, Signal)
- Verify power supply (5V)
- Test with multimeter
- Check DIO port number in code

### False Triggers
- Use debounced state instead of raw state
- Check for electrical interference
- Verify magnet positioning
- Ensure proper grounding

### Unexpected Behavior
- Remember the sensor inverts the digital input
- Use `getRawDigitalInput()` to see actual hardware state
- Use `getInvertedState()` for intuitive magnet detection

## Example Integration with PivotArm

Here's how you might integrate the limit switch with your existing PivotArm:

```java
// In your PivotArm class
private final MagneticLimitSwitch limitSwitch = new MagneticLimitSwitch();

public void manualControl(double speed) {
    // Check limit switch before moving
    if (speed > 0 && limitSwitch.isPressed()) {
        // Trying to move forward but at limit - stop
        setVoltage(0.0);
        System.out.println("Cannot move forward - at magnetic limit");
    } else if (speed < 0 && limitSwitch.isPressed()) {
        // Trying to move backward but at limit - stop  
        setVoltage(0.0);
        System.out.println("Cannot move backward - at magnetic limit");
    } else {
        // Safe to move
        setVoltage(speed * 12.0);
    }
}
```

## Summary

The REV Magnetic Limit Switch is now properly integrated into your robot code as a digital sensor. It provides reliable limit detection and can be easily used to safely control your motors and prevent mechanical damage.

**Key Points:**
- Uses DigitalInput (not I2C) despite 4-pin connector
- Only 3 pins are used: VCC, GND, Signal
- Sensor inverts digital input for intuitive use
- Includes debouncing for reliable operation
- Ready to use with your existing command structure
