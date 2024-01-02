#include <stdio.h>
#include <unistd.h>

// PID Controller parameters
#define Kp 2.0
#define Ki 0.2
#define Kd 0.2

// Simulation parameters
#define dt 0.1  // Time step

// Function to compute the PID control output
double computePID(double setpoint, double current_position, double *prev_error, double *integral) {
    double error = setpoint - current_position;
    double derivative = (error - *prev_error) / dt;

    *integral += error * dt;

    // PID control equation
    double output = Kp * error + Ki * (*integral) + Kd * derivative;

    *prev_error = error;

    return output;
}

// Function to simulate the dynamics of the system
void simulateSystem(double &position, double &velocity, double force) {
    // Simulate the dynamics (simple integration for this example)
    velocity += force * dt;
    position += velocity * dt;
}

int main() {
    // Initial conditions
    double ball_position = 0.5;
    double ball_velocity = 0.5;

    // Setpoint (desired position)
    double setpoint = 1.0;

    // Variables for PID control
    double prev_error = 0.0;
    double integral = 0.0;

    // Simulation loop
    for (int i = 0; i < 2000; ++i) {
        // Compute PID control output
        double control_output = computePID(setpoint, ball_position, &prev_error, &integral);

        // Apply control to the system and simulate dynamics
        simulateSystem(ball_position, ball_velocity, control_output);

        // Print current position and control output
        printf("Time: %.2f, Position: %.2f, Control Output: %.2f\n", i * dt, ball_position, control_output);
        usleep(1000);
    }

    return 0;
}