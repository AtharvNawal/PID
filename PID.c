#include <stdio.h>
#include <unistd.h>

#define Kp 20
#define Ki 0.2
#define Kd 0.2

#define dt 0.1  // Time step

    double setpoint  = 1.0;

// Function
double computePID(double setpoint, double current_position, double *prev_error, double *integral) {
    double error = setpoint - current_position;
    double derivative = (error - *prev_error) / dt;

    *integral += error * dt;

    // PID  eq
    double output = Kp * error + Ki * (*integral) + Kd * derivative;

    *prev_error = error;

    return output;
}

//  system dynamics
void simulateSystem(double &position, double &velocity, double force) {
    velocity += force * dt;
    position += velocity * dt;
}

// graph
void displayGraph(double position) {
   printf("\n");
    if(position<setpoint){
    printf("\t\t\t\t|");
    int graphHeight = 20;
    int scaledPosition = (int)((position + 1.0) * 0.5 * graphHeight);

    for (int i = graphHeight; i >= 0; --i) {
        if ((i-1) == scaledPosition)
            printf("#");
        else
            printf(" ");
    }

    printf("\n");
}
if(position>setpoint){
int i;
  int graphHeight = 20;
    int scaledPosition = (int)(((position)) * 0.5 * graphHeight);
    for (int j=0;j<(41-scaledPosition);j++){
    printf(" ");
    }
    printf("#");
    /*for ( i= graphHeight; i > 2; --i) { //wrong code

        if (i == scaledPosition){
            printf("*");
            }
        else
            printf(" ");

    }*/
    for(int j=10;j<(scaledPosition);j++){
    printf(" ");}
    printf("|");

    printf("\n");

  }

}

int main() {
    double ball_position = 3.0;
    double ball_velocity = 0.5;



    double prev_error = 0.0;
    double integral = 0.0;

    // Simulation loop
    for (int i = 0; i < 1000; ++i) {
        double control_output = computePID(setpoint, ball_position, &prev_error, &integral);

        simulateSystem(ball_position, ball_velocity, control_output);

        // Display graph
        displayGraph(ball_position);


       // printf("Time: %.2f, Position: %.2f, Control Output: %.2f\n", i * dt, ball_position, control_output);
        usleep(10000);
    }

    return 0;
}
