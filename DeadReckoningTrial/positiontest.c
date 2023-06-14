// for testing with inputs of 10 samples in the x-axis

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

void generate_acceleration_data(float acceleration_data[], int num_samples, float acceleration) {
    for (int i = 0; i < num_samples; i++) {
        acceleration_data[i] = acceleration;
    }
}

void calculate_displacement(float acceleration_data[], float displacement[], float time_interval, int num_samples) {
    float velocity = 0.0;
    float prev_velocity = 0.0;
    float prev_displacement = 0.0;

    for (int i = 0; i < num_samples; i++) {
        velocity += acceleration_data[i] * time_interval;
        displacement[i] = prev_displacement + 0.5 * (velocity + prev_velocity) * time_interval;
        prev_velocity = velocity;
        prev_displacement = displacement[i];
    }
}

int main() {
    // Number of samples
    int num_samples = 10;
    // Sample time interval between measurements
    float time_interval = 0.1;

    // Acceleration data for traveling in a straight line in the x-axis
    float acceleration_data[num_samples];
    // Set constant acceleration along the x-axis
    float acceleration = 1.0; // Change the value as desired

    // Array to store displacement for each axis
    float displacement[num_samples];

    // Generate acceleration data for straight-line motion in the x-axis
    generate_acceleration_data(acceleration_data, num_samples, acceleration);

    // Calculate displacement for each axis
    calculate_displacement(acceleration_data, displacement, time_interval, num_samples);

    // Print the resulting displacements for each axis
    for (int i = 0; i < num_samples; i++) {
        printf("Displacement (X-axis) at t=%.1f: %.2f\n", time_interval * i, displacement[i]);
    }

    return 0;
}