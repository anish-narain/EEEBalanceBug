
#include <stdio.h>

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
    // Sample acceleration data
    float acceleration_data[] = {1.2, 0.5, -0.8};
    // Sample time interval between measurements
    float time_interval = 0.1;
    // Number of samples
    int num_samples = sizeof(acceleration_data) / sizeof(acceleration_data[0]);

    // Array to store displacement for each axis
    float displacement[num_samples];

    // Calculate displacement for each axis
    calculate_displacement(acceleration_data, displacement, time_interval, num_samples);

    // Print the resulting displacements for each axis
    printf("Displacement (X-axis): %.2f\n", displacement[0]);
    printf("Displacement (Y-axis): %.2f\n", displacement[1]);
    printf("Displacement (Z-axis): %.2f\n", displacement[2]);

    return 0;
}