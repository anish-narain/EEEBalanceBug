#include <stdio.h>
#include <stdlib.h>
#include <time.h>

void generate_random_acceleration(float acceleration_data[], int num_samples) {
    for (int i = 0; i < num_samples; i++) {
        acceleration_data[i] = (float)rand() / RAND_MAX * 2.0 - 1.0; // Random value between -1 and 1
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
    int num_samples = 3;
    // Sample time interval between measurements
    float time_interval = 0.1;

    // Array to store acceleration data and displacement for each axis
    float acceleration_data[num_samples];
    float displacement[num_samples];

    // Seed the random number generator
    srand(time(NULL));

    // Continuously generate accelerometer data and calculate displacement
    while (1) {
        // Generate random accelerometer data
        generate_random_acceleration(acceleration_data, num_samples);

        // Calculate displacement for each axis
        calculate_displacement(acceleration_data, displacement, time_interval, num_samples);

        // Print the resulting displacements for each axis
        printf("Displacement (X-axis): %.2f\n", displacement[0]);
        printf("Displacement (Y-axis): %.2f\n", displacement[1]);
        printf("Displacement (Z-axis): %.2f\n", displacement[2]);

        // Add a delay (optional)
        Sleep(3000); // Delay for 1 second
    }

    return 0;
}
