#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>  // For handling SIGINT (Ctrl+C)
#include "livox_sdk.h"

// Substantial object criteria
#define OBSTACLE_THRESHOLD 1.0  // meters (anything closer than 1 meter is considered an obstacle)

// Flag to indicate when to stop the program
volatile sig_atomic_t stop_program = 0;

// Function to stop the robot
void StopRobot() {
    printf("Obstacle detected! Stopping the robot...\n");
    // Add code here to stop the robot's motors, for example:
    // motor_control.stop();
}

// Signal handler for SIGINT (Ctrl+C)
void sigint_handler(int signum) {
    stop_program = 1;
}

/** Receiving point cloud data from Livox LiDAR. */
void GetLidarData(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data) {
    if (data) {
        for (uint32_t i = 0; i < data_num; ++i) {
            LivoxRawPoint *point_data = (LivoxRawPoint *)data->data;

            // Apply obstacle detection criteria (distance threshold)
            float distance = sqrt(point_data->x * point_data->x + point_data->y * point_data->y + point_data->z * point_data->z);
            
            // Check if the point is within the threshold distance and is in front of the robot (positive x direction)
            if (distance < OBSTACLE_THRESHOLD && point_data->x > 0) {
                // This point is an obstacle in front of the robot, stop the robot
                StopRobot();
                return;  // Stop checking further points, since the obstacle is already detected
            }
        }
    }
}

// Directly connect to the first LiDAR device
void OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
    if (info == NULL || info->dev_type == kDeviceTypeHub) {
        return;
    }

    printf("Received Broadcast Code: %s\n", info->broadcast_code);

    uint8_t handle = 0;
    bool result = AddLidarToConnect(info->broadcast_code, &handle);
    if (result == kStatusSuccess) {
        // Set data callback to process point cloud data
        SetDataCallback(handle, GetLidarData, NULL);  
        printf("LiDAR connected and data callback set.\n");
    } else {
        printf("Failed to connect to LiDAR device with broadcast code %s.\n", info->broadcast_code);
    }
}

int main(int argc, const char *argv[]) {
    printf("Livox SDK initializing.\n");

    if (!Init()) {
        return -1;
    }
    printf("Livox SDK initialized.\n");

    LivoxSdkVersion _sdkversion;
    GetLivoxSdkVersion(&_sdkversion);
    printf("Livox SDK version %d.%d.%d .\n", _sdkversion.major, _sdkversion.minor, _sdkversion.patch);

    // Set the callback for receiving device broadcast information
    SetBroadcastCallback(OnDeviceBroadcast);

    // Set signal handler for SIGINT (Ctrl+C) to cleanly stop the program
    signal(SIGINT, sigint_handler);

    // Start discovering devices
    if (!Start()) {
        Uninit();
        return -1;
    }
    printf("Start discovering devices.\n");

    // Continuous loop to keep the program running
    while (!stop_program) {
        // You can add other tasks to run here, if needed (e.g., checking status or handling other events)
        usleep(100000);  // Sleep for a short period to prevent high CPU usage
    }

    printf("Program interrupted by user. Stopping...\n");

    // Uninitialize the SDK after interruption
    Uninit();
    return 0;
}
