#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>

// gcc -o motor_test motor_test_direct.cpp -lwiringPi -lm
// gcc -o motor_test "motor_test_direct.cpp" -lwiringPi -lm
// sudo ./motor_test


int motor_address;
int run_program = 1;

void send_motor(int motor_id, int speed)
{
    if (speed < 0)   speed = 0;
    if (speed > 1000) speed = 1000;
    uint8_t special_command = 0;
    uint8_t data[2];
    data[0] = 0x80 + (motor_id << 5) + (special_command << 4) + ((speed >> 7) & 0x0f);
    data[1] = speed & 0x7f;
    wiringPiI2CWrite(motor_address, data[0]);
    usleep(500);
    wiringPiI2CWrite(motor_address, data[1]);
    usleep(500);
}

void set_all(int speed)
{
    for (int m = 0; m < 4; m++)
        send_motor(m, speed);
}

void motor_enable()
{
    printf("Running ESC calibration sequence...\n");
    // send 0 for 1000 cycles
    for (int i = 0; i < 1000; i++) {
        for (int m = 0; m < 4; m++)
            send_motor(m, 0);
    }
    // send 50 for 2000 cycles (arm)
    for (int i = 0; i < 2000; i++) {
        for (int m = 0; m < 4; m++)
            send_motor(m, 50);
    }
    // back to 0
    for (int i = 0; i < 500; i++) {
        for (int m = 0; m < 4; m++)
            send_motor(m, 0);
    }
    printf("ESC calibration done.\n");
}

void trap(int signal)
{
    printf("\nKilling all motors and exiting.\n");
    set_all(0);
    run_program = 0;
}

int main()
{
    wiringPiSetup();
    motor_address = wiringPiI2CSetup(0x56);
    if (motor_address == -1) {
        printf("ERROR: can't connect to motor I2C at 0x56\n");
        return -1;
    }
    printf("Motor controller connected.\n");

    signal(SIGINT, &trap);

    motor_enable();

    // Test each motor individually at speed 300
    int test_speed = 300;
    int hold_ms = 2000; // how long to spin each motor (ms)

    for (int m = 0; m < 4 && run_program; m++) {
        printf("Spinning motor %d at speed %d for %d ms\n",
               m, test_speed, hold_ms);
        for (int t = 0; t < hold_ms && run_program; t++) {
            send_motor(m, test_speed);
            usleep(1000);
        }
        printf("Motor %d off.\n", m);
        // stop that motor before next
        for (int i = 0; i < 100; i++)
            send_motor(m, 0);
        sleep(1);
    }

    printf("All motors tested. Sending 0 to all.\n");
    set_all(0);
    return 0;
}
