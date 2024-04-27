#include <mbed.h>
#include "math.h"
#include "drivers/LCD_DISCO_F429ZI.h"
#include "iostream"
#include "stdio.h"
#include <cstdio>
#define WINDOW_SIZE 15

// Defining Gyroscope's settings
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4 0x23 // Second configure to set the DPS
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0

#define CTRL_REG3 0x22 
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000

#define OUT_X_L 0x28

#define SPI_FLAG 1
#define DATA_READY_FLAG 2
#define BUTTON_PRESSED_FLAG 4

#define SCALING_FACTOR (17.5f * 0.01745f / 1000.0f)

#define FILTER_COEFFICIENT 0.1f // Adjust this value as needed

// EventFlags object declaration
EventFlags flags;

// DigitalIn button(PA_0);
InterruptIn button(BUTTON1);

DigitalOut led(LED1);
float conv (uint16_t R, uint16_t A)
{
    float V;
    V = R*A;
    return V;
}

float dist (float V, float d)
{
    
    d = d +  float(V* 0.5);
    return d;
    
}
//Serial Port
BufferedSerial serial_port(USBTX, USBRX, 9600);

//Override writing to console to writer to serial port to handle writing to file
FileHandle *mbed::mbed_override_console(int fd)
{
    return &serial_port;
}


// spi callback function
void spi_cb(int event) {
    flags.set(SPI_FLAG);
}

// data ready callback function
void data_cb() {
    flags.set(DATA_READY_FLAG);
}

// button pushed callback function
void button_pushed_cb(){
    led = !led;
    flags.set(BUTTON_PRESSED_FLAG);
}


Ticker timer;
int countSecs = 0;
void timer_cb(){
    countSecs++;
}


int main() {

    LCD_DISCO_F429ZI LCD;

    //spi initialization
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
    uint8_t write_buf[32], read_buf[32];

    //interrupt initialization
    InterruptIn int2(PA_2, PullDown);
    int2.rise(&data_cb);

    //button setup
    button.rise(&button_pushed_cb);
    
    //spi format and frequency
    spi.format(8, 3);
    spi.frequency(1'000'000);

    // Write to control registers --> spi transfer
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG3;
    write_buf[1] = CTRL_REG3_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[1] = 0xFF;

    //(polling for\setting) data ready flag
    if (!(flags.get() & DATA_READY_FLAG) && (int2.read() == 1)) {
        flags.set(DATA_READY_FLAG);
    }

    // Example 2: LPF definitions
    float filtered_gx = 0.0f, filtered_gy = 0.0f, filtered_gz = 0.0f;

    // Example 3: HPF definitions
    // use with the example 2 definitions
    float high_pass_gx = 0.0f, high_pass_gy = 0.0f, high_pass_gz = 0.0f;

    led.write(0);
    

   float totalDistance = 0;
   char totalDis[12]; // Buffer to hold the string representation of total distance

    while (1) {
        //-----Conversion----
        uint16_t radius = 96; //needs to be measured based on the tester | unit = cms
        float Velocity[40];
        float angular_velocity_x[40],angular_velocity_y[40],angular_velocity_z[40];

        float Distance = 0.0; //cms
        char dis[12]; // Buffer to hold the string representation of dist
        float total_velocity = 0.0;
        float total_velocity_x = 0.0;
        float total_velocity_y = 0.0;
        float total_velocity_z = 0.0;

        float avg_velocity = 0.0;
        float avg_angular_velocity_x = 0.0;
        float avg_angular_velocity_y = 0.0;
        float avg_angular_velocity_z = 0.0;

        char avg_vel[12],avg_vel_x[12],avg_vel_y[12],avg_vel_z[12]; // Buffer to hold the string representation of average velocity
        char time[12]; // Buffer to hold the string representation of time elapsed
       
        int16_t raw_gx, raw_gy, raw_gz;
        float gx_final[40], gy_final[40], gz_final[40];
        float gx, gy, gz;
        int i = 0;


        //Push a button to initiate data recording for 20 seconds
        flags.wait_all(BUTTON_PRESSED_FLAG);
        timer.attach(&timer_cb, 1000ms);
        countSecs = 0;
        i = 0;
        while(countSecs < 20){

            flags.wait_all(DATA_READY_FLAG);
            write_buf[0] = OUT_X_L | 0x80 | 0x40;

            spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
            flags.wait_all(SPI_FLAG);

            // Process raw data
            raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
            raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
            raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

            gx = ((float)raw_gx) * SCALING_FACTOR;
            gy = ((float)raw_gy) * SCALING_FACTOR;
            gz = ((float)raw_gz) * SCALING_FACTOR;
            char buffer[50]; // Buffer to hold the formatted string

            // Assuming gx is a float and you want to keep it as a float
            sprintf(buffer, "%f rad/s", gx); // Format gx as a float and append " rad/s"

            // Display on LCD
            LCD.DisplayStringAt(0, LINE(6), (uint8_t *)buffer, CENTER_MODE);

            std::string a1 = std::string(buffer) + " m"; // Concatenate with units
            LCD.DisplayStringAt(0, LINE(6), (uint8_t *)a1.c_str(),CENTER_MODE);


            // Simple low-pass filter
            filtered_gx = FILTER_COEFFICIENT * gx + (1 - FILTER_COEFFICIENT) * filtered_gx;
            filtered_gy = FILTER_COEFFICIENT * gy + (1 - FILTER_COEFFICIENT) * filtered_gy;
            filtered_gz = FILTER_COEFFICIENT * gz + (1 - FILTER_COEFFICIENT) * filtered_gz;

            // high-pass filter with the lpf (by eliminating low freq elements)
            high_pass_gx = gx - filtered_gx;
            high_pass_gy = gy - filtered_gy;
            high_pass_gz = gz - filtered_gz;

            // record data samples in array
            gx_final[i] = high_pass_gx;
            gy_final[i] = high_pass_gy;
            gz_final[i] = high_pass_gz;
            
      

            //Conversion
            //Z axis is used as device will be attached to side
            Velocity[i] = conv(radius, gz_final[i]);
            Distance = dist(Velocity[i], Distance);
            printf("%d\n" , int(Distance));

            angular_velocity_x[i] = gx_final[i];  // X-axis angular velocity
            angular_velocity_y[i] = gy_final[i];  // Y-axis angular velocity
            angular_velocity_z[i] = gz_final[i];  // Z-axis angular velocity
            
            total_velocity_x+= abs(angular_velocity_x[i]);
            total_velocity_y+= abs(angular_velocity_y[i]);
            total_velocity_z+= abs(angular_velocity_z[i]);

            avg_angular_velocity_x = total_velocity_x/(i+1); //i+1 so that we dont divide by 0 or miss out on the first velocity values
            avg_angular_velocity_y = total_velocity_y/(i+1); //i+1 so that we dont divide by 0 or miss out on the first velocity values
            avg_angular_velocity_z = total_velocity_z/(i+1); //i+1 so that we dont divide by 0 or miss out on the first velocity values
            total_velocity += abs(Velocity[i]);

            avg_velocity = total_velocity/(i+1); //i+1 so that we dont divide by 0 or miss out on the first velocity values


            sprintf(totalDis, "%d", int(totalDistance)); // Convert float to string
            std::string finaldistance = std::string(totalDis) + " m"; // Concatenate with units
            sprintf(dis, "%d", int(Distance)/100); // Convert float to string
            std::string finalmeasurement = std::string(dis) + " m"; // Concatenate with units
            sprintf(avg_vel, "%d", int(avg_angular_velocity_x)); // Convert float to string
            std::string ang_velocity_x = std::string(avg_vel_x) + " rad/s"; // Concatenate with units
            sprintf(avg_vel, "%d", int(avg_velocity)); // Convert float to string
            std::string average_velocity = std::string(avg_vel) + " cm/s"; // Concatenate with units
            sprintf(avg_vel, "%d", int(avg_angular_velocity_y)); // Convert float to string
            std::string ang_velocity_y = std::string(avg_vel_y) + " rad/s"; // Concatenate with units
            sprintf(avg_vel, "%d", int(avg_angular_velocity_z)); // Convert float to string
            std::string ang_velocity_z = std::string(avg_vel_z) + " rad/s"; // Concatenate with units
            sprintf(time, "%d", countSecs); // Convert float to string
            std::string time_taken = std::string(time) + " seconds"; // Concatenate with units

            LCD.Clear(LCD_COLOR_ORANGE);
            LCD.SetBackColor(LCD_COLOR_ORANGE);
            LCD.SetTextColor(LCD_COLOR_BLACK);
            LCD.SetFont(&Font16);

            LCD.DisplayStringAt(0, LINE(2), (uint8_t *)"Parkinsons detector!",CENTER_MODE);

            LCD.DisplayStringAt(0, LINE(5), (uint8_t *)"Distance Travelled:",CENTER_MODE);

            LCD.DisplayStringAt(0, LINE(6), (uint8_t *)finaldistance.c_str(),CENTER_MODE);

            LCD.DisplayStringAt(0, LINE(7), (uint8_t *)"Angular Velocity:",CENTER_MODE);
            LCD.DisplayStringAt(0, LINE(8), (uint8_t *)ang_velocity_x.c_str(),CENTER_MODE);

            LCD.DisplayStringAt(0, LINE(10), (uint8_t *)"Time Elapsed:",CENTER_MODE);
            LCD.DisplayStringAt(0, LINE(11), (uint8_t *)time_taken.c_str(),CENTER_MODE);
        
            
            i++;

            //sleep for 500ms = 0.5 second readings
            thread_sleep_for(495);
            
        }
        
        //detach timer to stop it from running in background and using resources
        timer.detach();

        
        //update screen once more after end of while loop as loop will exit after countSecs increments from 19 to 20 and this will not get updated again
        totalDistance += Distance;

        avg_velocity = total_velocity/(i+1); //i+1 so that we dont divide by 0 or miss out on the first velocity values

        sprintf(totalDis, "%d", int(totalDistance)); // Convert float to string
        std::string finaldistance = std::string(totalDis) + " m"; // Concatenate with units
        sprintf(dis, "%d", int(Distance)/100); // Convert float to string
        std::string finalmeasurement = std::string(dis) + " m"; // Concatenate with units
        sprintf(avg_vel, "%d", int(avg_velocity)); // Convert float to string
        std::string average_velocity = std::string(avg_vel) + " cm/s"; // Concatenate with units
        sprintf(avg_vel, "%f", int(avg_angular_velocity_x)); // Convert float to string
        std::string ang_velocity_x = std::string(avg_vel_x) + " rad/s"; // Concatenate with units
        sprintf(avg_vel, "%f", int(avg_angular_velocity_y)); // Convert float to string
        std::string ang_velocity_y = std::string(avg_vel_y) + " rad/s"; // Concatenate with units
        sprintf(avg_vel, "%f", int(avg_angular_velocity_z)); // Convert float to string
        std::string ang_velocity_z = std::string(avg_vel_z) + " rad/s"; // Concatenate with units
        sprintf(time, "%d", countSecs); // Convert float to string
        std::string time_taken = std::string(time) + " seconds"; // Concatenate with units

        LCD.Clear(LCD_COLOR_ORANGE);
        LCD.SetBackColor(LCD_COLOR_ORANGE);
        LCD.SetTextColor(LCD_COLOR_BLACK);
        LCD.SetFont(&Font16);


        LCD.DisplayStringAt(0, LINE(5), (uint8_t *)"Total Distance:",CENTER_MODE);
        LCD.DisplayStringAt(0, LINE(6), (uint8_t *)finaldistance.c_str(),CENTER_MODE);

        LCD.DisplayStringAt(0, LINE(12), (uint8_t *)"Time Elapsed:",CENTER_MODE);
        LCD.DisplayStringAt(0, LINE(13), (uint8_t *)time_taken.c_str(),CENTER_MODE);
        
        LCD.DisplayStringAt(0, LINE(10), (uint8_t *)"Angular Velocity:",CENTER_MODE);
        LCD.DisplayStringAt(0, LINE(11), (uint8_t *)ang_velocity_x.c_str(),CENTER_MODE);


        LCD.DisplayStringAt(0, LINE(1), (uint8_t *)"PARKINSONS DETECTOR",CENTER_MODE);

        LCD.DisplayStringAt(0, LINE(2), (uint8_t *)"Project by:",CENTER_MODE);
        LCD.DisplayStringAt(0, LINE(3), (uint8_t *)"A,A,R",CENTER_MODE);



        led.write(0);
        
       
    }
}
