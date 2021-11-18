//Libraries
#include <PID_v1.h>

//Sensor pins names
#define ADC_0 A0
#define ADC_1 A1
#define ADC_2 A2
#define ADC_3 A3
#define ADC_4 A4
#define ADC_5 A5
#define ADC_6 A6
#define ADC_7 A7

//Motor pins names
#define PWMA 6 //A - right
#define AIN1 3
#define AIN2 2
#define PWMB 5 //B - left
#define BIN1 7
#define BIN2 8
#define MOTOR1_R 9
#define MOTOR2_R 10
#define MOTOR1_L 11
#define MOTOR2_L 12
#define STBY 4

//Global values
unsigned int ADC_x[8] = {ADC_0, ADC_1, ADC_2, ADC_3, ADC_4, ADC_5, ADC_6, ADC_7}; //table with pin numbers of each sensor
int weight[8] = {-450, -300, -180, -64, 64, 180, 300, 450};                       //table of weights - each sensor is assigned a weight which is the measure of its distance from the centre of the skid (in millimetres) multiplied by 10
unsigned int treshold = 250;                                                      //border between ground and line
unsigned int max_RPM_left_motor = 33333;                                          //max RMP where PWM = 255
unsigned int max_RPM_right_motor = 33333;                                         //TODO: Zmierzyć
unsigned int left_motor_RPM, right_motor_RPM;                                     //variables for current motors RPM
unsigned int left_motor_counter = 0;
unsigned int right_motor_counter = 0;
unsigned int current_time = 0;
unsigned int previous_time = 0;
int current_left_encoder_state = 0;
int previous_left_encoder_state = 0;
int current_right_encoder_state = 0;
int previous_right_encoder_state = 0;
int normal_speed = 100;
int state = 0;
int last_error = 0;
int output_sensors = 0;

//PID global values
double setpoint_left_motor, input_left_motor, output_left_motor;
double setpoint_right_motor, input_right_motor, output_right_motor;

//Specify the links and initial tuning parameters
double Kp = 0.044, Ki = 0.87, Kd = 0.005;

PID myPID_left_motor(&input_left_motor, &output_left_motor, &setpoint_left_motor, Kp, Ki, Kd, DIRECT);
PID myPID_right_motor(&input_right_motor, &output_right_motor, &setpoint_right_motor, Kp, Ki, Kd, DIRECT);

int *SensorsRead() //Function reads values from reflective sensors and returns them as an array
{
    static int arr[8];

    arr[0] = analogRead(ADC_0);
    arr[1] = analogRead(ADC_1);
    arr[2] = analogRead(ADC_2);
    arr[3] = analogRead(ADC_3);
    arr[4] = analogRead(ADC_4);
    arr[5] = analogRead(ADC_5);
    arr[6] = analogRead(ADC_6);
    arr[7] = analogRead(ADC_7);

    return arr;
}

void printSendorData() //Function reads values using SensorsRead() and writes them out at 200ms intervals
{
    int *sensor_data = SensorsRead();

    for (int i = 0; i < 8; i++)
    {
        Serial.print(*(sensor_data + i));

        if (i != 7)
        {
            Serial.print(", ");
        }
    }
    Serial.println("");

    delay(200);
}

void countLeftEncoder() //Function counts left encoder state changes (only on one channel(MOTOR1_L) - 6 counts per revolution)
{
    current_left_encoder_state = digitalRead(MOTOR1_L);
    if (current_left_encoder_state != previous_left_encoder_state)
    {
        left_motor_counter++;
        // if (digitalRead(MOTOR2_L) != current_left_encoder_state){
        //     left_motor_counter++;
        // }
    }
    // else if (current_left_encoder_state == previous_left_encoder_state){
    //     if (digitalRead(MOTOR2_L) != current_left_encoder_state){
    //     right_motor_counter++;
    //     }
    // }
    previous_left_encoder_state = current_left_encoder_state; // Updates the previous state of the MOTOR1_L with the current state
}

void countRightEncoder() //Function counts right encoder state changes (only on one channel(MOTOR1_R) - 6 counts per revolution)
{
    current_right_encoder_state = digitalRead(MOTOR1_R);
    if (current_right_encoder_state != previous_right_encoder_state)
    {
        right_motor_counter++;
        // if (digitalRead(MOTOR2_R) == current_right_encoder_state){
        //     right_motor_counter++;
        // }
    }
    // else if (current_right_encoder_state == previous_right_encoder_state){
    //     if (digitalRead(MOTOR2_R) != current_right_encoder_state){
    //         right_motor_counter++;
    //     }
    // }
    previous_right_encoder_state = current_right_encoder_state; // Updates the previous state of the MOTOR1_L with the current state
}

void currentLeftMotorRPM() //Function measures and calculates the current left motor RPM
{                          //TODO: Postarać się skrócić czas pomiarów
    left_motor_counter = 0;
    previous_time = millis();

    while ((millis() - previous_time) < 100)
    {
        countLeftEncoder();
    }

    left_motor_RPM = left_motor_counter * 100; // 6 counts per roration; (counter/6) in 100ms gives ((counter/6)*(60 * 1000ms))/100 ms = (100*counter) rotation per minute(RPM)
    // left_motor_RPM = left_motor_counter * 200; // 12 counts per roration; (counter/12) in t ms gives ((counter/6)*(60 * 1000ms))/t ms = (5000*counter/t) rotation per minute(RPM)
}

void currentMotorRPM() //Function measures and calculates the current left motor RPM
{                      //TODO: Postarać się skrócić czas pomiarów
    left_motor_counter = 0;
    previous_time = millis();

    while ((millis() - previous_time) < 100)
    {
        countLeftEncoder();
        countLeftEncoder();
    }

    left_motor_RPM = left_motor_counter * 100; // 6 counts per roration; (counter/6) in 100ms gives ((counter/6)*(60 * 1000ms))/100 ms = (100*counter) rotation per minute(RPM)
    // left_motor_RPM = left_motor_counter * 200; // 12 counts per roration; (counter/12) in t ms gives ((counter/6)*(60 * 1000ms))/t ms = (5000*counter/t) rotation per minute(RPM)
}

void currentRightMotorRPM() //Function measures and calculates the current right motor RPM
{
    right_motor_counter = 0;
    previous_time = millis();

    while ((millis() - previous_time) < 100)
    {
        countRightEncoder();
    }

    right_motor_RPM = right_motor_counter * 100; // 6 counts per roration; (counter/6) in 100ms gives ((counter/6)*(60 * 1000ms))/100 ms = (100*counter) rotation per minute(RPM)
    // right_motor_RPM = right_motor_counter * 200; // 12 counts per roration; (counter/12) in t ms gives ((counter/6)*(60 * 1000ms))/t ms = (5000*counter/t) rotation per minute(RPM)
}

int sensors_error() //Function calculates the value of sensores regulation error
{
    int error = 0;
    int counter = 0;

    for (int i = 0; i < 8; i++)
    {
        if (analogRead(ADC_x[i]) > treshold)
        {
            error = error + weight[i];
            counter = counter + 1;
        };
    };

    if (counter == 0)
    {
        error = last_error;
    }
    else
    {
        error = error / counter;
        last_error = error;
    }

    return error;
}

int left_motor_PWM() //Function calculates the value of left motor PWM
{
    currentLeftMotorRPM();
    int error = map(left_motor_RPM, 0, max_RPM_left_motor, 0, 255);

    return error;
}

int right_motor_PWM() //Function calculates the value of right motor PWM
{
    currentRightMotorRPM();
    int error = map(right_motor_RPM, 0, max_RPM_right_motor, 0, 255);

    return error;
}

void drive(int PWMB_value, int PWMA_value) //Function for setting the specified filling of the PWM signal for motors (left, right)
{
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);

    analogWrite(PWMA, PWMA_value);
    analogWrite(PWMB, PWMB_value);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
    Serial.begin(9600);

    //Sensor pins configuration
    pinMode(ADC_0, INPUT);
    pinMode(ADC_1, INPUT);
    pinMode(ADC_2, INPUT);
    pinMode(ADC_3, INPUT);
    pinMode(ADC_4, INPUT);
    pinMode(ADC_5, INPUT);
    pinMode(ADC_6, INPUT);
    pinMode(ADC_7, INPUT);

    //Motor pins configuration
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(MOTOR1_R, INPUT);
    pinMode(MOTOR2_R, INPUT);
    pinMode(MOTOR1_L, INPUT);
    pinMode(MOTOR2_L, INPUT);
    pinMode(STBY, OUTPUT);

    digitalWrite(STBY, HIGH);
    previous_right_encoder_state = digitalRead(MOTOR1_R);
    previous_left_encoder_state = digitalRead(MOTOR1_L);

    //Left Motor PID
    input_left_motor = 0;
    setpoint_left_motor = 0;

    myPID_left_motor.SetMode(AUTOMATIC);
    myPID_left_motor.SetOutputLimits(0, 255);

    //Right Motor PID
    input_right_motor = 0;
    setpoint_right_motor = 0;

    myPID_right_motor.SetMode(AUTOMATIC);
    myPID_right_motor.SetOutputLimits(0, 255);
}

void loop()
{
    if (state == 1)
    {
        output_sensors = sensors_error();
        int output_sensors_ = map(abs(output_sensors), 0, 450, 0, 50);

        if (output_sensors > 0)
        {
            setpoint_left_motor = normal_speed - output_sensors_;
            setpoint_right_motor = normal_speed + output_sensors_;
        }
        else
        {
            setpoint_left_motor = normal_speed + output_sensors_;
            setpoint_right_motor = normal_speed - output_sensors_;
        }

        input_left_motor = left_motor_PWM();
        input_right_motor = right_motor_PWM();

        //test

        Serial.print("* ");
        Serial.print(output_sensors);
        Serial.print(" ");
        Serial.print(setpoint_left_motor);
        Serial.print(" ");
        Serial.print(setpoint_right_motor);
        Serial.print(" ");
        Serial.print(input_left_motor);
        Serial.print(" ");
        Serial.print(input_right_motor);
        Serial.println(" ");
        myPID_left_motor.Compute();
        myPID_right_motor.Compute();

        drive(output_left_motor, output_right_motor);
    }
    else
    {
        drive(0, 0);
    }
}

void serialEvent()
{
    while (Serial.available())
    {
        state = Serial.readStringUntil(';').toInt();

        if (state != 0)
        {
            double Kp_ = Serial.readStringUntil(';').toDouble();
            Kp = (Kp_ == 0 ? Kp : Kp_);
            double Ki_ = Serial.readStringUntil(';').toDouble();
            Ki = (Ki_ == 0 ? Ki : Ki_);
            double Kd_ = Serial.readStringUntil(';').toDouble();
            Kd = (Kd_ == 0 ? Kd : Kd_);
            int normal_speed_ = Serial.readStringUntil(';').toInt();
            normal_speed = (normal_speed_ == 0 ? normal_speed : normal_speed_);
            int treshold_ = Serial.readStringUntil(';').toInt();
            treshold = (treshold_ == 0 ? treshold : treshold_);

            myPID_left_motor.SetMode(AUTOMATIC);
            myPID_right_motor.SetMode(AUTOMATIC);
        }
    }
}