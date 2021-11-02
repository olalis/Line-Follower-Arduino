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

void encodersState() //Function returns the current value read from the encoders
{
    ;
}

int error(int treshold) //Function calculates the value of regulation error
{
    int error = 0;
    int ADC_x[8] = {ADC_0, ADC_1, ADC_2, ADC_3, ADC_4, ADC_5, ADC_6, ADC_7};
    int weight[8] = {-300, -200, -100, -1, 1, 100, 200, 300};

    for (int i = 0; i < 8; i++)
    {
        if (analogRead(ADC_x[i]) < treshold)
        {
            error = error + weight[i];
        };
    };

    return error;
}

void drive(int PWMA_value, int PWMB_value) //Function for setting the specified filling of the PWM signal for motors
{
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);

    analogWrite(PWMA, PWMA_value);
    analogWrite(PWMB, PWMB_value);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
}

void loop()
{
    ;
}
