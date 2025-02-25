
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <math.h>

#define PIN_FLOW_SENSOR         PA0
#define PIN_GPS_TX              PA2
#define PIN_GPS_RX              PA3
#define PIN_GSM_PWRK            PA8
#define PIN_GSM_TX              PA9
#define PIN_GSM_RX              PA10

#define serial_monitor          Serial
#define serial_gsm              Serial1

#define earth_radius_km         6371.0  

#define PIN_ACC_SDA PB7
#define PIN_ACC_SCL PB6
#define ADDR_ADXL345 0x53

float acc_x_offset = 0;
float acc_velocity_x = 0;
float acc_distance_x = 0;
float acc_distance_x_km = 0;
unsigned long prevrime_1 = 0;
const float accel_threshold_1 = 0.005;
const float velocity_damping_1 = 0.99;
const float alpha = 0.2;

TinyGPSPlus gps;
HardwareSerial serial_gps(USART2);

const String PHONE_NUMBER = USER_PHONE_NUMBER;

double print_latitude = 0.0, print_longitude = 0.0;

double current_speed = 0.0;
int date_day, date_month, date_year;
int time_hour, time_minute, time_second;

int CALIBRATED_HOUR = MENTIONED_CALIBRATED_HOUR;
int CALIBRATED_MINUTE = MENTIONED_CALIBRATED_MINUTE;
int CALIBRATED_SECOND = MENTIONED_CALIBRATED_SECOND;

double current_latitude = 0.0, current_longitude = 0.0;
double last_latitude = 0.0, last_longitude = 0.0;
double total_distance_traveled = 0.0;

volatile uint32_t pulse_count = 0;
uint32_t last_time = 0;
double flow_rate = 0;
double total_fuel_passed = 0;

uint32_t last_update_time = 0;  
double gps_avg_fuel_per_km = 0;
double acc_avg_fuel_per_km = 0;

void gps_data() {
    while (serial_gps.available() > 0) {
        char c = serial_gps.read();
        gps.encode(c);

        if (gps.location.isUpdated()) {
            print_latitude = gps.location.lat();
            print_longitude = gps.location.lng();
            //serial_monitor.print("lat : ");
            //serial_monitor.println(print_latitude, 6);
            //serial_monitor.print("long : ");
            //serial_monitor.println(print_longitude, 6);
        }

        if (gps.speed.isUpdated()) {
            current_speed = gps.speed.kmph();
            //serial_monitor.print("Speed : ");
            //serial_monitor.print(current_speed, 2);
            //serial_monitor.println("km/h");  
        }
        
        if (gps.date.isUpdated() && gps.time.isUpdated()) {
            date_day = gps.date.day();
            date_month = gps.date.month();
            date_year = gps.date.year();
            time_hour = gps.time.hour();
            time_minute = gps.time.minute();
            time_second = gps.time.second();

            time_second += CALIBRATED_SECOND;
            if (time_second >= 60) {
                time_second -= 60;
                time_minute += 1;
            }

            time_minute += CALIBRATED_MINUTE;
            if (time_minute >= 60) {
                time_minute -= 60;
                time_hour += 1;
            }

            time_hour += CALIBRATED_HOUR;
                if (time_hour >= 24) {
                time_hour -= 24;
            }

            //serial_monitor.print("Date: ");
            //serial_monitor.print(date_day);
            //serial_monitor.print("/");
            //serial_monitor.print(date_month);
            //serial_monitor.print("/");
            //serial_monitor.print(date_year);
            //serial_monitor.print(" Time: ");
            //serial_monitor.print(time_hour);
            //serial_monitor.print(":");
            //serial_monitor.print(time_minute);
            //serial_monitor.print(":");
            //serial_monitor.println(time_second);
        }
    }
}

void positive_pulse_count() {
    pulse_count++;
}

void init_flow_sensor() {
    pinMode(PIN_FLOW_SENSOR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_FLOW_SENSOR), positive_pulse_count, RISING);
    last_time = millis();
}

void flow_sensor_data() {
    uint32_t current_time = millis();

    if (current_time - last_time >= 1000) { 
        noInterrupts(); 
        uint32_t pulses = pulse_count;
        pulse_count = 0;
        interrupts();

        flow_rate = (pulses / 450.0) * 60.0;

        float fuel_passed = flow_rate / 60.0;
        total_fuel_passed += fuel_passed;

        //serial_monitor.print("Flow Rate: ");
        //serial_monitor.print(flow_rate);
        //serial_monitor.print(" L/min, ");
        //serial_monitor.print("Total Fuel Passed: ");
        //serial_monitor.print(total_fuel_passed);
        //serial_monitor.println(" L");

        last_time = current_time;
    }
}

void at_command(const char* cmd) {
    serial_gsm.println(cmd);
    delay(1000);
    String response = "";
    while (serial_gsm.available()) {
        response += (char)serial_gsm.read();
    }
    serial_monitor.print("Command: ");
    serial_monitor.println(cmd);
    serial_monitor.print("Response: ");
    serial_monitor.println(response);
}

void init_gsm() {
    pinMode(PIN_GSM_PWRK, OUTPUT);
    digitalWrite(PIN_GSM_PWRK, LOW);
    delay(3000);
    digitalWrite(PIN_GSM_PWRK, HIGH);
    delay(5000);
    digitalWrite(PIN_GSM_PWRK, LOW);
    delay(5000);
    serial_monitor.println("GSM Module Powered ON.");
    serial_monitor.println("");
    at_command("AT");
    at_command("AT+CMGF=1");
    at_command("AT+CSCA=\"+917010075009\"");
    at_command("AT+CNMI=1,2,0,0,0");
}

float haversine(float lat1, float lon1, float lat2, float lon2) {
    if (lat1 == 0.0 && lon1 == 0.0) return 0.0;

    float dlat = (lat2 - lat1) * M_PI / 180.0;
    float dlon = (lon2 - lon1) * M_PI / 180.0;

    float a = sin(dlat / 2) * sin(dlat / 2) +
              cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
              sin(dlon / 2) * sin(dlon / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return earth_radius_km * c;
}

void measure_fuel_and_distance() {
    uint32_t current_time = millis();
    if (current_time - last_update_time >= 1000) {
        noInterrupts(); 
        uint32_t pulses = pulse_count;
        pulse_count = 0;
        interrupts();

        flow_rate = (pulses / 450.0) * 60.0;
        float fuel_passed = flow_rate / 60.0;
        total_fuel_passed += fuel_passed;

        if (gps.location.isUpdated()) {
            float new_latitude = gps.location.lat();
            float new_longitude = gps.location.lng();

            float distance = haversine(last_latitude, last_longitude, new_latitude, new_longitude);
            total_distance_traveled += distance;

            last_latitude = new_latitude;
            last_longitude = new_longitude;
        }

        gps_avg_fuel_per_km = (total_distance_traveled > 0) ? (total_fuel_passed / total_distance_traveled) : 0.0;

        //serial_monitor.print("Total Fuel Used: ");
        //serial_monitor.print(total_fuel_passed);
        //serial_monitor.println(" L");

        //serial_monitor.print("Total Distance Traveled: ");
        //serial_monitor.print(total_distance_traveled);
        //serial_monitor.println(" km");

        //serial_monitor.print("Average Fuel Consumption: ");
        //serial_monitor.print(gps_avg_fuel_per_km);
        //serial_monitor.println(" L/km");

        last_update_time = current_time;
    }
}

void accel_write_reg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(ADDR_ADXL345);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

void read_accel(float &ax) {
    Wire.beginTransmission(ADDR_ADXL345);
    Wire.write(0x32);
    Wire.endTransmission(false);
    Wire.requestFrom(ADDR_ADXL345, 6, true);
    
    int16_t x = Wire.read() | (Wire.read() << 8);
    Wire.read(); Wire.read();
    Wire.read(); Wire.read();
    
    ax = x * 9.81 * 0.0039;
}

void calibrate() {
    serial_monitor.println("Calibrating... Please keep the sensor still.");
    long numSamples = 1000;
    float sumX = 0;
    
    for (long i = 0; i < numSamples; i++) {
        float ax;
        read_accel(ax);
        sumX += ax;
        delay(1);
    }
    
    acc_x_offset = sumX / numSamples;
    serial_monitor.println("Calibration complete.");
}

void init_accel() {
    Wire.setSDA(PIN_ACC_SDA);
    Wire.setSCL(PIN_ACC_SCL);
    Wire.begin();
    accel_write_reg(0x2D, 0x08);
    accel_write_reg(0x31, 0x0B);
    accel_write_reg(0x2C, 0x0D);
    accel_write_reg(0x38, 0x00);
    serial_monitor.println("ADXL345 Configured and Initialized");
    calibrate();
}

void accel_data() {
    unsigned long currentTime = millis();
    if (currentTime - prevrime_1 < 10) return;
    float dt = (currentTime - prevrime_1) / 1000.0;
    prevrime_1 = currentTime;
    
    float accelX;
    read_accel(accelX);
    accelX -= acc_x_offset;
    
    static float filteredAccelX = 0;
    filteredAccelX = alpha * accelX + (1 - alpha) * filteredAccelX;
    
    if (fabs(filteredAccelX) < accel_threshold_1) filteredAccelX = 0;
    filteredAccelX = fabs(filteredAccelX);
    acc_velocity_x = acc_velocity_x * velocity_damping_1 + filteredAccelX * dt;
    
    if (acc_velocity_x < 0.001) acc_velocity_x = 0;
    acc_distance_x += acc_velocity_x * dt; 

    float acc_velocity_x_km_h = acc_velocity_x * 3.6;
    float distance_x_km = acc_distance_x / 1000.0;
    
    //Serial.print("Accel X (m/s²): "); Serial.print(filteredAccelX, 4);
    //Serial.print(" | Velocity X (km/h): "); Serial.print(acc_velocity_x_km_h, 4);
    //Serial.print(" | Distance X (km): "); Serial.println(distance_x_km, 6);

    acc_avg_fuel_per_km = (distance_x_km > 0) ? (total_fuel_passed / distance_x_km) : 0.0;
}

String get_sender_number(String sms) {
    int startIndex = sms.indexOf("+CMT:");  
    if (startIndex != -1) {
        int firstQuote = sms.indexOf("\"", startIndex + 6);
        int secondQuote = sms.indexOf("\"", firstQuote + 1);
        if (firstQuote != -1 && secondQuote != -1) {
            String sender_number = sms.substring(firstQuote + 1, secondQuote);
            sender_number.trim();

            String stored_phone = PHONE_NUMBER;
            stored_phone.trim();

            if (sender_number == stored_phone) {
                return "USER_VERIFIED";
            } else {
                return "UNKNOWN_USER";
            }
        }
    }
    return "UNKNOWN_USER";
}

void gsm_data() {
    delay(200);
    String sms_response = "";

    while (serial_gsm.available()) {
        char c = (char)serial_gsm.read();
        sms_response += c;
    }

    if (sms_response.length() > 0) {
        sms_response.trim();
        serial_monitor.println("\nSMS Response:");
        serial_monitor.println(sms_response);

        String verification_status = get_sender_number(sms_response);
        serial_monitor.println("Verification Status: " + verification_status);

        if (verification_status == "USER_VERIFIED" && sms_response.indexOf("status") != -1) {
            serial_gsm.println("AT+CMGS=\"" + PHONE_NUMBER + "\"");
            delay(500);
            serial_gsm.print("System is ONLINE.");
            serial_gsm.write(26);
            delay(5000);
            serial_monitor.println("SMS Sent Successfully.");
        } else {
            //serial_monitor.println("Unauthorized sender or incorrect message. Ignoring SMS.");
        }

        if (verification_status == "USER_VERIFIED" && sms_response.indexOf("location") != -1) {
            serial_gsm.println("AT+CMGS=\"" + PHONE_NUMBER + "\"");
            delay(500);
            serial_gsm.print("lat : ");
            serial_gsm.println(print_latitude, 6);
            serial_gsm.print("long : ");
            serial_gsm.print(print_longitude, 6);
            serial_gsm.write(26);
            delay(5000);
            serial_monitor.println("SMS Sent Successfully.");
        } else {
            //serial_monitor.println("Unauthorized sender or incorrect message. Ignoring SMS.");
        }

        if (verification_status == "USER_VERIFIED" && sms_response.indexOf("speed") != -1) {
            serial_gsm.println("AT+CMGS=\"" + PHONE_NUMBER + "\"");
            delay(500);
            serial_gsm.print("speed : ");
            serial_gsm.println(current_speed, 2);
            serial_gsm.print(" km/h");
            serial_gsm.write(26);
            delay(5000);
            serial_monitor.println("SMS Sent Successfully.");
        } else {
            //serial_monitor.println("Unauthorized sender or incorrect message. Ignoring SMS.");
        }

        if (verification_status == "USER_VERIFIED" && sms_response.indexOf("time") != -1) {
            serial_gsm.println("AT+CMGS=\"" + PHONE_NUMBER + "\"");
            delay(500);
            serial_gsm.print("date: ");
            serial_gsm.print(date_day);
            serial_gsm.print("/");
            serial_gsm.print(date_month);
            serial_gsm.print("/");
            serial_gsm.println(date_year);
            serial_gsm.print("time: ");
            serial_gsm.print(time_hour);
            serial_gsm.print(":");
            serial_gsm.print(time_minute);
            serial_gsm.print(":");
            serial_gsm.println(time_second);
            serial_gsm.write(26);
            delay(5000);
            serial_monitor.println("SMS Sent Successfully.");
        } else {
            //serial_monitor.println("Unauthorized sender or incorrect message. Ignoring SMS.");
        }

        if (verification_status == "USER_VERIFIED" && sms_response.indexOf("fuel_rate") != -1) {
            serial_gsm.println("AT+CMGS=\"" + PHONE_NUMBER + "\"");
            delay(500);
            serial_gsm.print("fuel rate : ");
            serial_gsm.println(flow_rate);
            serial_gsm.print(" L/min");
            serial_gsm.write(26);
            delay(5000);
            serial_monitor.println("SMS Sent Successfully.");
        } else {
            //serial_monitor.println("Unauthorized sender or incorrect message. Ignoring SMS.");
        }

        if (verification_status == "USER_VERIFIED" && sms_response.indexOf("fuel_used") != -1) {
            serial_gsm.println("AT+CMGS=\"" + PHONE_NUMBER + "\"");
            delay(500);
            serial_gsm.print("fuel used : ");
            serial_gsm.println(total_fuel_passed);
            serial_gsm.print(" L");
            serial_gsm.write(26);
            delay(5000);
            serial_monitor.println("SMS Sent Successfully.");
        } else {
            //serial_monitor.println("Unauthorized sender or incorrect message. Ignoring SMS.");
        }

        if (verification_status == "USER_VERIFIED" && sms_response.indexOf("avg_gps") != -1) {
            serial_gsm.println("AT+CMGS=\"" + PHONE_NUMBER + "\"");
            delay(500);
            serial_gsm.print("fuel used: ");
            serial_gsm.print(total_fuel_passed);
            serial_gsm.println(" L");
            serial_gsm.print("gps distance traveled: ");
            serial_gsm.print(total_distance_traveled);
            serial_gsm.println(" km");
            serial_gsm.print("fuel consumption: ");
            serial_gsm.print(gps_avg_fuel_per_km);
            serial_gsm.println(" L/km");
            serial_gsm.write(26);
            delay(5000);
            serial_monitor.println("SMS Sent Successfully.");
        } else {
            //serial_monitor.println("Unauthorized sender or incorrect message. Ignoring SMS.");
        }

        if (verification_status == "USER_VERIFIED" && sms_response.indexOf("avg_static") != -1) {
            serial_gsm.println("AT+CMGS=\"" + PHONE_NUMBER + "\"");
            delay(500);
            serial_gsm.print("fuel used: ");
            serial_gsm.print(total_fuel_passed);
            serial_gsm.println(" L");
            serial_gsm.print(acc_distance_x_km);
            serial_gsm.println(" km");
            serial_gsm.print("fuel consumption: ");
            serial_gsm.print(acc_avg_fuel_per_km);
            serial_gsm.println(" L/km");
            serial_gsm.write(26);
            delay(5000);
            serial_monitor.println("SMS Sent Successfully.");
        } else {
            //serial_monitor.println("Unauthorized sender or incorrect message. Ignoring SMS.");
        }
    }
    while (serial_gsm.available()) serial_gsm.read(); 
}

void setup() {
    serial_monitor.begin(115200);
    serial_gsm.begin(115200, SERIAL_8N1);
    serial_gps.begin(9600);
    
    init_gsm();
    init_flow_sensor();
    init_accel();
    
    prevrime_1 = millis();
    
    delay(2000);

    serial_monitor.println("Ready to receive SMS.");
}

void loop() {
    gsm_data();
    gps_data();
    flow_sensor_data();
    accel_data();
}