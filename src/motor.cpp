#include <Arduino.h>
#include "motor.h"

/*
    Hanya untuk pembacaan encoder
*/
Motor::Motor(byte en_a, byte en_b){
    this->en_a = en_a;
    this->en_b = en_b;
    pinMode(this->en_a, INPUT);
    pinMode(this->en_b, INPUT);
}

/*
    Mode tanpa encoder
    ma_pin   -> Motor A pin
    mb_pin   -> Motor B pin
    pwm_pin -> Motor PWM pin
*/
Motor::Motor(byte ma_pin, byte mb_pin, byte pwm_pin)
{
    this->ma_pin = ma_pin;
    this->mb_pin = mb_pin;
    this->pwm_pin = pwm_pin;

    pinMode(this->ma_pin, OUTPUT);
    pinMode(this->mb_pin, OUTPUT);
    pinMode(this->pwm_pin, OUTPUT);
    // setPwmFrequency();
}
/*
    Mode dengan encoder
    ma_pin   -> Motor A pin
    mb_pin   -> Motor B pin
    pwm_pin -> Motor PWM pin
    en_a    -> Encoder channel A
    en_b    -> Encoder channel B
*/
Motor::Motor(byte ma_pin, byte mb_pin, byte pwm_pin, byte en_a, byte en_b)
{
    this->ma_pin = ma_pin;
    this->mb_pin = mb_pin;
    this->pwm_pin = pwm_pin;
    this->en_a = en_a;
    this->en_b = en_b;
    pinMode(this->ma_pin, OUTPUT);
    pinMode(this->mb_pin, OUTPUT);
    pinMode(this->pwm_pin, OUTPUT);
    pinMode(this->en_a, INPUT);
    pinMode(this->en_b, INPUT);
    digitalWrite(this->pwm_pin, LOW);
    // setPwmFrequency();
}

/*
    Mode dengan encoder dengan setting manual PPR
    PPR default 135 (PG-36)
    ma_pin   -> Motor A pin
    mb_pin   -> Motor B pin
    pwm_pin -> Motor PWM pin
    en_a    -> Encoder channel A
    en_b    -> Encoder channel B
*/
Motor::Motor(byte ma_pin, byte mb_pin, byte pwm_pin, byte en_a, byte en_b, int ppr)
{
    this->ma_pin = ma_pin;
    this->mb_pin = mb_pin;
    this->pwm_pin = pwm_pin;
    this->en_a = en_a;
    this->en_b = en_b;
    this->ppr = ppr;
    pinMode(this->ma_pin, OUTPUT);
    pinMode(this->mb_pin, OUTPUT);
    pinMode(this->pwm_pin, OUTPUT);
    pinMode(this->en_a, INPUT);
    pinMode(this->en_b, INPUT);
    digitalWrite(this->pwm_pin, LOW);
}
/*
    Kp, Ki, Kd, Windup
    
    Kp
    + Cepat mencapai setpoint
    - Menimbulkan osilasi
    
    Ki
    + Koreksi pada interval tertentu
    - Menimbulkan windup
    
    Kd
    + Menahan kondisi stabil, meredam osilasi
    - Akan stuck pada kondisi sebelum setpoint

    Rumus PID = (kp * error * 0.1) + (ki * integral error* 0.01) + (kd * (error - last Error) * 0.1)
*/
void Motor::pid(float kp, float ki, float kd, float windup)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->windup = windup;
    this->pidEnable = true;
}
/*
    Atur batas PWM PID yang dihasilkan
    Range yang diperbolehkan antara 0-255
    default 0-150
*/
void Motor::setPidThreshold(int min_pwm, int max_pwm)
{
    this->min_pwm = min_pwm;
    this->max_pwm = max_pwm;
}

/*
    Mengatur kecepatan motor, apabila PID dimatikan maka akan menggunakan
    nilai dari pwm yg dimasukkan. Apabila PID enable maka nilai yang
    dimasukkan adalah nilai setpoint RPM. Nilai minus untuk CCW
    dan nilai plus untuk CW
*/
void Motor::speed(float target)
{
    if (pidEnable == true)
    {
        if (i_err > windup)
            i_err = 0;
        err = abs(target + correction) - rpm_abs;
        d_err = err - last_err;
        last_err = err;
        i_err = i_err + err;
        pwm_pid = (kp * err) + (kd * d_err) + (ki * i_err);
        if (pwm_pid <= min_pwm)
            pwm_pid = min_pwm;
        if (pwm_pid >= max_pwm)
            pwm_pid = max_pwm;
        target > 0 ? forward(pwm_pid) : reverse(pwm_pid);
        // Serial.print("sp : ");
        // Serial.print(target);
        // Serial.print(" rpm : ");
        // Serial.println(rpm);
    }
    else
    {
        if (target > 255)
            target = 255;
        else if (target < -255)
            target = -255;
        target > 0 ? forward(target) : reverse(target);
    }
}

/*
    Motor bergerak clock wise
*/
void Motor::forward(int pwm)
{
    if (abs(pwm) < threshold)
        pwm = 0;
    digitalWrite(this->ma_pin, HIGH);
    digitalWrite(this->mb_pin, LOW);
    analogWrite(this->pwm_pin, pwm);
}

/*
    Motor bergerak counter clock wise
*/
void Motor::reverse(int pwm)
{
    if (abs(pwm) < threshold)
        pwm = 0;
    digitalWrite(this->ma_pin, LOW);
    digitalWrite(this->mb_pin, HIGH);
    analogWrite(this->pwm_pin, abs(pwm));
}

#if defined(EMS) || defined(L298)
/*
    Khusus driver EMS terdapat fungsi untuk mengerem
    motor dengan membuat pin A dan pin B menjadi HIGH
*/
void Motor::brake()
{
    digitalWrite(this->ma_pin, HIGH);
    digitalWrite(this->mb_pin, HIGH);
}
#else

/*
    Motor berhenti, bisa diatur untuk driver EMS
    dengan mendefinisikan #define EMS pada Motor.h
*/
void Motor::brake()
{
    digitalWrite(this->ma_pin, LOW);
    digitalWrite(this->mb_pin, LOW);
}
#endif

/*
    Fungsi berikut adalah untuk merecord hasil pembacaan encoder motor.
    encoder_tick adalah variabel yang akan direset pada interval tertentu
    untuk mendapatkan kecepatan motor, sedangkan encoder_tick_acc
    adalah variabel yang tidak akan direset selama sistem berjalan
*/
void Motor::isrHandler()
{
    digitalRead(en_b) == LOW ? encoder_tick-- : encoder_tick++;
    digitalRead(en_b) == LOW ? encoder_tick_acc-- : encoder_tick_acc++;
}

/*
    Masukkan waktu sampling (ms) sama persis dengan waktu timer interrupt.
    Misal sampling rpm adalah 10 ms:

    int sampling_time_ms = 10;
    Timer1.initialize(1000 * sampling_time_ms);
    maka
    m.calculateRpm(sampling_time_ms);
    
*/
void Motor::calculateRpm(int sampling_time_ms)
{
#if (SAM3XA_SERIES) || (SAM3N_SERIES) || (SAM3S_SERIES)
    sampling_time_ms = sampling_time_ms / 2;
#endif
    rpm = (encoder_tick / ppr) * (60000 / sampling_time_ms);
    rpm_abs = abs(rpm);
    speed_ms = (PI * d_wheel) / 6000.000 * rpm;
    encoder_tick = 0;
}

void Motor::setPwmThreshold(int threshold)
{
    this->threshold = threshold;
}

void Motor::setPwmFrequency()
{
#if (SAM3XA_SERIES) || (SAM3N_SERIES) || (SAM3S_SERIES)
    uint32_t pwmPin = pwm_pin;
    uint32_t maxDutyCount = 2;
    uint32_t clkAFreq = 42000000ul;
    uint32_t pwmFreq = 42000000ul;
    pmc_enable_periph_clk(PWM_INTERFACE_ID);
    PWMC_ConfigureClocks(clkAFreq, 0, VARIANT_MCK);
    PIO_Configure(
        g_APinDescription[pwmPin].pPort,
        g_APinDescription[pwmPin].ulPinType,
        g_APinDescription[pwmPin].ulPin,
        g_APinDescription[pwmPin].ulPinConfiguration);
    uint32_t channel = g_APinDescription[pwmPin].ulPWMChannel;
    PWMC_ConfigureChannel(PWM_INTERFACE, channel, pwmFreq, 0, 0);
    PWMC_SetPeriod(PWM_INTERFACE, channel, maxDutyCount);
    PWMC_EnableChannel(PWM_INTERFACE, channel);
    PWMC_SetDutyCycle(PWM_INTERFACE, channel, 1);
    pmc_mck_set_prescaler(2);
#else
#endif
}

/*
    Mendapatkan data total jarak yg ditempuh oleh roda
    dalam satuan meter
*/
float Motor::getDistance()
{
    return (encoder_tick_acc / ppr) * pi * d_wheel;
}

/*
    Set diameter roda dalam satuan meter
*/
void Motor::setWheelDiameter(float diameter){
    this->d_wheel = diameter;
}