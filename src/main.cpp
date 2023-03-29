#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <AccelMotor.h>

#define MAX_SPEED 70          // максимальная скорость моторов, в тиках в секунду!
#define MIN_DUTY 50           // мин. сигнал, при котором мотор начинает движение
#define STEP_SIZE 50          // перемещение по кнопкам крестовины (в тиках)
#define ACCEL 7               // ускорение
#define MAX_FOLLOW_SPEED 500  // макс. скорость

// коэффициенты ПИД
#define PID_P 2.2
#define PID_I 0.4
#define PID_D 0.01

// пины энкодеров
#define OPTO_FL 10
#define OPTO_FR 11
#define OPTO_BL 12
#define OPTO_BR 13

// при запуске крутятся ВПЕРЁД по очереди:
// FL - передний левый
// FR - передний правый
// BL - задний левый
// BR - задний правый

// пины драйверов (_B должен быть ШИМ)
#define MOTOR1_A 2
#define MOTOR1_B 3  // ШИМ!
#define MOTOR2_A 4
#define MOTOR2_B 5  // ШИМ!
#define MOTOR3_A 7
#define MOTOR3_B 6  // ШИМ!
#define MOTOR4_A 8
#define MOTOR4_B 9  // ШИМ!

// тут можно поменять моторы местами
AccelMotor motorBL(DRIVER2WIRE, MOTOR1_A, MOTOR1_B, HIGH);
AccelMotor motorFL(DRIVER2WIRE, MOTOR2_A, MOTOR2_B, HIGH);
AccelMotor motorBR(DRIVER2WIRE, MOTOR3_A, MOTOR3_B, HIGH);
AccelMotor motorFR(DRIVER2WIRE, MOTOR4_A, MOTOR4_B, HIGH);

class encCounter {
public:
    encCounter(byte pin) {
        _pin = pin;
    }
    long update(int direction) {      // приняли
        bool curState = pinRead(_pin);  // опрос
        if (_lastState != curState) {   // словили изменение
            _lastState = curState;
            if (curState) {           // по фронту
                _counter += direction;  // запомнили поворот
            }
        }
        return _counter;  // вернули
    }

private:
    long _counter = 0;
    byte _pin;
    bool _lastState = 0;
    // быстрый digitalRead для atmega328
    bool pinRead(uint8_t pin) {
        if (pin < 8) {
            return bitRead(PIND, pin);
        } else if (pin < 14) {
            return bitRead(PINB, pin - 8);
        } else if (pin < 20) {
            return bitRead(PINC, pin - 14);
        }
    }
};

encCounter encFL(OPTO_FL);
encCounter encFR(OPTO_FR);
encCounter encBL(OPTO_BL);
encCounter encBR(OPTO_BR);

RF24 radio(1, 10);  // модуль

byte address[][6] = { "1Node", "2Node", "3Node", "4Node", "5Node", "6Node" };  // возможные номера труб
int val[3][3];

void setup() {
    Serial.begin(9600);        // открываем порт для связи с ПК
    radio.begin();             // активировать модуль
    radio.setAutoAck(1);       // режим подтверждения приёма, 1 вкл 0 выкл
    radio.setRetries(0, 15);   // (время между попыткой достучаться, число попыток)
    radio.enableAckPayload();  // разрешить отсылку данных в ответ на входящий сигнал
    radio.setPayloadSize(32);  // размер пакета, в байтах

    radio.openReadingPipe(1, address[0]);  // хотим слушать трубу 0
    radio.setChannel(0x70);                // выбираем канал (в котором нет шумов!)

    radio.setPALevel(RF24_PA_MAX);  // уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
    radio.setDataRate(RF24_1MBPS);  // скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
    // должна быть одинакова на приёмнике и передатчике!
    // при самой низкой скорости имеем самую высокую чувствительность и дальность!!
    // ВНИМАНИЕ!!! enableAckPayload НЕ РАБОТАЕТ НА СКОРОСТИ 250 kbps!

    radio.powerUp();         // начать работу
    radio.startListening();  // начинаем слушать эфир, мы приёмный модуль

    // // чуть подразгоним ШИМ https://alexgyver.ru/lessons/pwm-overclock/
    // // Пины D3 и D11 - 980 Гц
    // TCCR2B = 0b00000100;  // x64
    // TCCR2A = 0b00000011;  // fast pwm

    // // Пины D9 и D10 - 976 Гц
    // TCCR1A = 0b00000001;  // 8bit
    // TCCR1B = 0b00001011;  // x64 fast pwm
    // минимальный сигнал на мотор
    motorFR.setMinDuty(MIN_DUTY);
    motorBR.setMinDuty(MIN_DUTY);
    motorFL.setMinDuty(MIN_DUTY);
    motorBL.setMinDuty(MIN_DUTY);

    motorFR.setMode(AUTO);
    motorBR.setMode(AUTO);
    motorFL.setMode(AUTO);
    motorBL.setMode(AUTO);

    // период интегрирования
    motorFR.setDt(30);
    motorBR.setDt(30);
    motorFL.setDt(30);
    motorBL.setDt(30);

    // режим управления мотора в PID_SPEED
    motorFR.setRunMode(PID_SPEED);
    motorBR.setRunMode(PID_SPEED);
    motorFL.setRunMode(PID_SPEED);
    motorBL.setRunMode(PID_SPEED);

    // ПИДы
    motorFR.kp = PID_P;
    motorFL.kp = PID_P;
    motorBR.kp = PID_P;
    motorBL.kp = PID_P;

    motorFR.ki = PID_I;
    motorFL.ki = PID_I;
    motorBR.ki = PID_I;
    motorBL.ki = PID_I;

    motorFR.kd = PID_D;
    motorFL.kd = PID_D;
    motorBR.kd = PID_D;
    motorBL.kd = PID_D;

//    Serial.println("front left");
//    motorFL.run(FORWARD, 100);
//    delay(3000);
//    motorFL.run(STOP);
//    delay(1000);
//    Serial.println("front right");
//    motorFR.run(FORWARD, 100);
//    delay(3000);
//    motorFR.run(STOP);
//    delay(1000);
//    Serial.println("back left");
//    motorBL.run(FORWARD, 100);
//    delay(3000);
//    motorBL.run(STOP);
//    delay(1000);
//    Serial.println("back right");
//    motorBR.run(FORWARD, 100);
//    delay(3000);
//    motorBR.run(STOP);

}

long posFR = 0;
long posBR = 0;
long posFL = 0;
long posBL = 0;

void updatePos() {
    motorFR.setTarget(posFR);
    motorBR.setTarget(posBR);
    motorFL.setTarget(posFL);
    motorBL.setTarget(posBL);
}
uint32_t tmr;
uint32_t radio_tmr;

void loop(void) {
    byte pipeNo;
    while (radio.available(&pipeNo)) {  // слушаем эфир со всех труб
        radio.read(&val, sizeof(int)*9);              // чиатем входящий сигнал
//        Serial.print(   map(val[0][0],-3, 3, -MAX_SPEED, MAX_SPEED)); Serial.print(",");
//        Serial.print(   map(val[0][1],-3, 3, -MAX_SPEED, MAX_SPEED)); Serial.print(",");
//        Serial.println( map(val[0][2],-3, 3, -MAX_SPEED, MAX_SPEED));
        Serial.print(   val[0][0]); Serial.print(",");
        Serial.print(   val[0][1]); Serial.print(",");
        Serial.println( val[0][2]);
        // отправляем обратно то что приняли
        radio.writeAckPayload(pipeNo, &val, sizeof(int)*9);
        Serial.print("Recieved");
        // Serial.println(val[0][0]);
        radio_tmr = millis();

        if (millis() - tmr >= 50) {
            tmr += 50;

            // переводим диапазон 0..255 в -MAX_SPEED..MAX_SPEED
            int val0Z = map(val[0][0], -3, 3, -MAX_SPEED, MAX_SPEED);
            int val0X = map(val[0][1], -3, 3, -MAX_SPEED, MAX_SPEED);
            int val0Y = map(val[0][2], -3, 3, -MAX_SPEED, MAX_SPEED);

            int dutyFR = val0Y + val0X;
            int dutyFL = val0Y - val0X;
            int dutyBR = val0Y - val0X;
            int dutyBL = val0Y + val0X;

            dutyFR += -val0Z;
            dutyFL += +val0Z;
            dutyBR += -val0Z;
            dutyBL += +val0Z;

            // ПИД контроль скорости
            motorFR.setTargetSpeed(dutyFR);
            motorBR.setTargetSpeed(dutyBR);
            motorFL.setTargetSpeed(dutyFL);
            motorBL.setTargetSpeed(dutyBL);
        }
    }
    if (millis()-radio_tmr>=1000){
        motorFR.setTargetSpeed(0);
        motorBR.setTargetSpeed(0);
        motorFL.setTargetSpeed(0);
        motorBL.setTargetSpeed(0);
    }
    bool m1 = motorFR.tick(encFR.update(motorFR.getState()));
    bool m2 = motorBR.tick(encBR.update(motorBR.getState()));
    bool m3 = motorFL.tick(encFL.update(motorFL.getState()));
    bool m4 = motorBL.tick(encBL.update(motorBL.getState()));
}
ISR(PCINT0_vect) {  // пины 8-13
    encFR.update(motorFR.getState());
    encBR.update(motorBR.getState());
    encFL.update(motorFL.getState());
    encBL.update(motorBL.getState());
}

// функция для настройки PCINT
uint8_t attachPCINT(uint8_t pin) {
    if (pin < 8) {  // D0-D7 (PCINT2)
        PCICR |= (1 << PCIE2);
        PCMSK2 |= (1 << pin);
        return 2;
    } else if (pin > 13) {  //A0-A5 (PCINT1)
        PCICR |= (1 << PCIE1);
        PCMSK1 |= (1 << pin - 14);
        return 1;
    } else {  // D8-D13 (PCINT0)
        PCICR |= (1 << PCIE0);
        PCMSK0 |= (1 << pin - 8);
        return 0;
    }
}