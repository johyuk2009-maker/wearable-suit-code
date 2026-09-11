#include <ESP32Servo.h>
#include <math.h>

// =====================================================
// 1. 핀 설정
// =====================================================

// EMG 센서
const int QUAD_PIN = 34;       // 대퇴사두근 EMG
const int HAM_PIN  = 35;       // 햄스트링 EMG

// Flex 센서
const int FLEX_PIN = 32;

// 서보모터
const int SERVO_PIN = 25;


// =====================================================
// 2. 설정값
// =====================================================

const int RMS_WINDOW = 50;

// EMG 두 근육의 차이가 이 비율 이상일 때 방향 결정
const float DEAD_BAND = 1.20;

// 모델 다리의 각도 제한
const float MIN_KNEE_ANGLE = 40.0;
const float MAX_KNEE_ANGLE = 170.0;


// =====================================================
// 3. Flex 센서 보정값
// =====================================================

// 실제 Flex 센서의 측정값에 맞게 수정해야 함
//
// FLEX_STRAIGHT_RAW
// → 다리가 펴졌을 때의 ADC 값
//
// FLEX_BENT_RAW
// → 다리가 구부러졌을 때의 ADC 값

const int FLEX_STRAIGHT_RAW = 1800;
const int FLEX_BENT_RAW     = 3000;


// =====================================================
// 4. 서보 설정
// =====================================================

Servo kneeServo;

// 서보가 움직일 실제 범위
const int SERVO_MIN = 0;
const int SERVO_MAX = 180;


// =====================================================
// 5. 전역 변수
// =====================================================

float activationThreshold = 0.0;

float quadRMS = 0.0;
float hamRMS = 0.0;

float kneeAngle = 90.0;

String intent = "IDLE";


// =====================================================
// 6. EMG RMS 계산
// =====================================================

float calculateRMS(int pin)
{
    double sumSquares = 0;

    for (int i = 0; i < RMS_WINDOW; i++)
    {
        int value = analogRead(pin);

        /*
         * ESP32 ADC는 기본적으로 0~4095 범위.
         *
         * EMG 센터값이 약 2048이라고 가정.
         * 실제 센서에 따라 조정 필요.
         */
        value -= 2048;

        sumSquares += (double)value * value;

        delayMicroseconds(500);
    }

    return sqrt(sumSquares / RMS_WINDOW);
}


// =====================================================
// 7. EMG 캘리브레이션
// =====================================================

void calibrateEMG()
{
    bool calibrated = false;

    while (!calibrated)
    {
        Serial.println();
        Serial.println("================================");
        Serial.println("EMG CALIBRATION");
        Serial.println("다리에 힘을 빼고 편안하게 유지하세요.");
        Serial.println("5초 동안 측정합니다.");
        Serial.println("================================");

        delay(1000);

        float rmsSum = 0.0;
        float rmsMax = 0.0;

        int sampleCount = 0;

        unsigned long startTime = millis();

        while (millis() - startTime < 5000)
        {
            float quad = calculateRMS(QUAD_PIN);
            float ham  = calculateRMS(HAM_PIN);

            float strongest = max(quad, ham);

            rmsSum += strongest;

            if (strongest > rmsMax)
            {
                rmsMax = strongest;
            }

            sampleCount++;

            delay(10);
        }

        float rmsAvg = rmsSum / sampleCount;

        Serial.println();
        Serial.print("평균 RMS: ");
        Serial.println(rmsAvg);

        Serial.print("최대 RMS: ");
        Serial.println(rmsMax);

        /*
         * 캘리브레이션 중 신호 변화가 지나치게 큰 경우
         * 다시 측정
         */
        if ((rmsMax - rmsAvg) > 10.0)
        {
            Serial.println("움직임 또는 큰 신호 변화가 감지되었습니다.");
            Serial.println("캘리브레이션을 다시 시작합니다.");

            delay(2000);
        }
        else
        {
            /*
             * 기준값보다 약간 높은 값을
             * 실제 활성화 기준으로 사용
             */
            activationThreshold = rmsMax * 1.2;

            calibrated = true;

            Serial.println();
            Serial.println("캘리브레이션 완료!");

            Serial.print("Activation Threshold: ");
            Serial.println(activationThreshold);

            delay(1000);
        }
    }
}


// =====================================================
// 8. Flex 센서 → 무릎 각도 변환
// =====================================================

float getKneeAngle()
{
    int rawValue = analogRead(FLEX_PIN);

    /*
     * Flex 센서의 ADC 값을
     * 40~170도의 무릎 각도로 변환
     */
    float angle = map(
        rawValue,
        FLEX_STRAIGHT_RAW,
        FLEX_BENT_RAW,
        MAX_KNEE_ANGLE,
        MIN_KNEE_ANGLE
    );

    // 각도 범위를 40~170도로 제한
    angle = constrain(
        angle,
        MIN_KNEE_ANGLE,
        MAX_KNEE_ANGLE
    );

    return angle;
}


// =====================================================
// 9. 움직임 의도 판단
// =====================================================

void detectIntent()
{
    quadRMS = calculateRMS(QUAD_PIN);
    hamRMS  = calculateRMS(HAM_PIN);

    float strongest = max(quadRMS, hamRMS);

    // 기본 상태
    intent = "IDLE";


    // 기준값보다 신호가 강할 때만 판단
    if (strongest > activationThreshold)
    {
        // 대퇴사두근이 20% 이상 강함
        if (quadRMS > hamRMS * DEAD_BAND)
        {
            intent = "EXTEND";
        }

        // 햄스트링이 20% 이상 강함
        else if (hamRMS > quadRMS * DEAD_BAND)
        {
            intent = "FLEX";
        }

        // 두 신호가 비슷함
        else
        {
            intent = "IDLE";
        }
    }
}


// =====================================================
// 10. 서보 정지 위치
// =====================================================

void stopServo()
{
    /*
     * 서보는 DC 모터처럼 즉시 "PWM 0"으로 정지시키는
     * 방식이 아니다.
     *
     * 모델 테스트에서는 현재 각도를 유지하도록 한다.
     */
}


// =====================================================
// 11. 서보 제어
// =====================================================

void controlServo()
{
    int targetAngle = (int)kneeAngle;


    // -----------------------------------------------
    // EXTEND
    // -----------------------------------------------

    if (intent == "EXTEND")
    {
        /*
         * 무릎 각도가 최대 제한에 도달하지 않은 경우
         * 펴지는 방향으로 이동
         */

        if (kneeAngle < MAX_KNEE_ANGLE)
        {
            targetAngle = (int)kneeAngle + 5;

            targetAngle = constrain(
                targetAngle,
                (int)MIN_KNEE_ANGLE,
                (int)MAX_KNEE_ANGLE
            );

            kneeServo.write(targetAngle);

            Serial.println("SERVO: EXTEND");
        }
        else
        {
            stopServo();

            Serial.println("SERVO: EXTEND BLOCKED");
        }
    }


    // -----------------------------------------------
    // FLEX
    // -----------------------------------------------

    else if (intent == "FLEX")
    {
        /*
         * 무릎 각도가 최소 제한보다 큰 경우
         * 접히는 방향으로 이동
         */

        if (kneeAngle > MIN_KNEE_ANGLE)
        {
            targetAngle = (int)kneeAngle - 5;

            targetAngle = constrain(
                targetAngle,
                (int)MIN_KNEE_ANGLE,
                (int)MAX_KNEE_ANGLE
            );

            kneeServo.write(targetAngle);

            Serial.println("SERVO: FLEX");
        }
        else
        {
            stopServo();

            Serial.println("SERVO: FLEX BLOCKED");
        }
    }


    // -----------------------------------------------
    // IDLE
    // -----------------------------------------------

    else
    {
        stopServo();

        Serial.println("SERVO: IDLE");
    }
}


// =====================================================
// 12. setup
// =====================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("================================");
    Serial.println("ESP32 EMG LEG ASSIST SYSTEM");
    Serial.println("================================");


    // ESP32 ADC 설정
    analogReadResolution(12);


    // -----------------------------------------------
    // 서보 초기화
    // -----------------------------------------------

    kneeServo.setPeriodHertz(50);

    kneeServo.attach(
        SERVO_PIN,
        500,
        2400
    );


    // 초기 위치
    kneeServo.write(90);

    delay(1000);


    // -----------------------------------------------
    // EMG 캘리브레이션
    // -----------------------------------------------

    calibrateEMG();


    Serial.println();
    Serial.println("================================");
    Serial.println("SYSTEM READY");
    Serial.println("================================");
}


// =====================================================
// 13. loop
// =====================================================

void loop()
{
    // -----------------------------------------------
    // 1. Flex 센서로 현재 각도 측정
    // -----------------------------------------------

    kneeAngle = getKneeAngle();


    // -----------------------------------------------
    // 2. EMG로 움직임 의도 판단
    // -----------------------------------------------

    detectIntent();


    // -----------------------------------------------
    // 3. 상태에 따라 서보 제어
    // -----------------------------------------------

    controlServo();


    // -----------------------------------------------
    // 4. Serial Monitor 출력
    // -----------------------------------------------

    Serial.println("-------------------------------");

    Serial.print("Quad RMS: ");
    Serial.println(quadRMS);

    Serial.print("Ham RMS: ");
    Serial.println(hamRMS);

    Serial.print("Threshold: ");
    Serial.println(activationThreshold);

    Serial.print("Flex Angle: ");
    Serial.println(kneeAngle);

    Serial.print("Intent: ");
    Serial.println(intent);

    Serial.println("-------------------------------");


    // 약 50ms 간격
    delay(50);
}
