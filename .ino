/ =========================
// 핀 설정
// =========================
const int QUAD_PIN       = A0;  // 대퇴사두근 EMG 센서
const int HAM_PIN        = A1;  // 햄스트링 EMG 센서
const int ANGLE_PIN      = A2;  // 무릎 각도 센서 (가변저항 등)

const int MOTOR_DIR_PIN  = 4;   // 모터 방향 제어 핀
const int MOTOR_PWM_PIN  = 5;   // 모터 속도 제어 핀 (PWM)

// =========================
// 설정값 (상수)
// =========================
const int RMS_WINDOW = 50;
const float DEAD_BAND = 1.20;

const float MAX_KNEE_ANGLE = 170.0; // 최대 신전 제한 각도
const float MIN_KNEE_ANGLE = 40.0;  // 최소 굴곡 제한 각도

const int MOTOR_SPEED = 150;        // 모터 구동 속도 (0 ~ 255)

// =========================
// 전역 변수
// =========================
float activationThreshold = 0.0;

float quadRMS = 0.0;
float hamRMS = 0.0;
float kneeAngle = 90.0;

String intent = "IDLE";

// =========================
// RMS 계산 (EMG 신호 처리)
// =========================
float calculateRMS(int pin)
{
    long sumSquares = 0;

    for(int i = 0; i < RMS_WINDOW; i++)
    {
        int value = analogRead(pin);
        
        // 하드웨어 신호의 DC 오프셋 제거 (중앙값 512 기준)
        // 사용하는 EMG 모듈에 따라 이 값을 조절해야 할 수 있습니다.
        value -= 512; 

        sumSquares += (long)value * value;
        delayMicroseconds(500); // 2kHz 샘플링 속도 유지
    }

    return sqrt((float)sumSquares / RMS_WINDOW);
}

// =========================
// EMG 초기 캘리브레이션
// =========================
void calibrateEMG()
{
    bool calibrated = false;

    while(!calibrated)
    {
        Serial.println("\n[Calibration] 다리에 힘을 빼고 편안히 계세요.");
        Serial.println("[Calibration] 5초간 기저 노이즈를 측정합니다...");

        float rmsSum = 0;
        float rmsMax = 0;
        int sampleCount = 0;

        unsigned long startTime = millis();

        while(millis() - startTime < 5000)
        {
            float quad = calculateRMS(QUAD_PIN);
            float ham = calculateRMS(HAM_PIN);
            float strongest = max(quad, ham);

            rmsSum += strongest;
            if(strongest > rmsMax) {
                rmsMax = strongest;
            }
            sampleCount++;
            delay(10);
        }

        float rmsAvg = rmsSum / sampleCount;

        // 캘리브레이션 기간 동안 움직임(노이즈)이 너무 심했는지 체크
        if((rmsMax - rmsAvg) > 10.0)
            Serial.println("[Error] 활동이 감지되었습니다. 다시 시작합니다.");
            delay(2000);
        }
        else
        {
            // 여유값(안전 마진)을 1.5배 정도 더해주면 오작동을 줄일 수 있습니다.
            activationThreshold = rmsMax * 1.2; 
            calibrated = true;

            Serial.print("[Success] 설정된 문턱값(Threshold): ");
            Serial.println(activationThreshold);
        }
    }
}

// =========================
// 무릎 각도 읽기
// =========================
float getKneeAngle()
{
    int rawValue = analogRead(ANGLE_PIN);
    
    // analogRead 값(0~1023)을 실제 무릎 각도(예: 0~180도)로 매핑
    // 실제 센서의 장착 방향과 회전 범위에 맞게 아래의 0, 1023, 0, 180 수치를 수정해야 합니다.
    float angle = map(rawValue, 0, 1023, 0, 180); 
    
    return angle;
}

// =========================
// 의도 판별
// =========================
void detectIntent()
{
    quadRMS = calculateRMS(QUAD_PIN);
    hamRMS = calculateRMS(HAM_PIN);

    float strongest = max(quadRMS, hamRMS);
    intent = "IDLE";

    // 문턱값을 넘는 근수축이 발생했을 때만 판별
    if(strongest > activationThreshold)
    {
        // 대퇴사두근이 햄스트링보다 20% 이상 강할 때 (신전)
        if(quadRMS > hamRMS * DEAD_BAND)
        {
            intent = "EXTEND";
        }
        // 햄스트링이 대퇴사두근보다 20% 이상 강할 때 (굴곡)
        else if(hamRMS > quadRMS * DEAD_BAND)
        {
            intent = "FLEX";
        }
    }
}

// =========================
// 모터 구동 제어 함수
// =========================
void driveMotor(String direction, int speed)
{
    if (direction == "EXTEND")
    {
        digitalWrite(MOTOR_DIR_PIN, HIGH); // 정방향 회전 설정 (하드웨어에 맞게 수정)
        analogWrite(MOTOR_PWM_PIN, speed);
    }
    else if (direction == "FLEX")
    {
        digitalWrite(MOTOR_DIR_PIN, LOW);  // 역방향 회전 설정 (하드웨어에 맞게 수정)
        analogWrite(MOTOR_PWM_PIN, speed);
    }
    else // IDLE 또는 STOP
    {
        analogWrite(MOTOR_PWM_PIN, 0);     // 모터 정지
    }
}

// =========================
// 상태 평가 및 동작 실행
// =========================
void evaluateState()
{
    if(intent == "EXTEND")
    {
        // 소프트웨어 안전 리미트 확인 (최대 각도 미만일 때만 허용)
        if(kneeAngle < MAX_KNEE_ANGLE)
        {
            Serial.println("STATUS: EXTEND ALLOWED");
            driveMotor("EXTEND", MOTOR_SPEED);
        }
        else
        {
            Serial.println("STATUS: EXTEND BLOCKED (MAX LIMIT)");
            driveMotor("STOP", 0);
        }
    }
    else if(intent == "FLEX")
    {
        // 소프트웨어 안전 리미트 확인 (최소 각도 초과일 때만 허용)
        if(kneeAngle > MIN_KNEE_ANGLE)
        {
            Serial.println("STATUS: FLEX ALLOWED");
            driveMotor("FLEX", MOTOR_SPEED);
        }
        else
        {
            Serial.println("STATUS: FLEX BLOCKED (MIN LIMIT)");
            driveMotor("STOP", 0);
        }
    }
    else
    {
        Serial.println("STATUS: IDLE");
        driveMotor("STOP", 0);
    }
}

// =========================
// setup
// =========================
void setup()
{
    Serial.begin(115200);

    // 모터 제어 핀 출력 설정
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    
    // 초기 모터 정지
    driveMotor("STOP", 0);

    // EMG 센서 초기화
    calibrateEMG();
}

// =========================
// loop
// =========================
void loop()
{
    // 1. 현재 센서 값들 업데이트
    kneeAngle = getKneeAngle();
    detectIntent();

    // 2. 의도 및 안전 제어 상태 평가 후 모터 구동
    evaluateState();

    // 3. 시리얼 모니터 데이터 출력 (디버깅용)
    Serial.print("Quad: ");     Serial.print(quadRMS);
    Serial.print(" | Ham: ");   Serial.print(hamRMS);
    Serial.print(" | Thresh: "); Serial.print(activationThreshold);
    Serial.print(" | Angle: ");  Serial.print(kneeAngle);
    Serial.print(" | Intent: "); Serial.println(intent);

    delay(10); // 루프 주기를 조금 더 빠르게 조절 (원래 50ms -> 10ms)
}
