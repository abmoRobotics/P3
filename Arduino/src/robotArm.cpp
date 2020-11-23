#include <robotArm.h>
#include <Dynamixel2Arduino.h>

robotArm::robotArm(Dynamixel2Arduino &dxl2)
{
    dxl = &dxl2;
    const float DXL_PROTOCOL_VERSION = 2.0;
    dxl->begin(57600);
    dxl->setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    startMotors();
}

void robotArm::setGripperTorque(byte motorID, byte goalTorque[])
{
    digitalWrite(LED_BUILTIN, HIGH);
    if(goalTorque[2] == 0x00)
    {
        float goalPWM = 8.5*((goalTorque[0] << 8) | goalTorque[1]);
        dxl->setGoalPWM(motorID, goalPWM);
    }
    else if(goalTorque[2] == 0x01)
    {
        dxl->torqueOff(5);
        dxl->torqueOff(6);
        dxl->setOperatingMode(5, OP_POSITION);
        dxl->setOperatingMode(6, OP_POSITION);
        dxl->torqueOn(5);
        dxl->torqueOn(6);
        dxl->setGoalPosition(5, 1900);
        dxl->setGoalPosition(6, 2850);
        delay(1000);
        dxl->torqueOff(5);
        dxl->torqueOff(6);
        dxl->setOperatingMode(5, OP_PWM);
        dxl->setOperatingMode(6, OP_PWM);
        dxl->torqueOn(5);
        dxl->torqueOn(6);
        dxl->setGoalPWM(5, 0);
        dxl->setGoalPWM(6, 0);

    }
    
}

double robotArm::getTorque(int motorID)
{
    double measuredTorque = dxl->getPresentCurrent(motorID);
    return measuredTorque; 
}

void robotArm::setTorque(byte motorID, byte goalTorque_ptr[])
{
    
}
void robotArm::setPWM(int motorID, float PWM)
{
    dxl->setGoalPWM(motorID, PWM);
}

int16_t robotArm::getPosition(int motorID)
{
    int16_t measuredPos = dxl->getPresentPosition(motorID);
    float radianPos = ((2 * PI / 4095) * measuredPos);
    char firstByte = (byte)measuredPos;         //
    char secondByte = (byte)(measuredPos >> 8); // Shift 8 bit to left
    //Serial.write(firstByte);                    // Write first byte representing a number from 0-255
    //Serial.write(secondByte);                   // Write second byte representing a number from 256 til noget stort(ca 32000)
    return measuredPos;
} // Position in radians

double robotArm::getPositionRad(int motorID){
    int zeroPOS1 = 1290;
    int zeroPOS2 = 2015;
    int zeroPOS3 = 1090;
    int zeroPOS4 = 2045;
    int Resolution = 4095;
    int measuredPOS = dxl->getPresentPosition(motorID);
    
    if (motorID==1)
    {
        measuredPOS = measuredPOS - zeroPOS1;
    }
    if (motorID==2)
    {
        measuredPOS = measuredPOS - zeroPOS2;
    }
    if (motorID==3)
    {
        measuredPOS = measuredPOS - zeroPOS3;
    }
    if (motorID==4)
    {
        measuredPOS = measuredPOS - zeroPOS4;
    }
    
    float measuredPOSRad = ((2 * PI / Resolution) * measuredPOS);

    return measuredPOSRad;
}

void robotArm::setPosition(int motorID, int16_t goalPos)
{
    dxl->setGoalPosition(motorID, goalPos);
}

double robotArm::getVelocity(int motorID)
{
    double measuredVel = dxl->getPresentVelocity(motorID) * 0.023980823895;
    return measuredVel;
}

double robotArm::calculatePWM(int motorid, float torque)
{
    int PWM;
    float C1MX28{538.42}, C2MX28{149.78}, C1MX64{197.71}, C2MX64{124.29}, C1MX106{120.66}, C2MX106{165.78};
    float angularVel = dxl->getPresentVelocity(motorid) * 0.229 * 0.104719755;

    if (motorid == 1 || motorid == 2)
    {
        PWM = torque * C1MX106 + angularVel * C2MX106;
    }
    else if (motorid == 3)
    {
        PWM = torque * C1MX64 + angularVel * C2MX64;
    }
    else if (motorid == 4 || motorid == 5 || motorid == 6)
    {
        PWM = torque * C1MX28 + angularVel * C2MX28;
    }
    return PWM;
}

void robotArm::setVelocity(int motorID, byte goalVel_ptr[])
{

    byte goalVel = (goalVel_ptr[0] << 8) | goalVel_ptr[1];

    dxl->setGoalVelocity(motorID, goalVel);
}

double robotArm::calculateMass(int motorID, double Q1, double Q2, double Q3, double Q4){
double t2 = cos(Q1);
double t3 = cos(Q2);
double t4 = cos(Q3);
double t5 = cos(Q4);
double t6 = sin(Q1);
double t7 = sin(Q2);
double t8 = sin(Q3);
double t9 = sin(Q4);
double t10 = t2*t2;
double t11 = t3*t3;
double t12 = t4*t4;
double t13 = t3*7.23795776E-5;
double t14 = t9*7.232230896E-4;
double t15 = t2*t3*1.927698669E-6;
double t16 = t3*t4*-7.25855196E-5;
double t17 = t3*t4*7.25855196E-5;
double t18 = t3*t8*-1.3519085382E-3;
double t19 = t3*t8*1.3519085382E-3;
double t20 = t5*t7*-8.108864944E-4;
double t21 = t5*t7*8.108864944E-4;
double t22 = t6*t9*-7.232230896E-4;
double t24 = t3*t8*t9*-8.108864944E-4;
double t25 = t3*t8*t9*8.108864944E-4;
double t26 = t6*t8*t9*-1.88147582552E-3;
double t27 = t6*t8*t9*1.88147582552E-3;
double t28 = t2*t3*t4*6.0E-11;
double t29 = t2*t3*t8*-6.211091902400001E-5;
double t30 = t2*t3*t8*6.211091902400001E-5;
double t31 = t2*t5*t7*-7.232230896E-4;
double t32 = t2*t5*t7*7.232230896E-4;
double t33 = t4*t6*t9*6.834308976E-4;
double t34 = t5*t6*t8*-6.834308976E-4;
double t35 = t5*t6*t8*6.834308976E-4;
double t37 = t2*t3*t4*t5*6.834308976E-4;
double t38 = t2*t3*t8*t9*6.834308976E-4;
double t23 = t6*t14;
double t36 = t2*t7*t14;
double M;
    if (motorID==1)
    {
        M = t2*2.7030158864E-3+t4*3.3683604636E-4+t5*3.55591167024E-3-t6*1.428760506959E-3+t8*6.27358637862E-3-t10*1.140450473033E-3-t12*1.290494052896E-3+t13+t15+t16+t18+t20+t22+t24+t26+t28+t29+t31+t33+t34+t36+t37+t38+sin(Q1*2.0)*1.438251E-5+sin(Q3*2.0)*1.88147582552E-3+t2*t5*2.5970283672E-4-t2*t7*1.0027785350386E-2+t3*t6*1.14506653984E-2-t4*t6*2.30528942204E-4-t2*t11*2.7038170764E-3-t3*t10*3.3683604636E-4-t4*t9*1.3668617952E-3+t5*t8*1.3668617952E-3-t4*t10*3.3683604636E-4-t6*t8*3.13679324931E-3-t5*t10*5.00235784944E-3-t5*t12*3.55591167024E-3-t7*t10*6.27358637862E-3-t8*t10*6.27358637862E-3-t10*t11*1.558837051911E-3+t10*t12*1.290494052896E-3-t2*t3*t6*3.855397338E-6+t2*t3*t7*1.451710392E-4+t3*t4*t6*1.451710392E-4+t3*t5*t6*1.5325420128E-3+t2*t6*t7*1.626569764E-6+t3*t6*t8*2.7038170764E-3+t2*t5*t11*1.6217729888E-3-t3*t7*t10*1.7634261572E-5-t2*t8*t11*1.34178132E-5-t4*t8*t9*3.55591167024E-3-t2*t9*t11*1.5325420128E-3-t4*t8*t10*3.76295165104E-3+t5*t7*t10*3.76295165104E-3+t4*t9*t10*1.3668617952E-3-t5*t8*t10*1.3668617952E-3-t2*t11*t12*1.6217729888E-3+t5*t10*t11*1.4464461792E-3+t5*t10*t12*3.55591167024E-3+t10*t11*t12*1.290494052896E-3+t2*t3*t4*t7*1.34178132E-5+t2*t4*t5*t6*3.55591167024E-3-t2*t3*t6*t8*3.113295306E-5-t2*t4*t6*t7*9.308888498800001E-5-t2*t3*t6*t9*3.55591167024E-3+t3*t4*t6*t8*1.6217729888E-3-t4*t5*t6*t7*1.5325420128E-3-t2*t3*t6*t12*3.76295165104E-3-t2*t6*t7*t8*1.2E-10-t2*t6*t7*t9*1.4464461792E-3+t3*t4*t7*t10*1.2E-10+t2*t6*t8*t9*3.55591167024E-3-t3*t4*t9*t10*3.55591167024E-3-t3*t5*t6*t12*1.5325420128E-3+t3*t5*t8*t10*3.55591167024E-3-t3*t7*t8*t10*1.24221838048E-4-t6*t7*t8*t9*1.5325420128E-3+t4*t8*t9*t10*3.55591167024E-3+t2*t9*t11*t12*1.5325420128E-3-t2*t3*t4*t5*t6*1.3668617952E-3-t2*t3*t4*t6*t8*2.580988105792E-3-t2*t4*t5*t6*t7*1.3668617952E-3+t2*t3*t4*t7*t9*1.5325420128E-3-t2*t3*t5*t7*t8*1.5325420128E-3-t2*t3*t6*t8*t9*1.3668617952E-3-t2*t3*t7*t8*t9*1.6217729888E-3-t2*t4*t5*t8*t11*1.5325420128E-3-t3*t4*t6*t8*t9*1.5325420128E-3+t2*t3*t6*t9*t12*3.55591167024E-3-t2*t6*t7*t8*t9*5.12981344624E-3+t3*t4*t7*t9*t10*1.3668617952E-3-t3*t5*t7*t8*t10*1.3668617952E-3-t2*t3*t4*t5*t6*t8*3.55591167024E-3+2.3522464759519E-2;
    }

    if (motorID==2)
    {
        M = t2*-8.011900000000001E-7+t6*1.02648391296E-4+t13+1.59816726768E-4;
    }
    
    if (motorID==3)
    {
        M = t6*-1.531408898255E-3+t14+t15+t16+t18+t33+t34+t36+t37+t38-t2*t7*8.13284882E-7-t4*t6*1.6841802318E-4-t6*t8*3.13679318931E-3+1.531408898255E-3;
    }

    if (motorID==4)
    {
        M = t14+t20+t22+t24+t26+t28+t29+t31+t2*t5*1.88147582552E-3+t2*t7*1.423693332896E-3-t4*t6*6.211091902400001E-5-t6*t8*6.0E-11+1.423693332896E-3;
    }
return M;
}

double robotArm::calculateCoriolis(int motorID, double Q1, double Q2, double Q3, double Q4, double DQ1, double DQ2, double DQ3, double DQ4){
double t2 = cos(Q1);
double t3 = cos(Q2);
double t4 = cos(Q3);
double t5 = cos(Q4);
double t6 = sin(Q1);
double t7 = sin(Q2);
double t8 = sin(Q3);
double t9 = sin(Q4);
double t10 = DQ1*DQ1;
double t11 = DQ3*DQ3;
double t12 = DQ4*DQ4;
double t13 = Q3*2.0;
double t14 = t2*t2;
double t15 = t3*t3;
double t16 = t4*t4;
double t17 = t6*t6;
double t18 = t7*t7;
double t19 = t8*t8;
double t20 = sin(t13);
double t21 = t2*t3*t10*5.7253326992E-3;
double t22 = t6*t7*t10*5.7253326992E-3;
double t23 = t4*t6*t7*t9*t10*7.662710064E-4;
double t24 = t5*t6*t7*t8*t10*7.662710064E-4;
double t25 = t2*t4*t6*t9*t10*1.77795583512E-3;
double t26 = t2*t5*t6*t8*t10*1.77795583512E-3;
double t27 = t2*t3*t4*t6*t9*t10*6.834308976E-4;
double t28 = t2*t3*t5*t6*t8*t10*6.834308976E-4;
double t29 = t2*t3*t7*t8*t9*t10*7.662710064E-4;
double t32 = t2*t3*t4*t5*t7*t10*7.662710064E-4;
double t30 = t3*t4*t5*t10*t14*1.77795583512E-3;
double t31 = t3*t8*t9*t10*t14*1.77795583512E-3;
double t33 = t3*t4*t5*t7*t10*t14*6.834308976E-4;
double t34 = t3*t7*t8*t9*t10*t14*6.834308976E-4;
double V;
    if (motorID==1)
    {
        V = t10*-1.438251E-5+t21+t22+t10*sin(Q1*2.0)*5.702252365165001E-4+t3*t10*1.927698669E-6-t6*t10*1.3519085382E-3-t7*t10*8.13284882E-7+t10*t14*2.876502E-5+t3*t24-(DQ2*DQ2)*t7*7.23795776E-5-DQ1*DQ3*3.76295165104E-3-DQ1*DQ2*t2*1.451710392E-4+DQ1*DQ3*t4*6.27358637862E-3-DQ1*DQ3*t8*3.3683604636E-4+DQ1*DQ3*t9*3.55591167024E-3-DQ1*DQ4*t9*3.55591167024E-3+DQ1*DQ2*t14*1.7634261572E-5+DQ1*DQ3*t14*3.76295165104E-3+DQ1*DQ3*t16*7.52590330208E-3+DQ1*DQ3*t20*1.290494052896E-3-t3*t4*t11*1.3519085382E-3-t4*t5*t10*1.77795583512E-3+t3*t8*t10*1.556647653E-5-t4*t6*t11*3.13679318931E-3+t4*t7*t10*4.654444249400001E-5+t5*t6*t10*8.108864944E-4+t3*t8*t11*7.25855196E-5+t3*t9*t10*1.77795583512E-3-t2*t9*t12*1.88147582552E-3-t5*t6*t12*7.232230896E-4+t6*t8*t11*1.6841802318E-4+t7*t8*t10*6.0E-11+t7*t9*t10*7.232230896E-4-t3*t10*t14*3.855397338E-6-t8*t9*t10*1.77795583512E-3+t7*t9*t12*8.108864944E-4+t3*t10*t16*1.88147582552E-3+t6*t10*t15*1.3519085382E-3+t7*t10*t14*1.626569764E-6+t2*t3*t4*t10*7.25855196E-5+t2*t3*t5*t10*7.662710064E-4+t2*t3*t6*t10*3.3683604636E-4+t2*t4*t6*t10*3.3683604636E-4+t3*t4*t5*t10*6.834308976E-4+t2*t3*t8*t10*1.3519085382E-3+t2*t5*t6*t10*5.00235784944E-3+t2*t6*t7*t10*6.27358637862E-3+t3*t4*t8*t10*1.290494052896E-3+t2*t6*t8*t10*6.27358637862E-3-t3*t6*t7*t10*7.25855196E-5-t4*t5*t6*t11*6.834308976E-4+t4*t5*t7*t10*6.834308976E-4-t3*t5*t8*t12*8.108864944E-4+t2*t7*t9*t12*7.232230896E-4+t3*t8*t9*t10*6.834308976E-4-t5*t6*t8*t12*1.88147582552E-3+t2*t6*t10*t15*1.558837051911E-3+t4*t5*t10*t14*3.55591167024E-3-t2*t6*t10*t16*1.290494052896E-3-t6*t8*t9*t11*6.834308976E-4+t7*t8*t9*t10*2.56490672312E-3-t3*t8*t10*t14*3.113295306E-5-t4*t7*t10*t14*9.308888498800001E-5-t3*t9*t10*t14*3.55591167024E-3-t5*t6*t10*t15*8.108864944E-4-t3*t9*t10*t16*1.77795583512E-3+t6*t8*t10*t15*6.7089066E-6-t7*t8*t10*t14*1.2E-10+t6*t9*t10*t15*7.662710064E-4-t7*t9*t10*t14*1.4464461792E-3+t8*t9*t10*t14*3.55591167024E-3-t3*t10*t14*t16*3.76295165104E-3+t6*t10*t15*t16*8.108864944E-4-DQ1*DQ2*t2*t3*1.14506653984E-2-DQ1*DQ2*t2*t4*1.34178132E-5-DQ2*DQ3*t2*t3*8.13284882E-7+DQ2*DQ4*t2*t3*1.423693332896E-3-DQ1*DQ3*t3*t6*1.6217729888E-3+DQ1*DQ3*t4*t5*1.3668617952E-3-DQ1*DQ4*t4*t5*1.3668617952E-3-DQ2*DQ3*t2*t7*1.927698669E-6-DQ2*DQ4*t3*t5*8.108864944E-4-DQ1*DQ2*t6*t7*1.14506653984E-2+DQ1*DQ4*t2*t9*1.6217729888E-3+DQ2*DQ3*t4*t7*7.25855196E-5-DQ3*DQ4*t4*t6*6.0E-11+DQ1*DQ2*t2*t15*2.903420784E-4-DQ1*DQ2*t3*t14*6.27358637862E-3+DQ2*DQ3*t7*t8*1.3519085382E-3-DQ1*DQ2*t4*t14*1.2E-10+DQ1*DQ3*t8*t9*1.3668617952E-3+DQ3*DQ4*t6*t8*6.211091902400001E-5-DQ1*DQ3*t4*t14*6.27358637862E-3-DQ1*DQ4*t8*t9*1.3668617952E-3+DQ1*DQ2*t7*t14*3.3683604636E-4+DQ1*DQ2*t8*t14*1.24221838048E-4+DQ1*DQ3*t8*t14*3.3683604636E-4-DQ1*DQ3*t9*t14*3.55591167024E-3+DQ1*DQ4*t9*t14*5.00235784944E-3-DQ1*DQ3*t9*t16*7.11182334048E-3+DQ1*DQ4*t9*t16*3.55591167024E-3-DQ1*DQ2*t14*t15*3.526852314399999E-5-DQ1*DQ3*t14*t16*7.52590330208E-3+DQ1*DQ2*t2*t3*t6*1.626569764E-6+DQ1*DQ2*t2*t3*t7*5.4076341528E-3+DQ1*DQ3*t2*t3*t6*2.580988105792E-3-DQ2*DQ4*t2*t3*t5*7.232230896E-4-DQ3*DQ4*t2*t3*t4*6.211091902400001E-5+DQ1*DQ3*t3*t4*t6*2.7038170764E-3-DQ1*DQ2*t2*t4*t9*1.5325420128E-3+DQ1*DQ2*t2*t5*t8*1.5325420128E-3+DQ1*DQ2*t2*t6*t7*3.855397338E-6+DQ2*DQ3*t2*t3*t9*7.232230896E-4-DQ2*DQ4*t2*t4*t7*6.0E-11-DQ1*DQ2*t4*t6*t7*1.451710392E-4-DQ3*DQ4*t2*t3*t8*6.0E-11-DQ1*DQ2*t5*t6*t7*1.5325420128E-3-DQ1*DQ3*t3*t6*t8*1.451710392E-4+DQ1*DQ3*t4*t5*t8*7.11182334048E-3+DQ3*DQ4*t2*t5*t7*7.232230896E-4+DQ1*DQ2*t2*t8*t9*1.6217729888E-3+DQ1*DQ3*t3*t6*t9*1.5325420128E-3-DQ1*DQ4*t4*t5*t8*3.55591167024E-3+DQ3*DQ4*t4*t5*t6*6.834308976E-4-DQ1*DQ4*t3*t6*t9*1.5325420128E-3+DQ2*DQ4*t2*t7*t8*6.211091902400001E-5-DQ3*DQ4*t3*t4*t9*8.108864944E-4+DQ1*DQ2*t2*t4*t15*2.68356264E-5-DQ1*DQ2*t6*t7*t8*2.7038170764E-3+DQ1*DQ2*t3*t5*t14*3.76295165104E-3-DQ1*DQ3*t2*t4*t15*1.34178132E-5+DQ1*DQ3*t2*t5*t15*1.5325420128E-3-DQ3*DQ4*t4*t6*t9*1.88147582552E-3+DQ1*DQ2*t3*t7*t14*3.117674103822E-3-DQ1*DQ3*t4*t5*t14*1.3668617952E-3-DQ1*DQ4*t2*t5*t15*1.5325420128E-3+DQ1*DQ4*t4*t5*t14*1.3668617952E-3+DQ1*DQ3*t3*t6*t16*3.2435459776E-3-DQ1*DQ2*t4*t9*t14*1.3668617952E-3+DQ1*DQ2*t5*t8*t14*1.3668617952E-3-DQ1*DQ3*t4*t8*t14*2.580988105792E-3+DQ2*DQ4*t7*t8*t9*8.108864944E-4+DQ3*DQ4*t6*t8*t9*6.834308976E-4-DQ1*DQ4*t2*t9*t15*1.6217729888E-3-DQ1*DQ3*t8*t9*t14*1.3668617952E-3-DQ1*DQ4*t7*t9*t14*3.76295165104E-3+DQ1*DQ2*t4*t14*t15*2.4E-10+DQ1*DQ4*t8*t9*t14*1.3668617952E-3-DQ1*DQ2*t8*t14*t15*2.48443676096E-4+DQ1*DQ3*t9*t14*t16*7.11182334048E-3-DQ1*DQ4*t9*t14*t15*1.4464461792E-3-DQ1*DQ4*t9*t14*t16*3.55591167024E-3+t2*t3*t4*t8*t10*8.108864944E-4+t2*t3*t6*t7*t10*1.7634261572E-5-t2*t4*t5*t7*t10*7.662710064E-4+t2*t3*t4*t9*t11*6.834308976E-4-t2*t3*t5*t8*t11*6.834308976E-4+t2*t4*t6*t8*t10*3.76295165104E-3-t2*t5*t6*t7*t10*3.76295165104E-3+t3*t4*t5*t8*t10*1.77795583512E-3-t3*t4*t6*t7*t10*6.7089066E-6-t2*t4*t6*t9*t10*1.3668617952E-3+t2*t5*t6*t8*t10*1.3668617952E-3-t2*t3*t5*t10*t16*7.662710064E-4-t2*t7*t8*t9*t10*7.662710064E-4-t3*t4*t5*t10*t14*1.3668617952E-3-t2*t5*t6*t10*t15*1.4464461792E-3-t2*t5*t6*t10*t16*3.55591167024E-3-t3*t4*t8*t10*t14*2.580988105792E-3-t4*t5*t7*t10*t14*1.3668617952E-3-t3*t8*t9*t10*t14*1.3668617952E-3-t7*t8*t9*t10*t14*5.12981344624E-3-t2*t6*t10*t15*t16*1.290494052896E-3+t3*t9*t10*t14*t16*3.55591167024E-3-t6*t9*t10*t15*t16*7.662710064E-4-t2*t3*t4*t6*t7*t10*1.2E-10+t2*t3*t4*t6*t9*t10*3.55591167024E-3-t2*t3*t5*t6*t8*t10*3.55591167024E-3-t2*t3*t4*t8*t9*t10*7.662710064E-4+t2*t3*t6*t7*t8*t10*1.24221838048E-4-t2*t4*t6*t8*t9*t10*3.55591167024E-3-t3*t4*t6*t7*t9*t10*7.662710064E-4+t3*t6*t7*t8*t9*t10*8.108864944E-4-t3*t4*t5*t8*t10*t14*3.55591167024E-3+t4*t5*t6*t8*t10*t15*7.662710064E-4-DQ1*DQ2*t2*t3*t4*t6*9.308888498800001E-5-DQ1*DQ3*t2*t3*t4*t6*3.113295306E-5-DQ1*DQ2*t2*t3*t5*t7*3.2435459776E-3+DQ1*DQ3*t2*t3*t5*t6*3.55591167024E-3-DQ1*DQ2*t3*t4*t5*t6*1.5325420128E-3-DQ1*DQ4*t2*t3*t5*t6*3.55591167024E-3-DQ1*DQ2*t2*t3*t6*t8*1.2E-10-DQ1*DQ2*t2*t3*t6*t9*1.4464461792E-3+DQ1*DQ2*t2*t3*t7*t8*2.68356264E-5-DQ1*DQ3*t2*t4*t6*t7*1.2E-10-DQ2*DQ3*t2*t4*t5*t7*6.834308976E-4+DQ1*DQ2*t2*t3*t7*t9*3.0650840256E-3-DQ1*DQ3*t2*t3*t7*t8*1.34178132E-5+DQ1*DQ3*t2*t4*t6*t9*3.55591167024E-3-DQ1*DQ3*t2*t5*t6*t8*3.55591167024E-3-DQ1*DQ4*t2*t5*t6*t7*1.4464461792E-3-DQ3*DQ4*t2*t3*t4*t9*6.834308976E-4+DQ3*DQ4*t2*t3*t5*t8*6.834308976E-4+DQ1*DQ2*t2*t6*t7*t8*3.113295306E-5-DQ1*DQ4*t2*t4*t6*t9*3.55591167024E-3+DQ1*DQ4*t2*t5*t6*t8*3.55591167024E-3+DQ1*DQ2*t2*t6*t7*t9*3.55591167024E-3+DQ1*DQ3*t2*t6*t7*t8*9.308888498800001E-5-DQ1*DQ2*t4*t6*t7*t8*1.6217729888E-3-DQ1*DQ2*t3*t6*t8*t9*1.5325420128E-3+DQ1*DQ3*t3*t4*t5*t14*3.55591167024E-3-DQ1*DQ3*t4*t6*t7*t9*1.5325420128E-3+DQ1*DQ3*t5*t6*t7*t8*1.5325420128E-3+DQ1*DQ2*t2*t3*t7*t16*3.2435459776E-3-DQ1*DQ3*t2*t3*t6*t16*5.161976211584E-3-DQ1*DQ4*t3*t4*t5*t14*3.55591167024E-3+DQ1*DQ4*t4*t6*t7*t9*1.5325420128E-3-DQ1*DQ4*t5*t6*t7*t8*1.5325420128E-3-DQ2*DQ3*t2*t7*t8*t9*6.834308976E-4-DQ1*DQ2*t3*t5*t7*t14*2.8928923584E-3-DQ1*DQ3*t3*t4*t7*t14*1.24221838048E-4+DQ1*DQ2*t2*t4*t9*t15*3.0650840256E-3-DQ1*DQ2*t2*t5*t8*t15*3.0650840256E-3+DQ1*DQ3*t2*t4*t8*t15*3.2435459776E-3+DQ1*DQ2*t2*t6*t7*t16*3.76295165104E-3-DQ1*DQ3*t4*t5*t8*t14*7.11182334048E-3-DQ1*DQ3*t3*t7*t8*t14*1.2E-10+DQ1*DQ4*t4*t5*t8*t14*3.55591167024E-3-DQ1*DQ2*t2*t8*t9*t15*3.2435459776E-3+DQ1*DQ2*t4*t7*t9*t14*3.55591167024E-3+DQ1*DQ2*t5*t6*t7*t16*1.5325420128E-3-DQ1*DQ2*t5*t7*t8*t14*3.55591167024E-3-DQ1*DQ3*t3*t6*t9*t16*3.0650840256E-3+DQ1*DQ3*t3*t8*t9*t14*3.55591167024E-3+DQ1*DQ4*t3*t6*t9*t16*1.5325420128E-3-DQ1*DQ4*t3*t8*t9*t14*3.55591167024E-3-DQ1*DQ3*t2*t5*t15*t16*3.0650840256E-3-DQ1*DQ2*t3*t7*t14*t16*2.580988105792E-3+DQ1*DQ4*t2*t5*t15*t16*1.5325420128E-3+DQ1*DQ2*t4*t9*t14*t15*2.7337235904E-3-DQ1*DQ2*t5*t8*t14*t15*2.7337235904E-3-DQ1*DQ3*t4*t8*t14*t15*2.580988105792E-3-DQ1*DQ2*t2*t3*t4*t5*t6*1.3668617952E-3-DQ1*DQ3*t2*t3*t4*t5*t7*1.5325420128E-3+DQ1*DQ4*t2*t3*t4*t5*t7*1.5325420128E-3+DQ1*DQ2*t2*t4*t5*t6*t7*1.3668617952E-3+DQ1*DQ3*t2*t3*t4*t6*t8*7.52590330208E-3-DQ1*DQ3*t2*t3*t4*t6*t9*1.3668617952E-3+DQ1*DQ3*t2*t3*t5*t6*t8*1.3668617952E-3-DQ1*DQ3*t2*t3*t4*t7*t9*1.6217729888E-3+DQ1*DQ4*t2*t3*t4*t6*t9*1.3668617952E-3-DQ1*DQ4*t2*t3*t5*t6*t8*1.3668617952E-3+DQ1*DQ2*t2*t4*t6*t7*t8*2.580988105792E-3+DQ1*DQ3*t3*t4*t5*t6*t8*3.0650840256E-3-DQ1*DQ4*t2*t3*t5*t7*t8*1.6217729888E-3-DQ1*DQ2*t2*t3*t6*t8*t9*5.12981344624E-3-DQ1*DQ4*t3*t4*t5*t6*t8*1.5325420128E-3-DQ1*DQ3*t2*t4*t6*t7*t9*5.12981344624E-3+DQ1*DQ3*t2*t5*t6*t7*t8*1.3668617952E-3-DQ1*DQ3*t2*t3*t7*t8*t9*1.5325420128E-3+DQ1*DQ4*t2*t4*t6*t7*t9*1.3668617952E-3-DQ1*DQ4*t2*t5*t6*t7*t8*5.12981344624E-3+DQ1*DQ4*t2*t3*t7*t8*t9*1.5325420128E-3+DQ1*DQ2*t2*t6*t7*t8*t9*1.3668617952E-3-DQ1*DQ3*t2*t3*t5*t6*t16*7.11182334048E-3+DQ1*DQ2*t4*t6*t7*t8*t9*1.5325420128E-3-DQ1*DQ3*t3*t4*t5*t7*t14*1.3668617952E-3+DQ1*DQ4*t2*t3*t5*t6*t16*3.55591167024E-3+DQ1*DQ4*t3*t4*t5*t7*t14*1.3668617952E-3-DQ1*DQ2*t2*t3*t7*t9*t16*3.0650840256E-3-DQ1*DQ3*t2*t4*t8*t9*t15*3.0650840256E-3-DQ1*DQ2*t2*t6*t7*t9*t16*3.55591167024E-3+DQ1*DQ4*t2*t4*t8*t9*t15*1.5325420128E-3-DQ1*DQ3*t3*t7*t8*t9*t14*1.3668617952E-3+DQ1*DQ4*t3*t7*t8*t9*t14*1.3668617952E-3-t2*t3*t4*t6*t7*t9*t10*1.3668617952E-3+t2*t3*t5*t6*t7*t8*t10*1.3668617952E-3+DQ1*DQ2*t2*t3*t4*t5*t7*t8*3.0650840256E-3+DQ1*DQ2*t2*t4*t5*t6*t7*t8*3.55591167024E-3-DQ1*DQ3*t2*t3*t4*t6*t8*t9*7.11182334048E-3+DQ1*DQ4*t2*t3*t4*t6*t8*t9*3.55591167024E-3;
    }
    if (motorID==2)
    {
        V = t21+t22+t2*t10*1.75233910896E-4+t6*t10*8.011900000000001E-7-t10*t14*8.817130785999998E-6+t2*t4*t10*6.7089066E-6-t2*t10*t15*1.451710392E-4+t3*t10*t14*3.13679318931E-3+t4*t10*t14*6.0E-11-t7*t10*t14*1.6841802318E-4-t8*t10*t14*6.211091902400001E-5+t10*t14*t15*1.7634261572E-5-t2*t3*t6*t10*8.13284882E-7-t2*t3*t7*t10*2.7038170764E-3+t2*t4*t9*t10*7.662710064E-4-t2*t5*t8*t10*7.662710064E-4-t2*t6*t7*t10*1.927698669E-6+t4*t6*t7*t10*7.25855196E-5+t5*t6*t7*t10*7.662710064E-4-t2*t8*t9*t10*8.108864944E-4-t2*t4*t10*t15*1.34178132E-5+t6*t7*t8*t10*1.3519085382E-3-t3*t5*t10*t14*1.88147582552E-3-t3*t7*t10*t14*1.558837051911E-3+t4*t9*t10*t14*6.834308976E-4-t5*t8*t10*t14*6.834308976E-4-t4*t10*t14*t15*1.2E-10+t8*t10*t14*t15*1.24221838048E-4+DQ1*DQ3*t2*t3*8.13284882E-7-DQ1*DQ4*t2*t3*1.423693332896E-3+DQ1*DQ3*t2*t7*1.927698669E-6+DQ1*DQ4*t3*t5*8.108864944E-4-DQ1*DQ3*t4*t7*7.25855196E-5-DQ1*DQ3*t7*t8*1.3519085382E-3+DQ1*DQ4*t2*t3*t5*7.232230896E-4-DQ1*DQ3*t2*t3*t9*7.232230896E-4+DQ1*DQ4*t2*t4*t7*6.0E-11-DQ1*DQ4*t2*t7*t8*6.211091902400001E-5-DQ1*DQ4*t7*t8*t9*8.108864944E-4+t2*t3*t4*t6*t10*4.654444249400001E-5+t2*t3*t5*t7*t10*1.6217729888E-3+t3*t4*t5*t6*t10*7.662710064E-4+t2*t3*t6*t8*t10*6.0E-11+t2*t3*t6*t9*t10*7.232230896E-4-t2*t3*t7*t8*t10*1.34178132E-5-t2*t3*t7*t9*t10*1.5325420128E-3-t2*t6*t7*t8*t10*1.556647653E-5-t2*t6*t7*t9*t10*1.77795583512E-3+t4*t6*t7*t8*t10*8.108864944E-4+t3*t6*t8*t9*t10*7.662710064E-4-t2*t3*t7*t10*t16*1.6217729888E-3+t3*t5*t7*t10*t14*1.4464461792E-3-t2*t4*t9*t10*t15*1.5325420128E-3+t2*t5*t8*t10*t15*1.5325420128E-3-t2*t6*t7*t10*t16*1.88147582552E-3+t2*t8*t9*t10*t15*1.6217729888E-3-t4*t7*t9*t10*t14*1.77795583512E-3-t5*t6*t7*t10*t16*7.662710064E-4+t5*t7*t8*t10*t14*1.77795583512E-3+t3*t7*t10*t14*t16*1.290494052896E-3-t4*t9*t10*t14*t15*1.3668617952E-3+t5*t8*t10*t14*t15*1.3668617952E-3+t2*t3*t4*t5*t6*t10*6.834308976E-4-t2*t4*t5*t6*t7*t10*6.834308976E-4-t2*t4*t6*t7*t8*t10*1.290494052896E-3+t2*t3*t6*t8*t9*t10*2.56490672312E-3-t2*t6*t7*t8*t9*t10*6.834308976E-4-t4*t6*t7*t8*t9*t10*7.662710064E-4+t2*t3*t7*t9*t10*t16*1.5325420128E-3+t2*t6*t7*t9*t10*t16*1.77795583512E-3+DQ1*DQ3*t2*t4*t5*t7*6.834308976E-4+DQ1*DQ3*t2*t7*t8*t9*6.834308976E-4-t2*t3*t4*t5*t7*t8*t10*1.5325420128E-3-t2*t4*t5*t6*t7*t8*t10*1.77795583512E-3;
    }
    if (motorID==3)
    {
        V = t10*1.88147582552E-3+t23-t24-t25+t26+t27-t28+t29-t30-t31+t32+t33+t34-t2*t10*1.531408898255E-3-t4*t10*3.13679318931E-3+t5*t12*7.232230896E-4+t8*t10*1.6841802318E-4-t9*t10*1.77795583512E-3-t10*t14*1.88147582552E-3-t10*t16*3.76295165104E-3-t10*t20*6.45247026448E-4-t2*t4*t10*1.6841802318E-4+t3*t6*t10*8.08958795731E-4-t4*t5*t10*6.834308976E-4-t2*t8*t10*3.13679318931E-3+t6*t7*t10*8.13284882E-7-t8*t9*t10*6.834308976E-4+t4*t10*t14*3.13679318931E-3-t8*t10*t14*1.6841802318E-4+t9*t10*t14*1.77795583512E-3+t9*t10*t16*3.55591167024E-3+t10*t14*t16*3.76295165104E-3-t2*t3*t6*t10*1.290494052896E-3-t3*t4*t6*t10*1.3519085382E-3+t2*t4*t9*t10*6.834308976E-4-t2*t5*t8*t10*6.834308976E-4+t3*t6*t8*t10*7.25855196E-5-t4*t5*t8*t10*3.55591167024E-3-t3*t6*t9*t10*7.662710064E-4+t2*t4*t10*t15*6.7089066E-6-t2*t5*t10*t15*7.662710064E-4-t6*t7*t9*t10*7.232230896E-4+t4*t5*t10*t14*6.834308976E-4-t3*t6*t10*t16*1.6217729888E-3+t4*t8*t10*t14*1.290494052896E-3+t8*t9*t10*t14*6.834308976E-4-t9*t10*t14*t16*3.55591167024E-3-DQ1*DQ2*t2*t3*8.13284882E-7-DQ1*DQ2*t2*t7*1.927698669E-6+DQ1*DQ2*t4*t7*7.25855196E-5+DQ1*DQ4*t4*t6*6.0E-11+DQ1*DQ2*t7*t8*1.3519085382E-3-DQ1*DQ4*t6*t8*6.211091902400001E-5+DQ1*DQ4*t2*t3*t4*6.211091902400001E-5+DQ1*DQ2*t2*t3*t9*7.232230896E-4+DQ1*DQ4*t2*t3*t8*6.0E-11+DQ1*DQ4*t2*t5*t7*7.232230896E-4+DQ1*DQ4*t4*t5*t6*6.834308976E-4+DQ1*DQ4*t3*t4*t9*8.108864944E-4+DQ1*DQ4*t4*t6*t9*1.88147582552E-3+DQ1*DQ4*t6*t8*t9*6.834308976E-4+t2*t3*t4*t6*t10*1.556647653E-5-t2*t3*t5*t6*t10*1.77795583512E-3-t3*t4*t5*t6*t10*6.834308976E-4+t2*t4*t6*t7*t10*6.0E-11+t2*t3*t7*t8*t10*6.7089066E-6-t2*t6*t7*t8*t10*4.654444249400001E-5-t3*t6*t8*t9*t10*6.834308976E-4+t2*t3*t6*t10*t16*2.580988105792E-3+t3*t4*t7*t10*t14*6.211091902400001E-5-t2*t4*t8*t10*t15*1.6217729888E-3+t4*t5*t8*t10*t14*3.55591167024E-3+t3*t7*t8*t10*t14*6.0E-11+t3*t6*t9*t10*t16*1.5325420128E-3+t2*t5*t10*t15*t16*1.5325420128E-3+t4*t8*t10*t14*t15*1.290494052896E-3-t2*t3*t4*t6*t8*t10*3.76295165104E-3+t2*t3*t4*t7*t9*t10*8.108864944E-4-t3*t4*t5*t6*t8*t10*1.5325420128E-3+t2*t4*t6*t7*t9*t10*2.56490672312E-3-t2*t5*t6*t7*t8*t10*6.834308976E-4+t2*t3*t5*t6*t10*t16*3.55591167024E-3+t2*t4*t8*t9*t10*t15*1.5325420128E-3-DQ1*DQ2*t2*t4*t5*t7*6.834308976E-4-DQ1*DQ4*t2*t3*t4*t9*6.834308976E-4+DQ1*DQ4*t2*t3*t5*t8*6.834308976E-4-DQ1*DQ2*t2*t7*t8*t9*6.834308976E-4+t2*t3*t4*t6*t8*t9*t10*3.55591167024E-3;  
    }
    if (motorID==4)
    {
        V = -t23+t24+t25-t26-t27+t28-t29+t30+t31-t32-t33-t34-t2*t4*t10*6.211091902400001E-5-t2*t8*t10*6.0E-11-t2*t9*t10*7.232230896E-4-t5*t6*t10*1.88147582552E-3-t6*t7*t10*1.423693332896E-3-t3*t4*t6*t10*6.0E-11+t3*t6*t8*t10*6.211091902400001E-5+t5*t6*t7*t10*7.232230896E-4-t2*t8*t9*t10*1.88147582552E-3+t4*t5*t10*t17*6.834308976E-4-t2*t9*t10*t18*8.108864944E-4+t7*t9*t10*t14*1.88147582552E-3+t8*t9*t10*t17*6.834308976E-4-t9*t10*t14*t18*7.232230896E-4+t9*t10*t17*t19*1.77795583512E-3+DQ1*DQ2*t2*t3*1.423693332896E-3-DQ1*DQ2*t3*t5*8.108864944E-4-DQ1*DQ3*t4*t6*6.0E-11+DQ1*DQ3*t6*t8*6.211091902400001E-5-DQ1*DQ2*t2*t3*t5*7.232230896E-4-DQ1*DQ3*t2*t3*t4*6.211091902400001E-5-DQ1*DQ2*t2*t4*t7*6.0E-11-DQ1*DQ3*t2*t3*t8*6.0E-11-DQ1*DQ3*t2*t5*t7*7.232230896E-4-DQ1*DQ3*t4*t5*t6*6.834308976E-4+DQ1*DQ2*t2*t7*t8*6.211091902400001E-5-DQ1*DQ3*t3*t4*t9*8.108864944E-4-DQ1*DQ3*t4*t6*t9*1.88147582552E-3+DQ1*DQ2*t7*t8*t9*8.108864944E-4-DQ1*DQ3*t6*t8*t9*6.834308976E-4+t2*t5*t6*t7*t10*7.232230896E-4+t4*t5*t8*t10*t17*1.77795583512E-3+t3*t6*t9*t10*t19*7.662710064E-4+t2*t5*t10*t15*t19*7.662710064E-4+t2*t3*t5*t7*t8*t10*8.108864944E-4+t3*t4*t5*t6*t8*t10*7.662710064E-4-t2*t4*t6*t7*t9*t10*6.834308976E-4+t2*t5*t6*t7*t8*t10*2.56490672312E-3+t2*t3*t5*t6*t10*t19*1.77795583512E-3-t2*t4*t8*t9*t10*t15*7.662710064E-4+DQ1*DQ3*t2*t3*t4*t9*6.834308976E-4-DQ1*DQ3*t2*t3*t5*t8*6.834308976E-4-t2*t3*t4*t6*t8*t9*t10*1.77795583512E-3;
    } 
return V;
}
    

double robotArm::calculateGravity(int motorID, double Q1, double Q2, double Q3, double Q4){
double t2 = cos(Q1);
double t3 = cos(Q2);
double t4 = cos(Q3);
double t5 = cos(Q4);
double t6 = sin(Q1);
double t7 = sin(Q2);
double t8 = sin(Q3);
double t9 = sin(Q4);
double G;
   if (motorID==1)
    {
        G = t2*-1.27196840191+t2*t4*9.632294628E-3+t2*t8*7.7715683274E-2+t6*t7*8.71467553E-3-t2*t4*t5*1.07606829392E-1+t3*t4*t6*7.7715683274E-2-t3*t6*t8*9.632294628E-3-t6*t7*t9*1.07606829392E-1+t3*t5*t6*t8*1.07606829392E-1;
    }
    if (motorID==2)
    {
        G = t2*t3*-8.71467553E-3+t2*t4*t7*7.7715683274E-2+t2*t3*t9*1.07606829392E-1-t2*t7*t8*9.632294628E-3+t2*t5*t7*t8*1.07606829392E-1;
    }
    if (motorID==3)
    {
        G = t4*t6*7.7715683274E-2-t6*t8*9.632294628E-3+t2*t3*t4*9.632294628E-3+t2*t3*t8*7.7715683274E-2+t5*t6*t8*1.07606829392E-1-t2*t3*t4*t5*1.07606829392E-1;
    }
    if (motorID==4)
    {
        G = t2*t5*t7*1.07606829392E-1+t4*t6*t9*1.07606829392E-1+t2*t3*t8*t9*1.07606829392E-1;
    }
    return G;
}

double robotArm::ControlSystem(double ref_DQ1, double ref_DQ2, double ref_DQ3, double ref_DQ4){
    double Kp[4] = {100, 10, 10, 10};
    double Q1 = getPositionRad(1);
    double Q2 = getPositionRad(2);
    double Q3 = getPositionRad(3);
    double Q4 = getPositionRad(4);
    double DQ1 = getVelocity(1);
    double DQ2 = getVelocity(2);
    double DQ3 = getVelocity(3);
    double DQ4 = getVelocity(4);
    double error[4] = {ref_DQ1-DQ1, ref_DQ2-DQ2, ref_DQ3-DQ3, ref_DQ4-DQ4};
    double torque;

    for (size_t i = 0; i < 1; i++)
    {
        torque = ((error[i] * Kp[i] * calculateMass(1, Q1, Q2, Q3, Q4)) + (calculateCoriolis(1, Q1, Q2, Q3, Q4, DQ1, DQ2, DQ3, DQ4) + calculateGravity(1, Q1, Q2, Q3, Q4)));
        //Serial.print("Torque: ");
        //Serial.println(torque);
        //setTorque(i+1,torque); 
    }
    return torque;
}


bool robotArm::dataGatherer()
{
    digitalWrite(LED_BUILTIN, LOW);
    bool debug = false;
    byte header[5]{};
    byte lenght{};
    Serial.readBytesUntil(0x00, header, 5);
    if (header[0] == 0xFF && header[1] == 0xFF && header[2] == 0xFD && header[4] == 0x00)
    {
        int starttime = micros();

        lenght = header[3];

        while (Serial.available() < lenght)
        {
            // Serial.println("WAITING");
        }

        byte ReadData[lenght]{};

        Serial.readBytes(ReadData, lenght);
        int ID = ReadData[0];
        byte Instruction = ReadData[1];

        byte ReadCRC[2]{};
        ReadCRC[0] = ReadData[lenght - 2];
        ReadCRC[1] = ReadData[lenght - 1];
        unsigned short RecievedCRC = (ReadCRC[0] << 8) | ReadCRC[1];

        unsigned int datasize = lenght - 4;
        byte Param[datasize]{};

        for (size_t i = 0; i < datasize; i++)
        {
            Param[i] = ReadData[2 + i];
        }

        byte CRCArray[5 + lenght - 2];

        for (size_t i = 0; i < sizeof(header); i++)
        {
            CRCArray[i] = header[i];
        }
        for (size_t i = 0; i < sizeof(ReadData) - 2; i++)
        {
            CRCArray[i + 5] = ReadData[i];
        }

        unsigned short CalcCRC = robotArm::CalculateCRC(0, CRCArray, sizeof(CRCArray));
        
        if(debug == true)
        {
        int Endtime = micros();
        int Processtime = Endtime - starttime;

        //Serial3.write(ReadCRC[0]); Serial3.write(ReadCRC[1]);
        //Serial3.write(RecievedCRC);

        //Serial3.write((int)ID);
        //Serial3.write((int)lenght);
        //Serial3.write((int)Instruction)

        //Serial3.write(CalcCRC);

        for (size_t i = 0; i < sizeof(CRCArray); i++)
        {
            Serial3.write(CRCArray[i]);
        }

        Serial3.write(ReadCRC[0]); 
        Serial3.write(ReadCRC[1]);
        byte CRC2 = CalcCRC & 0xff;
        byte CRC1 = (CalcCRC >> 8);
        
        Serial3.write(CRC1); 
        Serial3.write(CRC2);
        Serial3.write(Processtime);
        }
        
        if (CalcCRC == RecievedCRC)
        {
            //digitalWrite(LED_BUILTIN, HIGH);
            for (size_t i = 0; i < sizeof(ReadData) - 2; i++)
            {
                robotArm::Parameters[i] = Param[i];
            }
            robotArm::MotorID = (int)ID;
            robotArm::Instruction = (char)Instruction;
            Serial.write(0x06);
            return true;
        }
        else
        {
            //Serial.println("\nDATA WAS CORRUPTED");
            return false;
        }
    
        
    }
    return false;
}


unsigned short robotArm::CalculateCRC(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202};

    for (j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}

void robotArm::startMotors()
{
    for (size_t i = 1; i < 7; i++)
    {
        dxl->torqueOff(i);
        dxl->setOperatingMode(i, OP_PWM);

        dxl->torqueOn(i);
    }
}