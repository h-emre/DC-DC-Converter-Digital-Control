#include "PID.h"
void PID_Init(PID_TypeDef *uPID) {
    uPID->OutputSum = *uPID->MyOutput;
    uPID->LastInput = *uPID->MyInput;
    if (uPID->OutputSum > uPID->OutMax) {
        uPID->OutputSum = uPID->OutMax;
    } else if (uPID->OutputSum < uPID->OutMin) {
        uPID->OutputSum = uPID->OutMin;
    }
}
void PID(PID_TypeDef *uPID, double *Input, double *Output,
		double *Setpoint, double Kp, double Ki, double Kd,
		PIDPON_TypeDef POn, PIDCD_TypeDef ControllerDirection) {
    uPID->MyOutput = Output;
    uPID->MyInput = Input;
    uPID->MySetpoint = Setpoint;
    uPID->InAuto = _PID_MODE_AUTOMATIC;
    PID_SetOutputLimits(uPID, 0, _PID_8BIT_PWM_MAX);
    uPID->SampleTime = _PID_SAMPLE_TIME_MS_DEF;
    PID_SetControllerDirection(uPID, ControllerDirection);
    PID_SetTunings2(uPID, Kp, Ki, Kd, POn);
    uPID->LastTime = HAL_GetTick() - uPID->SampleTime;
}
uint8_t PID_Compute(PID_TypeDef *uPID) {
    uint32_t now = HAL_GetTick();
    uint32_t timeChange = (now - uPID->LastTime);
    if (timeChange >= uPID->SampleTime) {
        double input = *uPID->MyInput;
        double error = *uPID->MySetpoint - input;
        double dInput = (input - uPID->LastInput);
        double output;
        uPID->OutputSum += (uPID->Ki * error);
        if (!uPID->POnE) {
            uPID->OutputSum -= uPID->Kp * dInput;
        }
        if (uPID->OutputSum > uPID->OutMax) {
            uPID->OutputSum = uPID->OutMax;
        } else if (uPID->OutputSum < uPID->OutMin) {
            uPID->OutputSum = uPID->OutMin;
        }
        if (uPID->POnE) {
            output = uPID->Kp * error;
        } else {
            output = 0;
        }
        output += uPID->OutputSum - uPID->Kd * dInput;
        if (output > uPID->OutMax) {
            output = uPID->OutMax;
        } else if (output < uPID->OutMin) {
            output = uPID->OutMin;
        }
        *uPID->MyOutput = output;
        uPID->LastInput = input;
        uPID->LastTime = now;
        return 1;
    } else {
        return 0;
    }
}

void PID_SetMode(PID_TypeDef *uPID, PIDMode_TypeDef Mode) {
    uint8_t newAuto = (Mode == _PID_MODE_AUTOMATIC);

    if (newAuto && !uPID->InAuto) {
        PID_Init(uPID);
    }

    uPID->InAuto = newAuto;
}
void PID_SetOutputLimits(PID_TypeDef *uPID, double Min, double Max) {
    if (Min >= Max) return;

    uPID->OutMin = Min;
    uPID->OutMax = Max;

    if (uPID->InAuto) {
        if (*uPID->MyOutput > uPID->OutMax) {
            *uPID->MyOutput = uPID->OutMax;
        } else if (*uPID->MyOutput < uPID->OutMin) {
            *uPID->MyOutput = uPID->OutMin;
        }

        if (uPID->OutputSum > uPID->OutMax) {
            uPID->OutputSum = uPID->OutMax;
        } else if (uPID->OutputSum < uPID->OutMin) {
            uPID->OutputSum = uPID->OutMin;
        }
    }
}
void PID_SetTunings(PID_TypeDef *uPID, double Kp,
		double Ki, double Kd) {
    PID_SetTunings2(uPID, Kp, Ki, Kd, uPID->POn);
}
void PID_SetTunings2(PID_TypeDef *uPID, double Kp,
		double Ki, double Kd, PIDPON_TypeDef POn) {
    if (Kp < 0 || Ki < 0 || Kd < 0) return;

    uPID->POn = POn;
    uPID->POnE = (POn == _PID_P_ON_E);

    double SampleTimeInSec = ((double)uPID->SampleTime) / 1000;

    uPID->Kp = Kp;
    uPID->Ki = Ki * SampleTimeInSec;
    uPID->Kd = Kd / SampleTimeInSec;

    if (uPID->ControllerDirection == _PID_CD_REVERSE) {
        uPID->Kp = -uPID->Kp;
        uPID->Ki = -uPID->Ki;
        uPID->Kd = -uPID->Kd;
    }
}
void PID_SetControllerDirection(PID_TypeDef *uPID, PIDCD_TypeDef Direction) {
    if (uPID->InAuto && Direction != uPID->ControllerDirection) {
        uPID->Kp = -uPID->Kp;
        uPID->Ki = -uPID->Ki;
        uPID->Kd = -uPID->Kd;
    }

    uPID->ControllerDirection = Direction;
}
void PID_SetSampleTime(PID_TypeDef *uPID, int32_t NewSampleTime) {
    if (NewSampleTime > 0) {
        double ratio = (double)NewSampleTime / (double)uPID->SampleTime;

        uPID->Ki *= ratio;
        uPID->Kd /= ratio;
        uPID->SampleTime = (uint32_t)NewSampleTime;
    }
}
double PID_GetKp(PID_TypeDef *uPID) {
    return uPID->DispKp;
}

double PID_GetKi(PID_TypeDef *uPID) {
    return uPID->DispKi;
}
double PID_GetKd(PID_TypeDef *uPID) {
    return uPID->DispKd;
}
