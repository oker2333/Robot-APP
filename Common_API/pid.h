#ifndef __PID_H
#define __PID_H

typedef struct PID
{
    int SetPoint;

    unsigned char BitMove;

    float Proportion;
    float Integral;
    float Derivative;

    int iError;
    int iIncpid;

    int LastError;
    int PrevError;

    int Uk;
} PID, *pPID;

void IncPIDInit(void);
int IncPIDCalc_l(int SetPoint,int measurement);
int IncPIDCalc_r(int SetPoint,int measurement);

#endif
