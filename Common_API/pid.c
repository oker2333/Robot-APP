#include "pid.h"
#include <stdio.h>
#include <math.h>
#include <limits.h>

#define MAXOUT 1000                  //输出最大值
#define MINOUT -1000                 //输出最小值

PID sPID;
static pPID sptr;

void IncPIDInit(void)
{
    sptr                = &sPID;
    sptr->SetPoint      = 0;      	//设定值
    sptr->BitMove       = 0u;       //返回结果比例

    sptr->LastError     = 0u;       //前2次误差值
    sptr->PrevError     = 0u;       //前1次误差值

    sptr->Proportion    = 0.175f;   //比例
    sptr->Integral      = 0.05f;   //积分
    sptr->Derivative    = 0.0f;     //微分

    sptr->iError        = 0;        //当前误差
    sptr->iIncpid       = 0;        //增量误差

    sptr->Uk            = 0;        //输出返回值
}

int IncPIDCalc(int SetPoint,int measurement)
{
	  sptr->SetPoint = SetPoint;
	  
    //当前误差
    sptr->iError = sptr->SetPoint - measurement;
	  if((sptr->iError <= 8) && (sptr->iError >= -8))
		{
			 return INT_MAX;
		}
	  printf("Eroor = %d\n",sptr->iError);
    //增量误差
    sptr->iIncpid = sptr->Proportion * sptr->iError - sptr->Integral * sptr->LastError
                    + sptr->Derivative * sptr->PrevError;
    //存储误差，用于下次计算
    sptr->PrevError = sptr->iError;
    sptr->LastError = sptr->LastError;

    sptr->Uk += sptr->iIncpid;

    //输出值限幅
    if (sptr->Uk >> sptr->BitMove >= MAXOUT)
    {
        sptr->Uk = MAXOUT;
    }
    else if(sptr->Uk >> sptr->BitMove <= MINOUT)
    {
        sptr->Uk = MINOUT;
    }
    else 
		{
				sptr->Uk = sptr->Uk >> sptr->BitMove;
		}

    return sptr->Uk;
}
