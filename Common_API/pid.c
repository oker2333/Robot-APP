#include "pid.h"
#include <stdio.h>
#include <math.h>
#include <limits.h>

#define MAXOUT 1000                  //输出最大值
#define MINOUT -1000                 //输出最小值

PID sPID_l;
static pPID sptr_l;

PID sPID_r;
static pPID sptr_r;

void IncPIDInit(void)
{
    sptr_l                = &sPID_l;
    sptr_l->SetPoint      = 0;      	//设定值
    sptr_l->BitMove       = 0u;       //返回结果比例

    sptr_l->LastError     = 0u;       //前2次误差值
    sptr_l->PrevError     = 0u;       //前1次误差值

    sptr_l->Proportion    = 0.110f;   //比例
    sptr_l->Integral      = 0.055f;   //积分
    sptr_l->Derivative    = 0.0f;     //微分

    sptr_l->iError        = 0;        //当前误差
    sptr_l->iIncpid       = 0;        //增量误差

    sptr_l->Uk            = 0;        //输出返回值
	
		/****************************************/
    sptr_r                = &sPID_r;
    sptr_r->SetPoint      = 0;      	//设定值
    sptr_r->BitMove       = 0u;       //返回结果比例

    sptr_r->LastError     = 0u;       //前2次误差值
    sptr_r->PrevError     = 0u;       //前1次误差值

    sptr_r->Proportion    = 0.110f;   //比例
    sptr_r->Integral      = 0.055f;   //积分
    sptr_r->Derivative    = 0.0f;     //微分

    sptr_r->iError        = 0;        //当前误差
    sptr_r->iIncpid       = 0;        //增量误差

    sptr_r->Uk            = 0;        //输出返回值	
}

int IncPIDCalc_l(int SetPoint,int measurement)
{
	  sptr_l->SetPoint = SetPoint;
	  
    //当前误差
    sptr_l->iError = sptr_l->SetPoint - measurement;
	  if((sptr_l->iError <= 8) && (sptr_l->iError >= -8))
		{
			 return INT_MAX;
		}

    //增量误差
    sptr_l->iIncpid = sptr_l->Proportion * sptr_l->iError - sptr_l->Integral * sptr_l->LastError
                    + sptr_l->Derivative * sptr_l->PrevError;
    //存储误差，用于下次计算
    sptr_l->PrevError = sptr_l->iError;
    sptr_l->LastError = sptr_l->LastError;

    sptr_l->Uk += sptr_l->iIncpid;

    //输出值限幅
    if (sptr_l->Uk >> sptr_l->BitMove >= MAXOUT)
    {
        sptr_l->Uk = MAXOUT;
    }
    else if(sptr_l->Uk >> sptr_l->BitMove <= MINOUT)
    {
        sptr_l->Uk = MINOUT;
    }
    else 
		{
				sptr_l->Uk = sptr_l->Uk >> sptr_l->BitMove;
		}

    return sptr_l->Uk;
}

int IncPIDCalc_r(int SetPoint,int measurement)
{
	  sptr_r->SetPoint = SetPoint;
	  
    //当前误差
    sptr_r->iError = sptr_r->SetPoint - measurement;
	  if((sptr_r->iError <= 8) && (sptr_r->iError >= -8))
		{
			 return INT_MAX;
		}
	  printf("velocity_r error = %d\n",sptr_r->iError);
    //增量误差
    sptr_r->iIncpid = sptr_r->Proportion * sptr_r->iError - sptr_r->Integral * sptr_r->LastError
                    + sptr_r->Derivative * sptr_r->PrevError;
    //存储误差，用于下次计算
    sptr_r->PrevError = sptr_r->iError;
    sptr_r->LastError = sptr_r->LastError;

    sptr_r->Uk += sptr_r->iIncpid;

    //输出值限幅
    if (sptr_r->Uk >> sptr_r->BitMove >= MAXOUT)
    {
        sptr_r->Uk = MAXOUT;
    }
    else if(sptr_r->Uk >> sptr_r->BitMove <= MINOUT)
    {
        sptr_r->Uk = MINOUT;
    }
    else 
		{
				sptr_r->Uk = sptr_r->Uk >> sptr_r->BitMove;
		}

    return sptr_r->Uk;
}
