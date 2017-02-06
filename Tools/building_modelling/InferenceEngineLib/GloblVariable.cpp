/*--------------------------------------------------
Initial creation:
Author : Biao Xiong
Date   : April 11, 2012
Description: 
Revise:
----------------------------------------------------------*/
#include "InferenceEngine.h"

//double ALPPHA_DIAMETER = 1.2;
//double MIN_RIDGE_LEN;
//int MIN_SEG_PNT_NUM=30;
//int MIN_RIDGE_PNT_NUM=7;

BldRecPar gBldRecPar;// = {1.2, 0.5, 30, 7, true};

IEDll_API void UpdataIEGlobelVari(BldRecPar& updataPar)
{
	gBldRecPar = updataPar;
}

IEDll_API BldRecPar& GetIEGlobelVari()
{
	return gBldRecPar;
}