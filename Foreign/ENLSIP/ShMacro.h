/*--------------------------------------------------------------------
*   Project   : Automated reconstruction of industrial installations
*
*   File made : April 2005
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : A macro for debugging
*
*--------------------------------------------------------------------*/
#ifndef __SH_MACRO__H___
#define __SH_MACRO__H___
#include <iostream>
#define SH std::cerr<<__FILE__<<": "<<__LINE__<<"\n"<<flush;
#define SHV(a) std::cerr<<##a<<": "<<a<<" in "<<__FILE__<<": "<<__LINE__<<"\n"<<flush;
#define SHM(a) std::cerr<<a<<"     in "<<__FILE__<<": "<<__LINE__<<"\n"<<flush;
#endif//__SH_MACRO__H___
