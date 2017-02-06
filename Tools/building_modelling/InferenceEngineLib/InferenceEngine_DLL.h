#ifndef _Inference_Engine_DLL_H_
#define _Inference_Engine_DLL_H_


#ifdef _WINDOWS
	#ifdef InfeEngine_EXPORTS
	#define IEDll_API __declspec(dllexport)
	#else
	#define IEDll_API __declspec(dllimport)
	#endif
#else
	#define IEDll_API
#endif


#endif
