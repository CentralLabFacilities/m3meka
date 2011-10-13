#ifndef DEF_WARNING_H	
#define DEF_WARNING_H	

//Compilation warnings:
#ifdef MAX2_BDC_0_3_A2R2
#warning "Project: MAX2_BDC_0_3_A2R2"
#endif

//MAX2 version reminder
#ifndef USE_MAX2_0_2 
#ifndef USE_MAX2_0_3
#warning "Please specify your MAX2 version!"
#endif
#endif
#ifdef USE_MAX2_0_2
#warning "Compiled for MAX2 v0.2."
#endif
#ifdef USE_MAX2_0_3
#warning "Compiled for MAX2 v0.3."
#endif

#endif
