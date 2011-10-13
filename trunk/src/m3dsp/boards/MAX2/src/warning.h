#ifndef DEF_WARNING_H	
#define DEF_WARNING_H	

//Compilation warnings:
#ifdef MAX2_BDC_0_3_A2R2
	#warning "Project: MAX2_BDC_0_3_A2R2"
#else 
	#ifdef MAX2_BLDC_0_3_A2R2
		#warning "Project: MAX2_BLDC_0_3_A2R2"
	#endif
#endif

//MAX2 version reminder
#ifndef USE_MAX2_0_2 
#ifndef USE_MAX2_0_3
#warning "Please specify your MAX2 version!"
#endif
#endif

#endif
