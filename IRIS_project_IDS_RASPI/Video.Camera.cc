#include "Video.Camera.h"
#include "Video.IDSCamera.h"
#include <cmath> //log
//#include "Log/Log.h"

namespace Video {

	Camera::~Camera(){}

	//AEC
	void Camera::AEC_configure(double expo_init_ms, double expo_consigne, double expo_max_ms, double expo_min_ms,   double expo_asservissementSpeed ){

		// expo_asservissementSpeed : Entre 0 et 100
	    AEC_accumulator_ = 0;
	    AEC_Tint_init_ms_ = expo_init_ms;
	    AEC_consigne_ = expo_consigne;
	    AEC_expo_asservissementK_ = expo_asservissementSpeed /100.;
	    AEC_expo_max_ms_ = expo_max_ms;
	    AEC_expo_min_ms_ = expo_min_ms;


	    //compute threshold
        AEC_threshold_up_   = std::log2(AEC_expo_max_ms_ / AEC_Tint_init_ms_ ) / AEC_expo_asservissementK_;
	    AEC_threshold_down_ = std::log2(AEC_expo_min_ms_ / AEC_Tint_init_ms_ ) / AEC_expo_asservissementK_;


	}
	void Camera::AEC_compute_error(double moy_actuelle){

		double increment =  std::log2(AEC_consigne_ / moy_actuelle);

		//Gestion saturatino de pas
    	increment = increment>AEC_expo_step_max_  ?  AEC_expo_step_max_:increment;

    	//Accumulation
    	AEC_accumulator_ += increment;

		//GEstion saturation de l'accumulateur => Borne le temps d'exposition
        AEC_accumulator_ = AEC_accumulator_ > AEC_threshold_up_   ?  AEC_threshold_up_  :AEC_accumulator_;
        AEC_accumulator_ = AEC_accumulator_ < AEC_threshold_down_ ?  AEC_threshold_down_:AEC_accumulator_;
	}

	double Camera::AEC_compute_expo(){
		// LOG (expo)
		double new_expo_log = std::log2(AEC_Tint_init_ms_) + AEC_expo_asservissementK_ * AEC_accumulator_;
		double new_expo_ms = std::pow(2, new_expo_log);

		return new_expo_ms;
	}

	double Camera::AEC_update(double moy_actuelle){

		if(AEC_counter_ % AEC_decimation_ == 0){
			if(  (moy_actuelle < (1.-AEC_tolerance_)*AEC_consigne_ ) || (moy_actuelle > (1.+AEC_tolerance_)*AEC_consigne_ )  ){
		    	AEC_compute_error(moy_actuelle);
		        AEC_current_expo_ = AEC_compute_expo();
		        //LOG_DEBUG("************************\nAEC \t consigne =  "<< AEC_consigne_  << "\tvu\t"<< moy_actuelle << "\terreur\t" << AEC_consigne_ / moy_actuelle  <<"\tnew tint\t"<< AEC_current_expo_ << "\tAccu\t" << AEC_accumulator_ << "\tAEC_expo_min_ms_\t" << AEC_expo_min_ms_ << "\tAEC_expo_max_ms_\t" << AEC_expo_max_ms_ << "\tTint_init_ms\t" << AEC_Tint_init_ms_ << "\n**********************************");

		        if(CameraStatus::OK != SetIntegrationTime(AEC_current_expo_) )
		        	return -1;
		    }//AEC_tolerance_
		}//AEC_decimation_
		AEC_counter_ ++;

		return AEC_current_expo_;
	}

}
