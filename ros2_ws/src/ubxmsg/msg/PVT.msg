uint32 time
uint16 year
uint8 month
uint8 day
uint8 hour
uint8 min
uint8 second
bool valid_data
bool valid_time
bool fully_resolved
bool valid_mag
uint32 t_acc # ns
int32 nano # ns
uint8 fix_type #0:no fix, 1:dead reckoning only, 2:2d fix, 3:3d fix 4:GNSS + dead reckoning combine, 5:time fix only
bool gnss_fix_ok
bool dif_soln
uint8 psm_state
bool head_veh_valid
uint8 carr_soln
bool comfirmed_avai
bool confirmed_data
bool confirmed_time
uint8 num_sv
float32 lon # 1e-7 deg
float32 lat # 1e-7 deg
int32 height # mm
int32 h_msl # mm
uint32 h_acc # mm
uint32 v_acc # mm
int32 vel_n # mm/s
int32 vel_e # mm/s
int32 vel_d # mm/s
float32 ground_speed # mm/s
float32 head_mot # 1e-5 deg
uint32 s_acc
float32 head_acc
float32 p_dop
bool invalid_llh
uint32 last_correction_age
float32 head_veh # 1e-5 deg
float32 mag_dec # 1e-2 deg
float32 mag_acc # 1e-2 deg
