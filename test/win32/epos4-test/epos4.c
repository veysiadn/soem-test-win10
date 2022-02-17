#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include "ethercat.h"
#include "osal_win32.h"
#define NSEC_PER_SEC 1000000000
#define MSEC_PER_SEC 1000
#define EC_TIMEOUTMON 500
#define DC_CYCLE_TIME 2000000 //deadline in ns
#define CYCLE_TIME 2000
#define EPOS4 1 
#define DC_SYNC_SHIFT DC_CYCLE_TIME/2
#define PI 3.141592654
#define NUM_OF_SAMPLES 60000
#define STEP 0.00005
#define AMPLITUDE 100
#define TRESHOLD 50

#define TIMER_ADD(a, b, result)                                               \
  do {                                                                        \
    (result)->sec = (a)->sec + (b)->sec;                             \
    (result)->usec = (a)->usec + (b)->usec;                          \
    if ((result)->usec >= 1000000)                                         \
      {                                                                       \
        ++(result)->sec;                                                   \
        (result)->usec -= 1000000;                                         \
      }                                                                       \
  } while (0)

#define TIMER_SUB(a, b, result)                                               \
  do {                                                                        \
    (result)->sec = (a)->sec - (b)->sec;                             \
    (result)->usec = (a)->usec - (b)->usec;                          \
    if ((result)->usec < 0) {                                              \
      --(result)->sec;                                                     \
      (result)->usec += 1000000;                                           \
    }                                                                         \
  } while (0)

//Statusword EPOS state machine
#define SWITCH_ON_DISABLED 0x40 
#define READY_TO_SWITCH_ON 0x21
#define SWITCHED_ON 0x23
#define OPERATION_ENABLED 0x37 
#define QUICKSTOP_ACTIVE 0x17
#define FAULT_REACTION_ACTIVE 0x1f
#define FAULT 0X8

//Controlword EPOS state machine
#define SHUTDOWN 0X6
#define SWITCH_ON 0x7
#define SWITCH_ON_ENABLE 0xF
#define DISABLE_VOLTAGE 0x0
#define QUICK_STOP 0X2
#define FAULT_RESET 0X80 
#define STOP 0
#define MOVE 1

#define GEAR_REDUCTION 103
#define ENCODER_RESOLUTION 1024
#define ONE_TURN_TO_DEGREE 360

#define stack8k (8 * 1024)
#define stack64k (64 * 1024)

const double g_RotationToIncConstant = ((double)ONE_TURN_TO_DEGREE / ((double)ENCODER_RESOLUTION * (double)GEAR_REDUCTION));
const int	g_IncPerRotation = 4 * GEAR_REDUCTION * ENCODER_RESOLUTION;
char IOmap[4096];
OSAL_THREAD_HANDLE thread_ecat, thread_ecatcheck, thread_csptest;
int dorun = 1;
//int deltat, tmax = 0;
int64 toff, gl_delta;
int expectedWKC;
int64 time1;
int64 time2;
int64 cycle;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
// Save vals to arrays
int32 target_position_abs[NUM_OF_SAMPLES + 100];
int32 position_offset;
int actual_position_vals[NUM_OF_SAMPLES + 100];
int actaul_current_vals[NUM_OF_SAMPLES + 100];
int current_rpm_vals[NUM_OF_SAMPLES + 100];
int actual_torque_vals[NUM_OF_SAMPLES + 100];
int64 tv[NUM_OF_SAMPLES + 100];
int64 cycles[NUM_OF_SAMPLES + 100];
int counter = 2000;
int check = 0 ;
int shutdown_ = 1;

/* INIT = synchronization and opening of the mailbox
* PRE_OP = exchange of SDOs via mailbox to set PDOs and other default values
* SAFE_OP = open EtherCAT connection with PDO exchange
* OP = data exchange allowed both via mailbox and via EtherCAT(synchronous and asynchronous) */

// structure that represents the entries of the Object Dictionary
typedef struct
{
	int index;
	int sub_index;
	int size;
	int value;

} OBentry;

// structure representing the inputs of the EPOS
typedef struct PACKED
{
	uint16 controlword;
	int32 target_position;
	int32 position_offset;

} out_EPOSt;

out_EPOSt* out_EPOS;

// structure representing the outputs of the EPOS
typedef struct PACKED
{
	int32 position_actual_value;
	int32 velocity_actual_value;
	int16 torque_actual_value;
	uint16 statusword;
	int32 current_actual_value;

} in_EPOSt;

in_EPOSt* in_EPOS;

// degree to encoder increment conversion
int deg_to_inc(double deg) {

	int32 temp = (int32)(deg / g_RotationToIncConstant);
	return temp;

}

// encdoder increment to degree conversion
double inc_to_deg(int inc) {

	double temp = ((double)inc * g_RotationToIncConstant);
	return temp;

}

// function that maps PDOs and enables SYNC0
void CSP_PDO_mapping(uint16 slave) {

	int RxPDOs_number = 0;
	int TxPDOs_number = 0;
	int retval;


	OBentry RxPDOs_mapped = { 0x1600,0x00,sizeof(uint8),0 };
	retval = ec_SDOwrite(slave, RxPDOs_mapped.index, RxPDOs_mapped.sub_index, FALSE, RxPDOs_mapped.size, &(RxPDOs_mapped.value),
		EC_TIMEOUTSAFE);
	if (retval < 0) printf("Mapping failed\n");

	//0x6040=controlword_index, 0x00=controlword_subindex, 0x10=controlword_bitlength
	OBentry RxPDO1 = { 0x1600,0x01,sizeof(uint32),0x60400010 };
	retval = ec_SDOwrite(slave, RxPDO1.index, RxPDO1.sub_index, FALSE, RxPDO1.size, &(RxPDO1.value), EC_TIMEOUTSAFE);
	if (retval < 0) printf("Mapping failed\n");
	else RxPDOs_number++;

	//0x607A=target_position_index, 0x00=target_position_subindex, 0x20=taget_position_bitlength
	OBentry RxPDO2 = { 0x1600,0x02,sizeof(uint32),0x607A0020 };
	retval = ec_SDOwrite(slave, RxPDO2.index, RxPDO2.sub_index, FALSE, RxPDO2.size, &(RxPDO2.value), EC_TIMEOUTSAFE);
	if (retval < 0) printf("Mapping failed\n");
	else RxPDOs_number++;

	//0x60B0=position_offset_index, 0x00=position_offset_subindex, 0x20=position_offset_bitlength
	OBentry RxPDO3 = { 0x1600,0x03,sizeof(uint32),0x60B00020 };
	retval = ec_SDOwrite(slave, RxPDO3.index, RxPDO3.sub_index, FALSE, RxPDO3.size, &(RxPDO3.value), EC_TIMEOUTSAFE);
	if (retval < 0) printf("Mapping failed\n");
	else RxPDOs_number++;

	// report the number of mapped objects
	RxPDOs_mapped.value = RxPDOs_number;
	retval = ec_SDOwrite(slave, RxPDOs_mapped.index, RxPDOs_mapped.sub_index, FALSE, RxPDOs_mapped.size, &(RxPDOs_mapped.value),
		EC_TIMEOUTSAFE);
	if (retval < 0) printf("Mapping failed\n");

	// Send the changes to the RxPDO mapping to the SyncManager that manages the master -> slave SM2 data
	// set the value to 0 in order to modify it
	OBentry SM2PDOs_mapped = { 0x1C12,0x00,sizeof(uint8),0 };
	retval = ec_SDOwrite(slave, SM2PDOs_mapped.index, SM2PDOs_mapped.sub_index, FALSE, SM2PDOs_mapped.size, &(SM2PDOs_mapped.value),
		EC_TIMEOUTSAFE);
	if (retval < 0) printf("Mapping failed\n");

	//SM2_choose_RxPDO.value will contain the index of the objects in the OB that describe the chosen mapping
	OBentry SM2_choose_RxPDO = { 0x1C12,0x01,sizeof(uint16),RxPDO1.index };
	retval = ec_SDOwrite(slave, SM2_choose_RxPDO.index, SM2_choose_RxPDO.sub_index, FALSE, SM2_choose_RxPDO.size,
		&(SM2_choose_RxPDO.value), EC_TIMEOUTSAFE);
	if (retval < 0) printf("Mapping failed\n");

	// make the changes take effect
	SM2PDOs_mapped.value = 1;
	retval = ec_SDOwrite(slave, SM2PDOs_mapped.index, SM2PDOs_mapped.sub_index, FALSE, SM2PDOs_mapped.size, &(SM2PDOs_mapped.value),
		EC_TIMEOUTSAFE);
	if (retval < 0) printf("Mapping failed\n");

	//mappo i PDO che l'EPOS trasmette
	//numero TxPDO mappati (settato a 0 per poter cambiare la mappatura)
	OBentry TxPDOs_mapped = { 0x1A00,0x00,sizeof(uint8),0 };
	retval = ec_SDOwrite(slave, TxPDOs_mapped.index, TxPDOs_mapped.sub_index, FALSE, TxPDOs_mapped.size, &(TxPDOs_mapped.value),
		EC_TIMEOUTSAFE);
	if (retval < 0) printf("Mapping failed\n");

	//primo elemento mappato
	//0x6064=position_actual_value_index, 0x00=position_actual_value_subindex, 0x20=position_actual_value_bitlength
	OBentry TxPDO1 = { 0x1A00,0x01,sizeof(uint32),0x60640020 };
	retval = ec_SDOwrite(slave, TxPDO1.index, TxPDO1.sub_index, FALSE, TxPDO1.size, &(TxPDO1.value), EC_TIMEOUTSAFE);
	if (retval < 0) printf("Mapping failed\n");
	else TxPDOs_number++;

	//secondo elemento mappato
	//0x606C=velocity_actual_value_index, 0x00=velocity_actual_value_subindex, 0x20=velocity_actual_value_bitlength
	OBentry TxPDO2 = { 0x1A00,0x02,sizeof(uint32),0x606C0020 };
	retval = ec_SDOwrite(slave, TxPDO2.index, TxPDO2.sub_index, FALSE, TxPDO2.size, &(TxPDO2.value), EC_TIMEOUTSAFE);
	if (retval < 0) printf("Mapping failed\n");
	else TxPDOs_number++;

	//terzo elemento mappato
	//0x6077=torque_actual_value_index, 0x00=torque_actual_value_subindex, 0x10=torque_actual_value_bitlength
	OBentry TxPDO3 = { 0x1A00,0x03,sizeof(uint32),0x60770010 };
	retval = ec_SDOwrite(slave, TxPDO3.index, TxPDO3.sub_index, FALSE, TxPDO3.size, &(TxPDO3.value), EC_TIMEOUTSAFE);
	if (retval < 0) printf("Mapping failed\n");
	else TxPDOs_number++;

	//quarto elemento mappato
	//0x6041=statusword_index, 0x00=statusword_subindex, 0x10=statusword_bitlength
	OBentry TxPDO4 = { 0x1A00,0x04,sizeof(uint32),0x60410010 };
	retval = ec_SDOwrite(slave, TxPDO4.index, TxPDO4.sub_index, FALSE, TxPDO4.size, &(TxPDO4.value), EC_TIMEOUTSAFE);
	if (retval < 0) printf("Mapping failed\n");
	else TxPDOs_number++;

	//quinto elemento mappato
	//0x30D0=current_actual_value_index, 0x02=current_actual_value_subindex, 0x20=current_actual_value_bitlength
	OBentry TxPDO5 = { 0x1A00,0x05,sizeof(int32),0x30D10220 };
	retval = ec_SDOwrite(slave, TxPDO5.index, TxPDO5.sub_index, FALSE, TxPDO5.size, &(TxPDO5.value), EC_TIMEOUTSAFE);
	if (retval < 0) printf("Mapping failed\n");
	else TxPDOs_number++;

	//comunico il numero di oggetti 
	TxPDOs_mapped.value = TxPDOs_number;
	retval = ec_SDOwrite(slave, TxPDOs_mapped.index, TxPDOs_mapped.sub_index, FALSE, TxPDOs_mapped.size, &(TxPDOs_mapped.value),
		EC_TIMEOUTSAFE);
	if (retval < 0) printf("Mapping failed\n");

	//comunico al SyncManager che gestisce i dati master <- slave SM3 le modifiche precedenti alla mappatura dei TxPDO
	//setto a 0 il valore per poterlo modificare
	OBentry SM3PDOs_mapped = { 0x1C13,0x00,sizeof(uint8),0 };
	retval = ec_SDOwrite(slave, SM3PDOs_mapped.index, SM3PDOs_mapped.sub_index, FALSE, SM3PDOs_mapped.size, &(SM3PDOs_mapped.value),
		EC_TIMEOUTSAFE);
	if (retval < 0) printf("Mapping failed\n");

	//SM2_choose_TxPDO.value conterrà l'indice degli oggetti nell'OB che descrivono la mappatura prescelta
	OBentry SM3_choose_TxPDO = { 0x1C13,0x01,sizeof(uint16),TxPDO1.index };
	retval = ec_SDOwrite(slave, SM3_choose_TxPDO.index, SM3_choose_TxPDO.sub_index, FALSE, SM3_choose_TxPDO.size,
		&(SM3_choose_TxPDO.value), EC_TIMEOUTSAFE);
	if (retval < 0) printf("Mapping failed\n");

	//rendo effettive le modifiche
	SM3PDOs_mapped.value = 1;
	retval = ec_SDOwrite(slave, SM3PDOs_mapped.index, SM3PDOs_mapped.sub_index, FALSE, SM3PDOs_mapped.size, &(SM3PDOs_mapped.value),
		EC_TIMEOUTSAFE);
	if (retval < 0) printf("Mapping failed\n");

	ec_dcsync0(EPOS4, TRUE, DC_CYCLE_TIME, DC_SYNC_SHIFT);

}

// Setup configuration parameters for EPOS4
int CSP_EPOSsetup(uint16 slave) {

	OBentry mode = { 0x6060, 0x00, sizeof(int8_t),(int8_t)8 }; //modalità di funzionamento 8=CSP
	OBentry period = { 0x60C2, 0x01, sizeof(uint8),(uint8)4 }; //periodo di interpolazione in ms
    OBentry profile_vel = { 0x6081,0x00, sizeof(int32_t),(uint32_t)750 };

//	OBentry nominal_current = { 0x3001, 0x01, sizeof(uint32),(uint32)2000 }; //corrente nominale in mA
//	OBentry max_current = { 0x3001, 0x02, sizeof(uint32),(uint32)(nominal_current.value) }; //corrente massima del motore in mA
//	OBentry thermal_time_constant_winding = { 0x3001, 0x04, sizeof(uint16),(uint16)440 }; //costante di tempo termica di avvolgimento in 0.1s
//	OBentry torque_costant = { 0x3001, 0x05, sizeof(uint32),(uint32)58500 }; //costante di coppia in uNm/A
//	OBentry following_error_window = { 0x6065, 0x00, sizeof(uint32),(uint32)1e6 };
//	//scostamento max tra riferimento e posizione attuale
//	OBentry actual_position = { 0x6064, 0x00, sizeof(int32),(int32)0 };
//	//parametri da settare
//	int8_t mode_prova;
//	uint8 period_prova;
//	uint32 nominal_current_prova;
//	uint32 max_current_prova;
//	uint16 thermal_time_constant_winding_prova;
//	uint16 torque_costant_prova;
//	uint32 following_error_window_prova;

	/* non settabili o corretti di default
	int32_t position_offset_prova;
	int16_t torque_offset_prova;
	uint32 main_sensor_resolution_prova;
	uint32 control_prova;
	uint32 sensor_prova;
	uint32 max_gear_input_speed_prova;
	uint32 max_motor_speed_prova;*/

	int retval;
	/*int length32=sizeof(uint32);
	int length16=sizeof(int16_t);*/
    retval = ec_SDOwrite(slave, mode.index, mode.sub_index, FALSE, mode.size, &(mode.value), EC_TIMEOUTSAFE);
    printf("%d\n", retval);
    retval = ec_SDOwrite(slave, period.index, period.sub_index, FALSE, sizeof(uint8), &(period.value), EC_TIMEOUTSAFE);
    printf("%d\n", retval);

	retval = ec_SDOwrite(slave, profile_vel.index, profile_vel.sub_index, FALSE, sizeof(int32_t), &(profile_vel.value), EC_TIMEOUTSAFE);
	printf("%d\n", retval);

//	retval = ec_SDOwrite(slave, nominal_current.index, nominal_current.sub_index, FALSE, sizeof(uint32), &(nominal_current.value),
//		EC_TIMEOUTSAFE);
//	printf("%d\n", retval);

//	retval = ec_SDOwrite(slave, max_current.index, max_current.sub_index, FALSE, sizeof(uint32), &(max_current.value), EC_TIMEOUTSAFE);
//	printf("%d\n", retval);

//	retval = ec_SDOwrite(slave, thermal_time_constant_winding.index, thermal_time_constant_winding.sub_index, FALSE, sizeof(uint16),
//		&(thermal_time_constant_winding.value), EC_TIMEOUTSAFE);
//	printf("%d\n", retval);

//	retval = ec_SDOwrite(slave, torque_costant.index, torque_costant.sub_index, FALSE, sizeof(uint32), &(torque_costant.value),
//		EC_TIMEOUTSAFE);
//	printf("%d\n", retval);

//	/* SETUP degli oggetti relativi alla modalità CSP */

//	retval = ec_SDOwrite(slave, following_error_window.index, following_error_window.sub_index, FALSE,
//		sizeof(uint32), &(following_error_window.value), EC_TIMEOUTSAFE);
//	printf("%d\n", retval);

//	//controllo che i parametri siano impostati correttamente
//	retval = ec_SDOread(slave, 0x6061, mode.sub_index, FALSE, &(mode.size), &mode_prova, EC_TIMEOUTSAFE);
//	printf("mode=%d,letti=%d,esito=%d\n", mode_prova, mode.size, retval);

//	retval = ec_SDOread(slave, period.index, period.sub_index, FALSE, &(period.size), &period_prova, EC_TIMEOUTSAFE);
//	printf("period=%u,letti=%d,esito=%d\n", period_prova, period.size, retval);

//	retval = ec_SDOread(slave, nominal_current.index, nominal_current.sub_index, FALSE, &(nominal_current.size),
//		&nominal_current_prova, EC_TIMEOUTSAFE);
//	printf("nominal_current=%u,letti=%d, esito=%d\n", nominal_current_prova, nominal_current.size, retval);

//	retval = ec_SDOread(slave, max_current.index, max_current.sub_index, FALSE, &(max_current.size),
//		&max_current_prova, EC_TIMEOUTSAFE);
//	printf("max_current=%u,letti=%d, esito=%d\n", max_current_prova, max_current.size, retval);

//	retval = ec_SDOread(slave, thermal_time_constant_winding.index, thermal_time_constant_winding.sub_index,
//		FALSE, &(thermal_time_constant_winding.size), &thermal_time_constant_winding_prova, EC_TIMEOUTSAFE);
//	printf("thermal_time_constant_winding=%u,letti=%d, esito=%d\n", thermal_time_constant_winding_prova,
//		thermal_time_constant_winding.size, retval);

//	retval = ec_SDOread(slave, torque_costant.index, torque_costant.sub_index, FALSE, &(torque_costant.size),
//		&torque_costant_prova, EC_TIMEOUTSAFE);
//	printf("torque_costant=%u,letti=%d, esito=%d\n", torque_costant_prova, torque_costant.size, retval);

//	retval = ec_SDOread(slave, following_error_window.index, following_error_window.sub_index,
//		FALSE, &(following_error_window.size), &following_error_window_prova, EC_TIMEOUTSAFE);
//	printf("errore=%d,letti=%d,esito=%d\n", following_error_window_prova, following_error_window.size, retval);

//	retval = ec_SDOread(slave, actual_position.index, actual_position.sub_index,
//		FALSE, &(actual_position.size), &position_offset, EC_TIMEOUTSAFE);
//	printf("offset=%d,letti=%d,esito=%d\n", position_offset, actual_position.size, retval);

	/*retval=ec_SDOread(slave, 0x3000, 0x05, FALSE, &length32, &main_sensor_resolution_prova, EC_TIMEOUTSAFE);
	printf("risoluzione=%u,letti=%d, esito=%d\n",main_sensor_resolution_prova,length32,retval);

	retval=ec_SDOread(slave, 0x60B2, 00, FALSE, &(length16), &torque_offset_prova, EC_TIMEOUTSAFE);
	printf("torque_offset=%u,letti=%d, esito=%d\n",torque_offset_prova,length16, retval);

	retval=ec_SDOread(slave, 0x60B0, 00, FALSE, &(length32), &position_offset_prova, EC_TIMEOUTSAFE);
	printf("position_offset=%u,letti=%d, esito=%d\n",position_offset_prova,length32, retval);

	retval=ec_SDOread(slave, 0x3000, 0x02, FALSE, &length32, &control_prova, EC_TIMEOUTSAFE);
	printf("controllo=%8x,letti=%d, esito=%d\n",control_prova,length32,retval);

	retval=ec_SDOread(slave, 0x3000, 0x01, FALSE, &length32, &sensor_prova, EC_TIMEOUTSAFE);
	printf("sensor=%x,letti=%d, esito=%d\n",sensor_prova,length32,retval);

	retval=ec_SDOread(slave, 0x6080, 0x00, FALSE, &length32, &max_motor_speed_prova, EC_TIMEOUTSAFE);
	printf("max_motor_speed=%u,letti=%d, esito=%d\n",max_motor_speed_prova,length32,retval);

	retval=ec_SDOread(slave, 0x3003, 0x03, FALSE, &length32, &max_gear_input_speed_prova, EC_TIMEOUTSAFE);
	printf("max_gear_input_speed=%u,letti=%d, esito=%d\n",max_gear_input_speed_prova,length32,retval);*/

	//mappo i PDO
	CSP_PDO_mapping(EPOS4);
	return 1;

}

// CiA402 State machine control function
int Servo_state_machine(int flag) {

	switch (in_EPOS->statusword & 0xff) {

	case SWITCH_ON_DISABLED:
		if (flag)
			out_EPOS->controlword = SHUTDOWN;
		else
			out_EPOS->controlword = DISABLE_VOLTAGE;

		return 0;
		break;

	case READY_TO_SWITCH_ON:
		if (flag)
			out_EPOS->controlword = SWITCH_ON;
		else
			out_EPOS->controlword = DISABLE_VOLTAGE;

		return 0;
		break;

	case SWITCHED_ON:
		if (flag) {
			out_EPOS->controlword = SWITCH_ON_ENABLE;
			return 1;
		}

		else {
			out_EPOS->controlword = DISABLE_VOLTAGE;
			return 0;
		}
		break;

	case OPERATION_ENABLED:
		if (flag) {
			out_EPOS->controlword = SWITCH_ON_ENABLE;
			return 1;
		}
		else {
			out_EPOS->controlword = DISABLE_VOLTAGE;
			return 0;
		}
		break;

	case QUICK_STOP:
		if (flag) {
			out_EPOS->controlword = SWITCH_ON;
			return 1;
		}
		else {
			out_EPOS->controlword = DISABLE_VOLTAGE;
			return 0;
		}

		break;

	case FAULT_REACTION_ACTIVE:
		out_EPOS->controlword = FAULT_RESET;
		return 0;
		break;

	case FAULT:
		out_EPOS->controlword = FAULT_RESET;
		return 0;
		break;

	default:
		printf("%x", in_EPOS->statusword);
		return 0;
		break;

	}
}

//funzione che aggiorna quando ecathread dovrà svegliarsi
//void add_timespec(struct timespec* ts, int64 addtime) {
//	int64 sec, nsec;
//
//	nsec = addtime % NSEC_PER_SEC;
//	sec = (addtime - nsec) / NSEC_PER_SEC;
//	ts->sec += sec;
//	ts->tv_nsec += nsec;
//	if (ts->tv_nsec > NSEC_PER_SEC)
//	{
//		nsec = ts->tv_nsec % NSEC_PER_SEC;
//		ts->sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
//		ts->tv_nsec = nsec;
//	}
//}

// synchronization of the clock of the master and the network
void ec_sync(int64 reftime, int64 cycletime, int64* offsettime) {
	static int64 integral = 0;
	int64 delta;

	delta = (reftime) % cycletime;
	if (delta > (cycletime / 2)) { delta = delta - cycletime; }
	if (delta > 0) { integral++; }
	if (delta < 0) { integral--; }
	*offsettime = -(delta / 100) - (integral / 20);
	gl_delta = delta;
}

/* RT EtherCAT thread */
OSAL_THREAD_FUNC ecatthread() {
	struct timespec ts = {0}; 
	struct timespec tleft = {0};
	int ht=0;
	int64 cycletime;
	ec_timet  t_start={0}, t_left={0}, t_end={0};
	ec_timet cycle_time = { 0,CYCLE_TIME };

	t_start = osal_current_time();
	cycletime = DC_CYCLE_TIME; /* cycletime in ns */
	toff = 0;

	ec_send_processdata();

	while (counter) {
		osal_usleep(2000);
		wkc = ec_receive_processdata(EC_TIMEOUTRET);
		ec_sync(ec_DCtime, cycletime, &toff);
		ec_send_processdata();
		counter--;
	}

	// brings the controller into the OPERATION_ENABLED state 
	// takes about 20 ms
	while (check != 1) {
		osal_usleep(2000);
		wkc = ec_receive_processdata(EC_TIMEOUTRET);
		check = Servo_state_machine(MOVE);
		ec_sync(ec_DCtime, cycletime, &toff);
		ec_send_processdata();
	}

	counter = 0;

	// provides all references of the trajectory // lasts SAMPLES * 1ms
	t_start = osal_current_time();
	t_end = t_start; 
	while (counter < NUM_OF_SAMPLES)
	{
		time1 = ec_DCtime;
		// calculate next cycle start 
		// add_timespec(&ts, cycletime + toff);
		// wait to cycle start 
		//TIMER_ADD(&t_start, &cycle_time,&t_start);
		//clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
		//if (dorun>0)
		osal_time_diff(&t_start, &t_end, &t_left);
		t_start = osal_current_time();
		// important sleep parameter effects all sync
		if (t_left.usec < 2000) {
			osal_usleep((2000 - t_left.usec));
		}
		else
		{
			osal_usleep(2000);
		}
		wkc = ec_receive_processdata(EC_TIMEOUTRET);
		//calcola toff per sincronizzare il master e l'orologio DC 
		ec_sync(ec_DCtime, cycletime, &toff);
		check = Servo_state_machine(MOVE);
		if (TRUE) {
			actual_position_vals[counter] = in_EPOS->position_actual_value;
			actaul_current_vals[counter] = in_EPOS->current_actual_value;
			current_rpm_vals[counter] = in_EPOS->velocity_actual_value;
			actual_torque_vals[counter] = in_EPOS->torque_actual_value;
			tv[counter] = ec_DCtime;
			out_EPOS->target_position = target_position_abs[counter];
			ec_send_processdata();
			time2 = ec_DCtime;
			cycle = time2 - time1;
			cycles[counter] = cycle;
			counter++;
		}
		else {
			printf("All samples collected, exiting thread... \n");
			return;
		}
		t_end = osal_current_time();
	}

	// counter=NUM_OF_SAMPLES
	// Cycle to stop at final position
	// Takes about 15-20ms
	t_start = osal_current_time();
	t_end = t_start; 
	while ((in_EPOS->position_actual_value <= (target_position_abs[NUM_OF_SAMPLES - 1] - TRESHOLD + out_EPOS->position_offset)) ||
		(in_EPOS->position_actual_value >= (target_position_abs[NUM_OF_SAMPLES - 1] + TRESHOLD + out_EPOS->position_offset))) {

		//TIMER_ADD(&T_START, &CYCLE_TIME, &T_START);
		osal_time_diff(&t_start, &t_end, &t_left);
		t_start = osal_current_time();
		// important sleep parameter effects all sync operation
		if (t_left.usec < 2000) {
			osal_usleep((2000 - t_left.usec));
		}
		else
		{
			osal_usleep(2000);
		}
		wkc = ec_receive_processdata(EC_TIMEOUTRET);
		ec_sync(ec_DCtime, cycletime, &toff);
		check = Servo_state_machine(MOVE);
		if (check) {
			actual_position_vals[counter] = in_EPOS->position_actual_value;
			actaul_current_vals[counter] = in_EPOS->current_actual_value;
			current_rpm_vals[counter] = in_EPOS->velocity_actual_value;
			actual_torque_vals[counter] = in_EPOS->torque_actual_value;
			target_position_abs[counter] = target_position_abs[NUM_OF_SAMPLES - 1];
			tv[counter] = ec_DCtime;
			out_EPOS->target_position = target_position_abs[NUM_OF_SAMPLES - 1];
			ec_send_processdata();
			counter++;
		}

		else {
			printf("Cycle stop, exiting thread... \n");
			return;
		}
		t_end = osal_current_time();
	}

	int j = 500;
	while (j) {
		//TIMER_ADD(&t_start, &cycle_time, &t_start);
		osal_usleep(2000);
		wkc = ec_receive_processdata(EC_TIMEOUTRET);
		ec_sync(ec_DCtime, cycletime, &toff);
		ec_send_processdata();
		j--;
	}

	//add_timespec(&ts, cycletime + toff);
	osal_usleep(2000);
	ec_sync(ec_DCtime, cycletime, &toff);
	Servo_state_machine(STOP); // Bring controller initial state
	ec_send_processdata();
	wkc = ec_receive_processdata(EC_TIMEOUTRET);

	//add_timespec(&ts, cycletime + toff);
	osal_usleep(2000);
	ec_sync(ec_DCtime, cycletime, &toff);

	ec_slave[0].state = EC_STATE_INIT; // Bring slaves to init state.
	ec_writestate(0);
	ec_statecheck(0, EC_STATE_INIT, 5 * EC_TIMEOUTSTATE);

	ec_dcsync0(EPOS4, FALSE, 0, 0); // deactivate SYNC0
	ec_close();  // close connection
	shutdown_ = 0;

}


OSAL_THREAD_FUNC CSP_test(char* ifname) {
	int cnt;
	int print_count = 1000;
	printf("Starting Redundant test\n");

	// initialize SOEM and connect it to the ifname port
	if (ec_init(ifname))
	{
		printf("ec_init on %s succeeded.\n", ifname);
		// Enumerate and init all slaves.
		if (ec_config_init(FALSE) > 0)
		{
			printf("%d slaves found and configured.\n", ec_slavecount);

			ec_slave[0].state = EC_STATE_PRE_OP;
			ec_writestate(0);
			ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4);

			//when the PRE_OP-> SAFE_OP transition takes place call CSP_EPOSsetup to 
			// set the parameters and map the PDOs
			ec_slave[EPOS4].PO2SOconfig = CSP_EPOSsetup;

			// Configure DC.
			ec_configdc();
			// map the previously mapped PDOs into the local buffer
			ec_config_map(&IOmap);

			out_EPOS = (out_EPOSt*)ec_slave[1].outputs; // master commands
			in_EPOS = (in_EPOSt*)ec_slave[1].inputs;  // slave feedback
			// read the current starting position to start the movement assuming this as 0
			//out_EPOS->position_offset = position_offset + deg_to_inc((double)AMPLITUDE);

			// read and store the state in the ec_slave [] vector
			ec_readstate();
			for (cnt = 1; cnt <= ec_slavecount; cnt++)
			{
				printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d\n",
					cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
					ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
			}

			expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
			printf("Calculated workcounter %d\n", expectedWKC);

			ec_slave[0].state = EC_STATE_SAFE_OP;
			ec_writestate(0);
			ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

			printf("Request operational state for all slaves\n");
			ec_slave[0].state = EC_STATE_OPERATIONAL;
			ec_writestate(0);
			// create real-time thread for ecat
			osal_thread_create_rt(&thread_ecat, stack64k * 2, &ecatthread, NULL);
			//dorun = 1;
		    // Wait for operational state
			ec_statecheck(0, EC_STATE_OPERATIONAL, 5 * EC_TIMEOUTSTATE);

			if (ec_slave[0].state == EC_STATE_OPERATIONAL)
			{
				printf("Operational state reached for all slaves.Actual state=%d\n", ec_slave[0].state);
				inOP = TRUE;
				//ciclo per stampare i dati in tempo reale
				while (shutdown_)
				{
					printf("PDO n.i=%d,target_position=%d,cycle1=%I64d,cycle2=%d \n",
						counter, out_EPOS->target_position + out_EPOS->position_offset, cycle, ec_slave[1].DCcycle);

					printf("statusword %4x,controlword %x, position_actual_value %d\n",
						in_EPOS->statusword, out_EPOS->controlword, in_EPOS->position_actual_value);
					osal_usleep(500000);
				}
				//dorun = 0;
				inOP = FALSE;
			}
			else
			{
				printf("Not all slaves reached operational state.\n");
				ec_readstate();
				for (int j = 1; j <= ec_slavecount; j++)
				{
					if (ec_slave[j].state != EC_STATE_OPERATIONAL)
					{
						printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
							j, ec_slave[j].state, ec_slave[j].ALstatuscode, ec_ALstatuscode2string(ec_slave[j].ALstatuscode));
					}
				}
			}
		}
		else
		{
			printf("No slaves found!\n");
		}
		printf("End CSP test, close socket\n");
		exit(-1);
	}
	else
	{
		printf("No socket connection on %s\nExcecute as root\n", ifname);
	}
	//prima di terminare aspetta che ecatthread abbia finito tutto
	//pthread_join(thread1, NULL);
}


OSAL_THREAD_FUNC ecatcheck() {
	int slave;

	while (1)
	{
		if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
		{
			if (needlf)
			{
				needlf = FALSE;
				printf("\n");
			}
			/* one ore more slaves are not responding */
			ec_group[currentgroup].docheckstate = FALSE;
			ec_readstate();
			for (slave = 1; slave <= ec_slavecount; slave++)
			{
				if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
				{
					ec_group[currentgroup].docheckstate = TRUE;
					if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
					{
						printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
						ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
						ec_writestate(slave);
					}
					else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
					{
						printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
						ec_slave[slave].state = EC_STATE_OPERATIONAL;
						ec_writestate(slave);
					}
					else if (ec_slave[slave].state > EC_STATE_NONE)
					{
						if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
						{
							ec_slave[slave].islost = FALSE;
							printf("MESSAGE : slave %d reconfigured\n", slave);
						}
					}
					else if (!ec_slave[slave].islost)
					{
						/* re-check state */
						ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
						if (ec_slave[slave].state == EC_STATE_NONE)
						{
							ec_slave[slave].islost = TRUE;
							printf("ERROR : slave %d lost\n", slave);
						}
					}
				}
				if (ec_slave[slave].islost)
				{
					if (ec_slave[slave].state == EC_STATE_NONE)
					{
						if (ec_recover_slave(slave, EC_TIMEOUTMON))
						{
							ec_slave[slave].islost = FALSE;
							printf("MESSAGE : slave %d recovered\n", slave);
						}
					}
					else
					{
						ec_slave[slave].islost = FALSE;
						printf("MESSAGE : slave %d found\n", slave);
					}
				}
			}
			if (!ec_group[currentgroup].docheckstate)
				printf("OK : all slaves resumed OPERATIONAL.\n");
		}
		osal_usleep(200000);
	}
}


int main(int argc, char* argv[]) {
	char ifbuf[1024] = "\\Device\\NPF_{2C4C20D3-EBAA-4C1F-B7B0-E0C399EA0CDA}";

	printf("SOEM (Simple Open EtherCAT Master)\nCSP test\n");

		//dorun = 0;
		double t = 0;   //tempo in secondi
		double f = 3;  //frequenza in Hz
		int rotation = 0;
		const double counter = g_IncPerRotation*2 / NUM_OF_SAMPLES;
		target_position_abs[0] = 0;
		for (int j = 1; j < NUM_OF_SAMPLES; j++) {
			//target_position_abs[j] = deg_to_inc(-((double)AMPLITUDE) * cos(2 * PI * f * t));
			//target_position_abs[j]=deg_to_inc(((double)AMPLITUDE)*sin(2*PI*f*t));
			if (!rotation) {
				target_position_abs[j] = target_position_abs[j-1]+(int32)counter;
				if (target_position_abs[j] > g_IncPerRotation)
					rotation = 1;
			}
			else
			{
				target_position_abs[j] = target_position_abs[j - 1] - (int32)counter;
				if (target_position_abs[j] < 0)
					rotation = 0;
			}
		}
		/* create RT thread */
		 //osal_thread_create(&thread1, stack8k * 2, &ecatthread, NULL);
		  /* create thread to handle slave error handling in OP */
		osal_thread_create(&thread_ecatcheck, stack8k * 4, &ecatcheck, NULL);
		osal_thread_create_rt(&thread_csptest, stack8k * 4, &CSP_test, ifbuf);

		/* start acyclic part */

		osal_usleep(60000000);
	//	FILE* fposizione, * fcicli, * fcorrente, * fp;

	//	fposizione = fopen("posizione.txt", "wt");
	//	fcorrente = fopen("corrente.txt", "wt");
	//	fcicli = fopen("cicli.txt", "wt");

	//	for (int j = 0; j < i; j++) {
	//		//matlab legge \n correttamente come a capo, in Windows serve \r\n
	//		fprintf(fposizione, "%f %f %f\r\n", ((double)(tv[j] - tv[0]) / (double)(NSEC_PER_SEC)),
	//			inc_to_deg(misure_posizione[j]), inc_to_deg(target_position_abs[j] + out_EPOS->position_offset));

	//		fprintf(fcicli, "%f %ld\r\n", ((double)(tv[j] - tv[0]) / (double)(NSEC_PER_SEC)), cicli[j]);

	//		fprintf(fcorrente, "%f %d %d %d\n", ((double)(tv[j] - tv[0]) / (double)(NSEC_PER_SEC)), misure_corrente[j],
	//			misure_rpm[j], misure_coppia[j]);
	//	}

	//	fclose(fposizione);
	//	fclose(fcicli);
	//	fclose(fcorrente);
	//	//per usare GNUplot da riga di comando
	//	fp = fopen("comando.txt", "wt");

	//	//scrivo sul file il comando da eseguire 

	//	fprintf(fp, "plot \"posizione.txt\" with lines\n");

	//	//chiudo il file su cui ho scritto il comando da eseguire 

	//	fclose(fp);

	//	//eseguo il programma GNUplot passandogli il nome del file che contiene il comando da eseguire

	//	system("gnuplot -persist comando.txt");

	//}
	//else
	//{
	//	printf("Usage: CSP_test ifname \nifname = eth0 for example\n");
	//}
	printf("End program\n");
	return (0);
}
