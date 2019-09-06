#include "angle_calc.h"
#include <math.h>
#define PI 3.14159265

void angle_init(){
    _angle_x = _angle_y = _angle_z = 0;
}

void angle_update(int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, int16_t acc_x, int16_t acc_y, int16_t acc_z){
    uint64_t x2,y2,z2;
	double result;

    int64_t _gyro_x, _gyro_y, _gyro_z, _acc_x, _acc_y, _acc_z;

	//Berechne korriegierten Beschleunigungs-Sensor-Wert	
	_acc_x =	((int64_t) acc_x);
	_acc_y =	((int64_t) acc_y);
	_acc_z =	((int64_t) acc_z);
	
	//TODO: Berechnung funktioniert nur bis 45Grad!
	//Vorbereitung Winkelberechnung durch Beschleunigungs-Sensor; quadriere
	x2 = _acc_x * _acc_x;
	y2 = _acc_y * _acc_y;
	z2 = _acc_z * _acc_z;
	
	//Winkelberechnungs-Vorschrift:
	//	98% GYRO(skaliere Winkel -> integriere Winkel -> addiere auf vorherigen Winkel)
	//	+
	//	2% ACC(Trigonometrische Winkelberechnung)
	
	result = sqrt(x2+z2);
	if(result != 0)
		result = _acc_y/result;
	
	
	//Berechne Ist-Winkel-x
	_angle_x = SHARE_GYRO*(_angle_x + DEGREE_PER_VALUE * _gyro_x * DT_GYRO_ASKING_TIME) + SHARE_ACC * -atan(result)*(180/PI);
	result = sqrt(y2+z2);
	if(result != 0)
		result = _acc_x/result;
	//Berechne Ist-Winkel-y
	_angle_y = SHARE_GYRO*(_angle_y + DEGREE_PER_VALUE * _gyro_y * DT_GYRO_ASKING_TIME) + SHARE_ACC * atan(result)*(180/PI);
	//Ist-Winkel-z wird aktuell nicht gebraucht
	//angle_z += (DEGREE_PER_VALUE * gyro_z * DT_GYRO_ASKING_TIME);
}