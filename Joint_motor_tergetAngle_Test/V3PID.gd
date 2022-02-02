extends Node
class_name Vector3_PID

var _prev_error: Vector3
var _integral: Vector3
var _int_max = 200
export var _Kp: float = 0.01
export var _Ki: float = 2
export var _Kd: float = 0
export var _dt = 0.01


func Update(currentError:Vector3 , timeFrame:float ):

	_integral += currentError * timeFrame;
	var deriv = (currentError - _prev_error) / timeFrame;
	_prev_error = currentError;
	return currentError * _Kp + _integral * _Ki + deriv * _Kd;
