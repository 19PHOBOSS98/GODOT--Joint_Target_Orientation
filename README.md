# GODOT--Joint_Target_Orientation

Currently, Godot's Physics Joints use Euler angles. I needed joints that use Quaternions so I wrote myself a prototype instead. Plan to use them to puppet physics bodies with IK for my building game.

YT Video:
https://youtu.be/htMDZ0fXJWo

![001](https://user-images.githubusercontent.com/37253663/152513594-8664cd0a-4f48-4bde-9882-42eccecb7944.png)


<img width="1206" alt="Screen Shot 2022-02-06 at 7 17 30 AM" src="https://user-images.githubusercontent.com/37253663/152662183-76c1b943-bfc9-4248-b293-dda5869be2f9.png">


Works in chains too:

![003](https://user-images.githubusercontent.com/37253663/152647270-7901cdda-60ce-4f41-856f-6728b5bf5b24.png)


I'm working on a new joint that would use basis transforms from a given node instead of using angle rotations. This way it doesn't need to calculate the target rotations so it's faster.

Regular Angle Rotation Input:

<img width="596" alt="Screen Shot 2022-02-05 at 11 09 12 PM" src="https://user-images.githubusercontent.com/37253663/152647458-35681e65-a7a5-4530-92af-de537540228e.png">

Basis Vector Transform from "Puppeteer" node:

<img width="1271" alt="Screen Shot 2022-02-05 at 11 16 41 PM" src="https://user-images.githubusercontent.com/37253663/152647647-212c9c96-079c-44ae-a55b-1c10fda87dd5.png">


![002](https://user-images.githubusercontent.com/37253663/152647516-0660b44c-a222-445b-ba05-b908a4e8131d.png)

![001](https://user-images.githubusercontent.com/37253663/152647513-cbbbf4fb-5bf6-48f2-81c6-4685e1ab87fc.png)


My (Current) Quaternion Angle Calculation and Safeguards for when the quaternion axis reaches infinity:
```
	...
	var qBT = Quat(body_b.global_transform.basis)	#Quaternion Node B Transform Basis
	
	
	var qTargetO = calc_target_orientation_by_basis()	#Quaternion Target Orientation, using puppeteer_node transform basis
	#var qTargetO = calc_target_orientation(body_a.global_transform.basis)	#Does the same thing but using manual euler angles instead
	
	var rotChange = qTargetO * qBT.inverse()	#rotation change quaternion
	
	var angle = 2.0 * acos(rotChange.w)	#Turns to angle radians, the amount it turns around the quats axis
	
	
	#if node B's quat is already facing the same way as qTargetO the axis shoots to infinity
	#this is my sorry ass attempt to protect us from it:
	
	if(is_nan(angle)):
		angle = 0
	
	var axis = Vector3(0,0,0)
	var v = Vector3(rotChange.x,rotChange.y,rotChange.z)	#rotation change quaternion "V" component
	
	var axis_den = sin(angle*0.5)	#quaternion axis denominator
	if(is_nan(axis_den)):
		axis = Vector3(0,0,0)
	elif(axis_den==0):
		axis = Vector3(0,0,0)
	else:
		axis = v / axis_den	#the quaternion's axis
	
	var axis_Mag = axis.length()	#axis magnitude
	if(is_nan(axis_Mag)||is_inf(axis_Mag)):		#here's the axis shooting up to infinity
		axis = Vector3(0,0,0)			#Might as well set it to zero since the error would be zero at this point

	if(angle>PI):		
		angle -= 2.0*PI



	var error = axis*angle
	...
```

Here's the function where you can manually set the rest angle:
```
func calc_target_orientation(Abasis:Basis):

	var baseBTOActual = Abasis*baseBTOrig#the actual original base node B Transform following node A around in current global space

	var qx = Quat(Abasis.x,rest_angle_x*PI/180.0)
	var qy = Quat(Abasis.y,rest_angle_y*PI/180.0)
	var qz = Quat(Abasis.z,rest_angle_z*PI/180.0)
	
	#var qBTargRo = qz*qy*qx# Quaternion node B Target Rotation
	var qBTargRo = qx*qy*qz
	var BTargetBasis = Basis()
	BTargetBasis.x =  qBTargRo*baseBTOActual.x
	BTargetBasis.y =  qBTargRo*baseBTOActual.y
	BTargetBasis.z =  qBTargRo*baseBTOActual.z

	return Quat(BTargetBasis)
```

Here's the function where you can use a "Puppeteer" transform basis instead. This is way faster in my opinion. Just orient the Puppeteer and the joint should try to follow it:
```
func calc_target_orientation_by_basis():
	var BTargetBasis = get_node(puppeteer_node).global_transform.basis #can be any node as long as it has it's own basis vector
	return Quat(BTargetBasis)
```


Thanks to:

DMGregory: https://gamedev.stackexchange.com/questions/182850/rotate-rigidbody-to-face-away-from-camera-with-addtorque/182873#182873
	
The Step Event: https://youtu.be/vewwP8Od_7s

h4tt3n: https://www.gamedev.net/tutorials/programming/math-and-physics/towards-a-simpler-stiffer-and-more-stable-spring-r3227/

For the calculations

