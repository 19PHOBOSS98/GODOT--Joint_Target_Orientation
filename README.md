# GODOT--Joint_Target_Orientation

Currently, Godot's Physics Joints use Euler angles. I needed joints that use Quaternions so I wrote myself a prototype instead. Plan to use them to puppet physics bodies with IK for my building game.

YT Video:
https://youtu.be/htMDZ0fXJWo

![001](https://user-images.githubusercontent.com/37253663/152513594-8664cd0a-4f48-4bde-9882-42eccecb7944.png)


<img width="1226" alt="Screen Shot 2022-02-05 at 10 50 37 PM" src="https://user-images.githubusercontent.com/37253663/152646834-69c39674-d93a-4267-9cb5-08fce587f031.png">

Works in chains too:

![003](https://user-images.githubusercontent.com/37253663/152647270-7901cdda-60ce-4f41-856f-6728b5bf5b24.png)


I'm working on a new joint that would use basis transforms from a given node instead of using angle rotations. This way it doesn't need to calculate the target rotations so it's faster.

Regular Angle Rotation Input:

<img width="596" alt="Screen Shot 2022-02-05 at 11 09 12 PM" src="https://user-images.githubusercontent.com/37253663/152647458-35681e65-a7a5-4530-92af-de537540228e.png">

Basis Vector Transform from "Puppeteer" node:

<img width="1271" alt="Screen Shot 2022-02-05 at 11 16 41 PM" src="https://user-images.githubusercontent.com/37253663/152647647-212c9c96-079c-44ae-a55b-1c10fda87dd5.png">


![002](https://user-images.githubusercontent.com/37253663/152647516-0660b44c-a222-445b-ba05-b908a4e8131d.png)

![001](https://user-images.githubusercontent.com/37253663/152647513-cbbbf4fb-5bf6-48f2-81c6-4685e1ab87fc.png)

Thanks to:
  DMGregory: https://gamedev.stackexchange.com/questions/182850/rotate-rigidbody-to-face-away-from-camera-with-addtorque/182873#182873
	and
	The Step Event: https://youtu.be/vewwP8Od_7s
For the calculations

