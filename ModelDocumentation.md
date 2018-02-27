# UDACITY - Self Driving Car Engineer - Path Planner Project
#### This writeup contains all steps as well the approach to implement a path planner to drive a car in the 3th term simulator.


## 1. Development Environment:

  -> MacBook Pro i7 - 8Gb
  
  -> MacOs High Sierra
  
  -> Xcode IDE
  
## 2. Files:

  -> main.cpp: contains the solution for the path planner.
  
  -> writeup.md: contains and explanation about how the solution was implemented.
  
## 3. The Code:

### First Challenge, keep the car in the road and generate smooth trajectories:

In the very beginning of this project was fundamental Aaron and David lesson explaining the main code, simulator functionalities and the use of spline to generate smooth trajectories. I followed the lesson implementing Aaron solution with some small changes to have an smoother lane change as follow:

	  vector<double> next_wp0 = getXY(car_s+40,2+4*lane,map_waypoints_s,map_waypoints_x,map_waypoints_y);
     vector<double> next_wp1 = getXY(car_s+80,2+4*lane,map_waypoints_s,map_waypoints_x,map_waypoints_y);
     vector<double> next_wp2 = getXY(car_s+120,2+4*lane,map_waypoints_s,map_waypoints_x,map_waypoints_y);

Instead of using an space of 30 meters I used 40 meters.

### Velocity Control and anti-colision:

To control the car velocity and to avoid the car to collide with the front car was implemented the following code:

            int cl_counter = 0; //Count closer than 30m sensor fusion data
            double front_car_vel; // Front Car Velocity
            bool front_left = false;
            bool rear_left = false;
            bool front_right = false;
            bool rear_right = false;
            
            
            //find ref_v and anti-colision code
            for (int i=0; i < sensor_fusion.size(); i++){
                
                double id = sensor_fusion[i][0];
                double x = sensor_fusion[i][1];
                double y = sensor_fusion[i][2];
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double s = sensor_fusion[i][5];
                double d = sensor_fusion[i][6];
                
                double car_dist = distance(x,y,car_x,car_y);
                front_car_vel = sqrt((vx*vx+vy*vy));

                s += ((double)prev_size*0.02*front_car_vel);
                
                
                if (d < (2+4*lane+2) && d > (2+4*lane-2)){ // Check if there is a car in the same lane
                    
                    if (s > car_s && s - car_s <=30){ //Check if the car is in front of EGO
                    
                        
                            std::cout << "Front ID: " << id << std::endl;
                            std::cout << "Distance S: " << s - car_s << std::endl;
                            std::cout << "Distance XY: " << car_dist << std::endl;
                            std::cout << "Front Car Velocity: " << front_car_vel*2.24 << std::endl;
                            std::cout << "Reference Vel: " << ref_vel << std::endl;
                            std::cout << "Car Speed: " << car_speed << std::endl;
                            
                            cl_counter++;
                
                            if(ref_vel > front_car_vel*2.24){
                                ref_vel -= 17.92/(s-car_s);
                            }
                        
                    }//End if S > car_s
                    
                }//End if is in the same lane
                
Here we have the cl_counter variable. This variable counts how many cars are in a space smaller than 30 meters and the line "ref_vel -= 17.92/(s-car_s)" 
controls the actuation of the breaks to prevent a collision. As closer the front car is stronger will be the breaks action. 
In regular driving conditions with no possibility of changing lane the car will follow the front car keeping a safe distance with the same velocity. More details will be explained later.

### Sensor Fusion and driving conditions detection: 

This part of the code takes the information from sensor fusion and traces the driving scenario and how is the environment around EGO car. Here are covered all the possibilities of the distribution of the car in the road, in all lanes as follow (from line 292 to 381).

	Front car.
	Front right  car.
	Rear right car.
	Front left car.
	Rear left car. 
	
Here the code atributes boolean states for all the conditions described above. This boolean states will be used later to set the conditions of lane changes and GAP searching. Is considered as free lane when have no rear car closer than 12 meters and front car within 30 meters. 

### Lane change states and GAP searching: 

Here is the core of this project, where according the cars distribution around EGO will determine what action the car must perform. 

The first and the simplest state is if have no car in front of EGO (cl_counter == 0). In this case the car will accelerates until limit velocity and will not perform lane change. 

	if (cl_counter == 0)  { //Increase velocity if no car is closer than 30m
                if (ref_vel <= 49.50) ref_vel += 0.448 ;
		
For the next states I will explain using as example EGO in lane 1, because for lane 0 was just excluded the possibility for change to left lane and for lane 2 was excluded the possibility to change to right.

Considering EGO in lane 1 , front car is slower than 95% of speed limit and left lane is free the car will change to lane 0, but if lane 0 is not free and lane 2 is free the car will change to lane 2.

	if (lane == 1 && front_car_vel<0.95*ref_vel && !front_left && !rear_left){
                    lane = 0;
                } else if (lane == 1 && front_car_vel<0.95*ref_vel && !front_right && !rear_right){
                    lane = 2;

Considering the two lanes are not free, but has no rear car (left or right) and has  front car (left or right) , EGO will start to slow down searching a GAP to change lane (lane 0 or lane 2, the first who become free).

	} else if (lane == 1 && front_car_vel<0.95*ref_vel && ((front_right && !rear_right) || (front_left && !rear_left))){
                    ref_vel -= 0.224;

In the opposite situation, if has no front car (left or right) , but has a rear car(left or right), EGO will start to accelerate until speed limit and prepares to change lane (lane 0 or lane 2, the first who becomes free). 

	} else if (lane == 1 && front_car_vel<0.95*ref_vel && ((!front_right && rear_right) || (!front_left && rear_left))){
                    if(ref_vel <= 49.5) ref_vel += 0.224;
                }
		












