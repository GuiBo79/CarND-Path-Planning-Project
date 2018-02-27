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
	
Here the code atributes boolean states for all the conditions described above. This boolean states will be used later to set the conditions of lane changes and GAP searching. Is considered as free lane when have no rear car closer than 12 meters and front car 30 meters. 








