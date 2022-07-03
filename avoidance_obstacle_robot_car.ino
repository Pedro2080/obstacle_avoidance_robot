#include <Servo.h>                                     //* Servo library 
  Servo myservo;                                        //* Servo object
  int Echo=A4;  
  int Trig=A5; 

  int engine_speed=255;                                //. Engine speed between 0 a 255
  float right_distance=0.0;                           //. Right distance of the robot
  float left_distance=0.0;                           //. Left distance of the robot
  float center_distance=0.0;                        //. Distance in front of the robot 
  float right_distance=0.0;                        //. Store the right distance
  float left_distance=0.0;                        //. Store the distance on the left

  #define ENA 5
  #define ENB 6
  #define IN1 7
  #define IN2 8
  #define IN3 9
  #define IN4 11 
  
  void setup()
  {                 
  myservo.attach(3);                        //. Pin where the servo is connected 
  Serial.begin(9600);                      //. Start serial communication
  // Ultrasonic sensor setup
  pinMode(Echo, INPUT);                   //. Pin Echo set to input 
  pinMode(Trig, OUTPUT);                 //. Pin Trig set to output 
// Setting motor pins as output
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  stop();
  } 
  void loop()
  { 
    forward();
    center_distance=measure_distance();                 //. Measuring the Distance in CM , we call the measure distance() function to measure the distance to the object.
                 
    if(center_distance<=20)                           //. Checks If there is an obstacle found less than 20 cm in front of the robot.
    {                        
      stop();                                       //. Robot stops
      backward();                                  //. Robot goes back
      delay(500);                      
      stop();                                     //. Robot stops
      right_distance=look_right();               //. We call the function to turn and look at the servo on the right
      delay(500);                                    
      left_distance=look_left();               //. We call the function to turn and look at the servo on the left
      delay(500);

     if(right_distance>=left_distance) 
     {
       right();                            //. Robot turns right
      } 
    else
    { 
        left();                         //. Robot turns left
      }
    }
    else 
    {
        forward();                       //. Robot goes forward
    }   
  }
  
  void forward()                          //* Function for the robot toi move forward
  {              
  analogWrite(ENA, velocidadeMotorres);
  analogWrite(ENB, velocidadeMotorres);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Robot goes forward");
  }

  void right()                                //* Function for the robot to turn right
  {              
  analogWrite(ENA, velocidadeMotorres);
  analogWrite(ENB, velocidadeMotorres);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Robot turns right");  
  }

  void left()                               //* Function for the robot to turn left
  {             
  analogWrite(ENA, velocidadeMotorres);
  analogWrite(ENB, velocidadeMotorres);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
  Serial.println("Robot turns left");
  }
  
  void stop() {                           //* Function for the robot to stop
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Robot stop!");
  } 

  void backward()                        //* Function for the robot to move backward
  {       
  analogWrite(ENA, velocidadeMotorres);
  analogWrite(ENB, velocidadeMotorres);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("robot goes back");
  }

  float measure_distance()                                                             //* Function to measure the distance from the robot to the object
  {                            
  float duration=0.0;                                                                 //.  Variable for the duration of the sound wave travel
  float distance=0.0;
  digitalWrite(Trig,LOW);
  delayMicroseconds(2);                                                              //. Trigger 2 ms OFF then 20 ms On
  digitalWrite(Trig,HIGH);
  delayMicroseconds(20);                                                            //. 20ms ON
  digitalWrite(Trig,LOW);
  duration=pulseIn(Echo,HIGH);                                                      //. Reads echoPin and returns the sound's wave travel time in microseconds
  // distance= t* v   t= time in s, v= speed of the sound =344m/s = 0.0344 cm/uS
  distance=duration*0.034/2;                                                     //.calculating distance in cm, speed of sound wave divided by 2 (go and come back)
  Serial.print("Distance");                                                             //. Displays distance on Serial Monitor
  Serial.println(distance);    
  Serial.print(" cm ");

  return distance;                                                                    //. Returns the distance
  }

  float look_right()                                                                //* Function for the robot to look to the right
  {             
    myservo.write(0);                                                             //. Servo rotates 90 degrees to the right,
    delay(1000) ;                                                                //. Wait for 1s
    right_distance_robot=measure_distance();   
    myservo.write(90);                                                         // Servo rotates to starting position
    delay(200);                                                               //Wait for 200ms

    return right_distance_robot;                                             //. Returns the distance of the obstacle to the right
  }

  float look_left()                                                       //* Function for the robot to look to the Left
  {    
    myservo.write(180);                                                 //. Servo rotates 90 degrees to the left
    delay(500);                                                        //. Wait 500 ms
    left_distance_robot=measure_distance();
    myservo.write(90);                                               //. Servo rotates 90 degrees to the left
    delay(500);                                                     //. Wait 500ms

    return left_distance_robot;                                    //. Returns the distance of the obstacle to the Left
  }