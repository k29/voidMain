int flag=0;
int r_inp,theta_inp;
double vmax=255;
double r=0;
int theta=0;
double time,time_angle;


long enable_front, enable_back, enable_right, enable_left;
int inp1_front, inp2_front, inp1_back, inp2_back, inp1_right, inp2_right, inp1_left, inp2_left;

int en_pin_front=11,en_pin_back=10, en_pin_right=6, en_pin_left=5;
int in_pin1_front=13,in_pin2_front=12,in_pin1_back=9,in_pin2_back=8,in_pin1_right=7,in_pin2_right=4,in_pin1_left=3,in_pin2_left=2;

int index,lastindex;

String readString;
char c;

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(en_pin_front,OUTPUT);
  pinMode(en_pin_back,OUTPUT);
  pinMode(en_pin_right,OUTPUT);
  pinMode(en_pin_left,OUTPUT);
  pinMode(in_pin1_front,OUTPUT);
  pinMode(in_pin2_front,OUTPUT);
  pinMode(in_pin1_back,OUTPUT);
  pinMode(in_pin2_back,OUTPUT);
  pinMode(in_pin1_right,OUTPUT);
  pinMode(in_pin2_right,OUTPUT);
  pinMode(in_pin1_left,OUTPUT);
  pinMode(in_pin2_left,OUTPUT);
}

void loop() 
{
  // put your main code here, to run repeatedly: 
  r=0,theta=0;
  r_inp=0,theta_inp=0;
  input_var();
  
  
  if(r!=0&&(r_inp!=9999&&theta_inp!=9999))
  {
    
    
    time=abs(r)/vmax;
    Serial.end();
    implement_var_angle();
    
    analogWrite(en_pin_front, enable_front);
    analogWrite(en_pin_back, enable_back);
    analogWrite(en_pin_right, enable_right);
    analogWrite(en_pin_left, enable_left);
    
    digitalWrite(in_pin1_front,inp1_front);
    digitalWrite(in_pin2_front,inp2_front);
    
    digitalWrite(in_pin1_back,inp1_back);  
    digitalWrite(in_pin2_back,inp2_back);
    
    digitalWrite(in_pin1_right,inp1_right);
    digitalWrite(in_pin2_right,inp2_right);
    
    digitalWrite(in_pin1_left,inp1_left);
    digitalWrite(in_pin2_left,inp2_left);
    
    /*Serial.println("enablefront");Serial.println(enable_front);
    Serial.println("enable back");Serial.println(enable_back);
    Serial.println("enable right");Serial.println(enable_right);
    Serial.println("enable left");Serial.println(enable_left);
    Serial.println("inp1front");Serial.println(inp1_front);
    Serial.println("inp2front");Serial.println(inp2_front);
    Serial.println("inp1right");Serial.println(inp1_right);
    Serial.println("inp2right");Serial.println(inp2_right);
    Serial.println("inp1back");Serial.println(inp1_back);
    Serial.println("inp2back");Serial.println(inp2_back);
    Serial.println("inp1left");Serial.println(inp1_left);
    Serial.println("inp2left");Serial.println(inp2_left);*/
    
    if(theta>0)
      {delay(1.75*abs(theta*80/9));}
    else
      {delay(1.75*abs(theta*80/9));}
      
    /* analogWrite(en_pin_front, 0);
    analogWrite(en_pin_back, 0);
    analogWrite(en_pin_right, 0);
    analogWrite(en_pin_left, 0);
    delay(500);*/
    
    /*Serial.println("enablefront");Serial.println(enable_front);
    Serial.println("enable back");Serial.println(enable_back);
    Serial.println("enable right");Serial.println(enable_right);
    Serial.println("enable left");Serial.println(enable_left);
    Serial.println("inp1front");Serial.println(inp1_front);
    Serial.println("inp2front");Serial.println(inp2_front);
    Serial.println("inp1right");Serial.println(inp1_right);
    Serial.println("inp2right");Serial.println(inp2_right);
    Serial.println("inp1back");Serial.println(inp1_back);
    Serial.println("inp2back");Serial.println(inp2_back);
    Serial.println("inp1left");Serial.println(inp1_left);
    Serial.println("inp2left");Serial.println(inp2_left);*/
    implement_var_radius();
    analogWrite(en_pin_front, enable_front);
    analogWrite(en_pin_back, enable_back);
    analogWrite(en_pin_right, enable_right);
    analogWrite(en_pin_left, enable_left);
    
    digitalWrite(in_pin1_front,inp1_front);
    digitalWrite(in_pin2_front,inp2_front);
    
    digitalWrite(in_pin1_back,inp1_back);  
    digitalWrite(in_pin2_back,inp2_back);
    
    digitalWrite(in_pin1_right,inp1_right);
    digitalWrite(in_pin2_right,inp2_right);
    
    digitalWrite(in_pin1_left,inp1_left);
    digitalWrite(in_pin2_left,inp2_left);
    delay(time*1000*1.0);
    
    flag=1;
    Serial.begin(9600);
    
    Serial.write(flag);
    
    Serial.print("The value of flag is   ");
    Serial.println(flag);
    
    
    
    
    enable_front=0,enable_back=0, enable_right=0, enable_left=0;
    
    
    r=0;
    analogWrite(en_pin_front, enable_front);
    analogWrite(en_pin_back, enable_back);
    analogWrite(en_pin_right, enable_right);
    analogWrite(en_pin_left, enable_left);
  }
  else if(r_inp==9999&&theta_inp==9999)
  {
    Serial.println("implement rotate before delay");
    Serial.end();
    implement_rotate();
    analogWrite(en_pin_front, enable_front);
    analogWrite(en_pin_back, enable_back);
    analogWrite(en_pin_right, enable_right);
    analogWrite(en_pin_left, enable_left);
    
    digitalWrite(in_pin1_front,inp1_front);
    digitalWrite(in_pin2_front,inp2_front);
    
    digitalWrite(in_pin1_back,inp1_back);  
    digitalWrite(in_pin2_back,inp2_back);
    
    digitalWrite(in_pin1_right,inp1_right);
    digitalWrite(in_pin2_right,inp2_right);
    
    digitalWrite(in_pin1_left,inp1_left);
    digitalWrite(in_pin2_left,inp2_left);
    delay(250);
    flag=1;
    Serial.begin(9600);
    Serial.println("implement rotate after delay");
    Serial.print("The value of flag is   ");
    Serial.println(flag);
    
    
    
    enable_front=0,enable_back=0, enable_right=0, enable_left=0;
    
    
    r=0;
    analogWrite(en_pin_front, enable_front);
    analogWrite(en_pin_back, enable_back);
    analogWrite(en_pin_right, enable_right);
    analogWrite(en_pin_left, enable_left);
    
  }
  else
  {
    analogWrite(en_pin_front, 0);
    analogWrite(en_pin_back, 0);
    analogWrite(en_pin_right,0);
    analogWrite(en_pin_left,0);
  }
  
    
  enable_front=0,enable_back=0, enable_right=0, enable_left=0;
  readString="";
}

void input_var()
{
  
  if(Serial.available()>0)
  {
    
    readString="";
    
    Serial.println("Enter angle from right (degrees)");
    
    
    while(Serial.available())
    {
      c=Serial.read();
      
      readString += c;
      delay(2);
    }
    
    if(readString.length()>0)
    {
      index=readString.indexOf(' ');
      lastindex=readString.indexOf(' ');
      r=(readString.substring(0,index)).toInt();
      
      theta=(readString.substring(index+1,readString.length())).toInt();
      
        Serial.print("Radius recieved by serial monitor   ");
        Serial.println(r);
        Serial.print("Angle recieved by serial monitor   ");
        Serial.println(theta);
        r_inp=r;
        theta_inp=theta;
       r=(r+2.5)/0.145;    // in centimetres
      
      flag=0;
      
      Serial.print("The value of flag is   ");
      Serial.println(flag);
      
    }
    
  }
  
  
  
}

void implement_var_angle()
{
  
  if(theta>0)
  {
    enable_front=220;
    enable_back=220;
    enable_right=240;
    enable_left=255;
    
    inp1_left=1;
    inp2_left=0;
    
    inp1_right=0;
    inp2_right=1;
    
    inp1_front=1;
    inp2_front=0;
    
    inp1_back=1;
    inp2_back=0;
    
  }  
  else
  {
    
    enable_front=220;
    enable_back=220;
    enable_right=255;
    enable_left=240;
    
    inp1_left=1;
    inp2_left=0;
    
    inp1_right=0;
    inp2_right=1  ;
    
    inp1_front=0;
    inp2_front=1;
    
    inp1_back=0;
    inp2_back=1;
    
  }
  
  
}

void implement_var_radius()
{
  if(r>0)
  {
    enable_right=vmax;
    enable_left=vmax;
    enable_front=0;
    enable_back=0;
    
    
    inp1_left=1;
    inp2_left=0;
    
    
    inp1_right=0;
    inp2_right=1;
   }
   else
   {
    
    enable_right=vmax;
    enable_left=vmax;
    enable_front=0;
    enable_back=0;
    
    
    inp1_left=0;
    inp2_left=1;
    
    
    inp1_right=1;
    inp2_right=0;
     
     
   }
  
  
}

void implement_rotate()
{
  
  enable_front=255;
  enable_back=255;
  enable_right=255;
  enable_left=255;
  
  inp1_left=1;
  inp2_left=0;
  
  inp1_right=1;
  inp2_right=0;
  
  inp1_front=1;
  inp2_front=0;
  
  inp1_back=1;
  inp2_back=0;
  
}
