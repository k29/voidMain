#define pi acos(-1)

double vmax=255;
double r=0;
int theta=0;
double time;
int time1=millis();
int time2=millis();
int enable_front, enable_back, enable_right, enable_left;
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
  input_var();
  //implement_var();
  if(r!=0)
  {
    if(theta!=90&&theta!=270)
    {
      time=r*abs(cos(theta*3.14/180))/vmax;
    }
    else
    {
      time=r/vmax;
    }
    implement_var();
    
    
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
    
    
    delay(time*1000);
    Serial.print("R is :  ");
    Serial.println(r);
    Serial.print("Theta is :  "); 
    Serial.println(theta);
    Serial.print("Vmax is :  ");
    Serial.println(vmax);
    
    Serial.print("Enable front is:");Serial.println(enable_front);
    Serial.print("Enable back is:");Serial.println(enable_back);
    Serial.print("Enable right is:");Serial.println(enable_right);
    Serial.print("Enable left is:");Serial.println(enable_left);
    
    Serial.print("pin1_front is:");Serial.println(inp1_front);
    Serial.print("pin2_front is:");Serial.println(inp2_front);
    Serial.print("pin1_back is:");Serial.println(inp1_back);
    Serial.print("pin2_back is:");Serial.println(inp2_back);
    Serial.print("pin1_right is:");Serial.println(inp1_right);
    Serial.print("pin2_right is:");Serial.println(inp2_right);
    Serial.print("pin1_left is:");Serial.println(inp1_left);
    Serial.print("pin2_left is:");Serial.println(inp2_left);
    Serial.println(time);
    enable_front=0,enable_back=0, enable_right=0, enable_left=0;
    
    //Serial.flush();
    r=0;
    analogWrite(en_pin_front, enable_front);
    analogWrite(en_pin_back, enable_back);
    analogWrite(en_pin_right, enable_right);
    analogWrite(en_pin_left, enable_left);
  }
  else
  {
    enable_front=0,enable_back=0, enable_right=0, enable_left=0;
  }
    
  enable_front=0,enable_back=0, enable_right=0, enable_left=0;
  readString="";
}

void input_var()
{
  // Get input from the user of vmax, r, theta:
  /*if(Serial.available()>0)
  {
    readString="";
    
    Serial.println("Enter distance to be covered (cm): ");
    
    while(Serial.available())
    {
      c=Serial.read();
      readString += c;
      delay(2);
    }
    if(readString.length()>0)
    {
      r = readString.toInt();
    }
    
    
  }*/
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
      //(readString.substring(index+1,lastindex)).toInt();
      theta=(readString.substring(index+1,readString.length())).toInt();
      if(theta>=0&&theta<=180)
        {theta+=180;}
      else
        {theta=theta-180;}
        
       r=(r+2.5)/0.145;
      
      Serial.println(readString);
      Serial.println(index);
      Serial.println(lastindex);
    }
    
  }/*
  if(Serial.available()>0)
 { 
    readString="";
    
    Serial.println("Enter the speed of approach(0 to 255)");
    
    while(Serial.available())
    {
      c=Serial.read();
      readString += c;
      delay(2);
    }
    if(readString.length()>0)
    {
      vmax = readString.toInt();
    }
    
  }
  */
  
}

void implement_var()// Assuming inp1____ =1, inp2____ =0  gives clockwise motion
{                   // Viewing from motor to wheels
  if(theta<=45&&theta>=0)  // Ist Quad
  {
    enable_front=vmax;
    enable_back=vmax;
    enable_right=(vmax*tan(theta*3.14/180));
    enable_left=(vmax*tan(theta*3.14/180));
    inp1_front=1;
    inp2_front=0;
    inp1_back=0;
    inp2_back=1;
    inp1_right=0;
    inp2_right=1;
    inp1_left=1;
    inp2_left=0;
  }
  else
  if(theta>45&&theta<=90)  // Ist Quad
  {
    enable_right=vmax;
    enable_left=vmax;
    enable_front=(vmax*tan((90-theta)*3.14/180));
    enable_back=(vmax*tan((90-theta)*3.14/180));
    inp1_front=1;
    inp2_front=0;
    inp1_back=0;
    inp2_back=1;
    inp1_right=0;
    inp2_right=1;
    inp1_left=1;
    inp2_left=0;
  }
  else
  if(theta>90&&theta<=135)  // IInd Quad
  {
    enable_right=vmax;
    enable_left=vmax;
    enable_front=(vmax*tan((theta-90)*3.14/180));
    enable_back=(vmax*tan((theta-90)*3.14/180));
    inp1_front=0;
    inp2_front=1;
    inp1_back=1;
    inp2_back=0;
    inp1_right=0;
    inp2_right=1;
    inp1_left=1;
    inp2_left=0;
  }
  else
  if(theta>135&&theta<=180)  // IInd Quad
  {
    enable_front=vmax;
    enable_back=vmax;
    enable_right=(vmax*tan((180-theta)*3.14/180));
    enable_left=(vmax*tan((180-theta)*3.14/180));
    inp1_front=0;
    inp2_front=1;
    inp1_back=1;
    inp2_back=0;
    inp1_right=0;
    inp2_right=1;
    inp1_left=1;
    inp2_left=0;
  }
  else
  if(theta>180&&theta<=225)  //  IIIrd Quad
  {
    enable_front=vmax;
    enable_back=vmax;
    enable_right=(vmax*tan((theta-180)*3.14/180));
    enable_left =(vmax*tan((theta-180)*3.14/180));
    inp1_front=0;
    inp2_front=1;
    inp1_back=1;
    inp2_back=0;
    inp1_right=1;
    inp2_right=0;
    inp1_left=0;
    inp2_left=1;
  }
  else
  if(theta>225&&theta<=270)  //  IIIrd Quad
  {
    enable_right=vmax;
    enable_left=vmax;
    enable_front=(vmax*tan((270-theta)*3.14/180));
    enable_back=(vmax*tan((270-theta)*3.14/180));
    inp1_front=0;
    inp2_front=1;
    inp1_back=1;
    inp2_back=0;
    inp1_right=1;
    inp2_right=0;
    inp1_left=0;
    inp2_left=1;
  }
  else
  if(theta>270&&theta<=315)  //  IVth Quad
  {
    enable_right=vmax;
    enable_left=vmax;
    enable_front=(vmax*tan((theta-270)*3.14/180));
    enable_back=(vmax*tan((theta-270)*3.14/180));
    inp1_front=1;
    inp2_front=0;
    inp1_back=0;
    inp2_back=1;
    inp1_right=1;
    inp2_right=0;
    inp1_left=0;
    inp2_left=1;
  }
  else
  if(theta>315&&theta<=360)  //  IVth Quad
  {
    enable_front=vmax;
    enable_back=vmax;
    enable_right=(vmax*tan((360-theta)*3.14/180));
    enable_left=(vmax*tan((360-theta)*3.14/180));
    inp1_front=1;
    inp2_front=0;
    inp1_back=0;
    inp2_back=1;
    inp1_right=1;
    inp2_right=0;
    inp1_left=0;
    inp2_left=1;
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
  }
}
