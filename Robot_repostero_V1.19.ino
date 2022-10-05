#include <VarSpeedServo.h>
#include <serialGLCD.h> 
#include <PID_v1.h>

VarSpeedServo s_base;
VarSpeedServo s_hombro;
VarSpeedServo s_codo;
VarSpeedServo s_herr;
VarSpeedServo s_pinza;

serialGLCD lcd;

double base_Setpoint, base_Input, base_Output=90;//iniciamos a 90 para que el PID parta de una posición estable y no oscile al encenderse el robot
PID base_PID(&base_Input, &base_Output, &base_Setpoint,0,5,0, P_ON_M, DIRECT);

double hombro_Setpoint, hombro_Input, hombro_Output=180;//iniciamos a 180 para que el PID parta de una posición estable y no oscile al encenderse el robot
PID hombro_PID(&hombro_Input, &hombro_Output, &hombro_Setpoint,0,5,0, P_ON_M, DIRECT);

//Pines para leer los potenciometros
int pin_p_base=A0;
int pin_p_hombro=A1;
int pin_p_codo=A2;
int pin_p_herr=A3;
int pin_p_pinza=7;
//Pines para leer los potenciometros

//Pines realimentación
int pin_rea_base=A4;
int pin_rea_hombro=A5;
//Pines realimentación

//Pines para leer los botones
int pin_bot_A=7;
int pin_bot_B=6;
int pin_bot_C=5;
int pin_bot_D=4;
//Pines para leer los botones

//Tiempo de espera en el loop
int t_delay=100;
//Tiempo de espera en el loop

//Velocidad máxima
int V_MAX=15;

//Información pinza
bool aux_pinza_1=0;//Variables auxiliares
bool aux_pinza_2=0;//Variables auxiliares
bool pos_pinza=0;//0 cerrado, 1 abierto
int ang_pinza_0=65;//Ángulo cerrado
int ang_pinza_1=115;//Ángulo abierto
//Información pinza

//Vectores de datos
int ang_pot[4];
int estado_bot[4];
int ang_actual_robot[5];
int ang_i_robot[5];
int ang_f_robot[5];
float coord_actual_robot[5];//x,y,z,alfa,pinza
float coord_i_robot[5];
float coord_f_robot[5];
//Vectores de datos

//Estados de la máquina de estados
enum Estados{
  MENU,
  CM_ANG,
  CM_COORD,
  T_1,
  T_2,
  T_3,
  T_4,
  T_5,
  T_6
};
//Estados de la máquina de estados

//Variables para la máquina de estados
enum Estados estado; 
int seleccion=0;
int refrescar=0;
//Variables para la máquina de estados

void setup() {
  Serial.begin(115200);
  delay(1000);

  base_PID.SetMode(AUTOMATIC);
  base_PID.SetOutputLimits(0, 180); 
  hombro_PID.SetMode(AUTOMATIC);
  hombro_PID.SetOutputLimits(0, 180); 
  
  mover_d(0,90,-150,0,0,1.0f);
  s_base.attach(12);
  s_hombro.attach(11);
  s_codo.attach(10);
  s_herr.attach(9);
  s_pinza.attach(8);
  estado=MENU;
  lcd.backLight(100);
  lcd.clearLCD();
  mover_d(0,90,-150,0,0,1.0f);
}



void loop() {
  gestor();
  delay(t_delay);
}

//Coordinador del resto de funciones
void gestor(){
  if(estado==MENU){
      lcd.gotoLine(1);
      Serial.print("Control Manual dir");
      lcd.gotoLine(2);
      Serial.print("Control Manual inv");
      lcd.gotoLine(3);
      Serial.print("Extender");
      lcd.gotoLine(4);
      Serial.print("Esparcir lacasitos");
      lcd.gotoLine(5);
      Serial.print("Extender + esparcir");
      lcd.gotoLine(6);
      Serial.print("Demo 1");
      lcd.gotoLine(7);
      Serial.print("Demo 2");
      lcd.gotoLine(8);
      Serial.print("Demo 3");
      lcd.drawBox(0,seleccion*8,127,(seleccion+1)*8-1,1);
      while(leer_bot()==1){
        delay(20);
      }
      while(leer_bot()==0){
        delay(20);
      }
      if(estado_bot[0]==1){
        seleccion--;if(seleccion<0){seleccion=7;}
      }
      if(estado_bot[2]){
        seleccion++;if(seleccion>7){seleccion=0;}
      }
      if(estado_bot[1]==1){
        if(seleccion==0){estado=CM_ANG;asegurar_ang();}
        if(seleccion==1){estado=CM_COORD;asegurar_coord();}
        if(seleccion==2){estado=T_1;}
        if(seleccion==3){estado=T_2;}
        if(seleccion==4){estado=T_3;}
        if(seleccion==5){estado=T_4;}
        if(seleccion==6){estado=T_5;}
        if(seleccion==7){estado=T_6;}
      }
      lcd.clearLCD();
  }
  
  else if(estado==CM_ANG){
    control_manual();
    leer_bot();
    if(estado_bot[3]==1){estado=MENU; mover_d(0,90,-150,0,0,2.0f);lcd.clearLCD();}
    if(refrescar==10&&estado==CM_ANG){mostrar_datos();refrescar=0;}
    refrescar++;
  }
  else if(estado==CM_COORD){
    control_manual_inv();
    leer_bot();
    if(estado_bot[3]==1){estado=MENU; mover_d(0,90,-150,0,0,2.0f);lcd.clearLCD();}
    if(refrescar==10&&estado==CM_COORD){mostrar_datos();refrescar=0;}
    refrescar++;
  }
  else if(estado==T_1){//Tarta a unos 20 cms de la base
    lcd.clearLCD();
    lcd.gotoLine(4);
    Serial.print("Ejecutando...");
    lcd.gotoLine(5);
    Serial.print("Extender");
    //Posicionamos arriba
    mover_inv(0,20,20,0,0,1);
    mover_inv(0,25,30,0,0,1);
    //Pasada 1
    mover_inv(-9,30,15,-35,0,1);
    mover_inv(-9,15,15,-35,0,2);
    mover_inv(-6,25,20,0,0,0.5);
    //Pasada 2
    mover_inv(-6,30,15,-35,0,0.5);
    mover_inv(-6,15,15,-35,0,2);
    mover_inv(-3,25,20,0,0,0.5);
    //Pasada 3
    mover_inv(-3,30,15,-35,0,0.5);
    mover_inv(-3,15,15,-35,0,2);
    mover_inv(0,25,20,0,0,0.5);
    //Pasada 4
    mover_inv(0,30,15,-35,0,0.5);
    mover_inv(0,15,15,-35,0,2);
    mover_inv(3,25,20,0,0,0.5);
    //Pasada 5
    mover_inv(3,30,15,-35,0,0.5);
    mover_inv(3,15,15,-35,0,2);
    mover_inv(6,25,20,0,0,0.5);
    //Pasada 6
    mover_inv(6,30,15,-35,0,0.5);
    mover_inv(6,15,15,-35,0,2);
    mover_inv(9,25,20,0,0,0.5);
    //Pasada 7
    mover_inv(9,30,15,-35,0,0.5);
    mover_inv(9,15,15,-35,0,2);
    mover_inv(0,25,30,0,0,1);
    
    mover_d(0,90,-150,35,0,2);
    mover_d(0,90,-150,0,0,1);
    estado=MENU;
    lcd.clearLCD();
  }
  else if(estado==T_2){ 
    lcd.clearLCD();
    lcd.gotoLine(4);
    Serial.print("Ejecutando...");
    lcd.gotoLine(5);
    Serial.print("Esparcir lacasitos");
    mover_inv(0,20,20,0,0,1);
    mover_inv(0,25,30,0,0,1);
    mover_inv(16,20,20,91,1,2);
    mover_inv(16,20,13.5,91,0,2);
    mover_inv(16,20,30,91,0,1);
    mover_inv(0,35,30,91,1,2);
    delay(1000);
    mover_d(0,90,-150,35,0,2);
    mover_d(0,90,-150,0,0,1);
    estado=MENU;
    lcd.clearLCD();
  }
  else if(estado==T_3){
    lcd.clearLCD();
    lcd.gotoLine(4);
    Serial.print("Ejecutando...");
    lcd.gotoLine(5);
    Serial.print("Extender + Esparcir");
        //Posicionamos arriba
    mover_inv(0,20,20,0,0,1);
    mover_inv(0,25,30,0,0,1);
    //Pasada 1
    mover_inv(-9,30,15,-35,0,1);
    mover_inv(-9,15,15,-35,0,2);
    mover_inv(-6,25,20,0,0,0.5);
    //Pasada 2
    mover_inv(-6,30,15,-35,0,0.5);
    mover_inv(-6,15,15,-35,0,2);
    mover_inv(-3,25,20,0,0,0.5);
    //Pasada 3
    mover_inv(-3,30,15,-35,0,0.5);
    mover_inv(-3,15,15,-35,0,2);
    mover_inv(0,25,20,0,0,0.5);
    //Pasada 4
    mover_inv(0,30,15,-35,0,0.5);
    mover_inv(0,15,15,-35,0,2);
    mover_inv(3,25,20,0,0,0.5);
    //Pasada 5
    mover_inv(3,30,15,-35,0,0.5);
    mover_inv(3,15,15,-35,0,2);
    mover_inv(6,25,20,0,0,0.5);
    //Pasada 6
    mover_inv(6,30,15,-35,0,0.5);
    mover_inv(6,15,15,-35,0,2);
    mover_inv(9,25,20,0,0,0.5);
    //Pasada 7
    mover_inv(9,30,15,-35,0,0.5);
    mover_inv(9,15,15,-35,0,2);

    mover_inv(0,20,20,0,0,1);
    mover_inv(0,25,30,0,0,1);
    mover_inv(16,20,20,91,1,2);
    mover_inv(16,20,13.5,91,0,2);
    mover_inv(16,20,30,91,0,1);
    mover_inv(0,35,30,91,1,2);
    
    delay(1000);
    mover_d(0,90,-150,35,0,2);
    mover_d(0,90,-150,0,0,1);

    delay(1000);
    mover_d(0,90,-150,0,0,2.0f);
    estado=MENU;
    lcd.clearLCD();
  }
  else if(estado==T_4){
    lcd.clearLCD();
    lcd.gotoLine(4);
    Serial.print("Ejecutando...");
    lcd.gotoLine(5);
    Serial.print("Demo 1");

    mover_inv(0,20,20,0,0,1);
    mover_inv(0,25,30,0,0,1);
    mover_inv(0,25,5,0,0,2);
    mover_inv(0,25,30,0,0,2);
    mover_inv(0,21,20,-90,1,1);
    mover_inv(0,30,20,-90,1,1);
    mover_inv(0,21,20,-90,1,1);
    
    mover_d(0,90,-150,0,0,2);
    estado=MENU;
    lcd.clearLCD();
  }
  else if(estado==T_5){
    lcd.clearLCD();
    lcd.gotoLine(4);
    Serial.print("Ejecutando...");
    lcd.gotoLine(5);
    Serial.print("Demo 2");

    mover_d(-45,90,-150,0,0,1);
    mover_d(45,90,-150,0,0,2);
    mover_d(0,90,-150,0,0,1);
    mover_d(0,75,-150,0,0,1);
    mover_d(0,90,-150,0,0,1);
    mover_d(0,90,-60,0,0,2);
    mover_d(0,90,-150,0,0,2);
    mover_d(-45,70,-60,0,0,2);
    mover_d(0,90,-150,0,0,2);
    mover_d(45,70,-60,0,0,2);
    mover_d(0,90,-150,0,1,2);
    mover_d(0,90,-150,0,0,1);
    estado=MENU;
    lcd.clearLCD();
  }
  else if(estado==T_6){
    lcd.clearLCD();
    lcd.gotoLine(4);
    Serial.print("Ejecutando...");
    lcd.gotoLine(5);
    Serial.print("Demo 3");

    delay(1000);
    mover_d(0,90,-150,0,0,2.0f);
    estado=MENU;
    lcd.clearLCD();
  }
}
//Coordinador del resto de funciones

//Función para modificar los ángulós relativos de las articulaciones del robot con los potenciómetros
void control_manual(){
  s_base_write(map(analogRead(pin_p_base),0,1023,180,0));
  s_hombro_write(map(analogRead(pin_p_hombro),0,1023,0,180));
  s_codo.write(map(analogRead(pin_p_codo),0,1023,180,0),V_MAX);
  s_herr.write(map(analogRead(pin_p_herr),0,1023,180,0),V_MAX);

  aux_pinza_1=digitalRead(pin_bot_A);
  if(aux_pinza_1==1&&aux_pinza_2==0){
    if(pos_pinza==0){pos_pinza=1;}
    else if(pos_pinza==1){pos_pinza=0;}
    if(pos_pinza==0){s_pinza.write(ang_pinza_0);}
    if(pos_pinza==1){s_pinza.write(ang_pinza_1);}
  }
  aux_pinza_2=aux_pinza_1;
  }
//Función para modificar los ángulós relativos de las articulaciones del robot con los potenciómetros


//Función para modificar la coordenada espacial del extremo del robot con los potenciómetros
void control_manual_inv(){
  float x,y,z,alfa,aux;
  int q0,q1,q2,q3;
  x=mapfloat(analogRead(pin_p_base),0,1023,-20,20);
  y=mapfloat(analogRead(pin_p_hombro),0,1023,5,31);
  z=mapfloat(analogRead(pin_p_codo),0,1023,10,30);
  alfa=mapfloat(analogRead(pin_p_herr),0,1023,-90,120);
  q0=57.296*atan2(x,y);
  aux=(x*x+y*y+(z-12)*(z-12)-1017)/1008;
  q2=57.296*atan2(-sqrt(1-aux*aux),aux);
  q1=57.296*(atan2((z-12),(sqrt(x*x+y*y)))-atan2(21*sin(q2*(0.017453)),24+21*aux));
  q3=alfa-q1-q2;
  s_base_write(map(q0,90,-90,0,180));
  s_hombro_write(map(q1,0,90,0,180));
  s_codo.write(map(q2,0,-180,0,180),V_MAX);
  s_herr.write(map(q3,-90,90,170,45),V_MAX);

  aux_pinza_1=digitalRead(pin_bot_A);
  if(aux_pinza_1==1&&aux_pinza_2==0){
    if(pos_pinza==0){pos_pinza=1;}
    else if(pos_pinza==1){pos_pinza=0;}
    if(pos_pinza==0){s_pinza.write(ang_pinza_0);}
    if(pos_pinza==1){s_pinza.write(ang_pinza_1);}
  }
  aux_pinza_2=aux_pinza_1;
}
//Función para modificar la coordenada espacial del extremo del robot con los potenciómetros

int leer_bot(){
  estado_bot[0]=digitalRead(pin_bot_A);
  estado_bot[1]=digitalRead(pin_bot_B);
  estado_bot[2]=digitalRead(pin_bot_C);
  estado_bot[3]=digitalRead(pin_bot_D);
  for(int i=0;i<4;i++){
    if(estado_bot[i]==1){return 1;}
  }
  return 0;
}


//Función para mover el robot empleando ángulos relativos (cambia la posición cada 20ms que es lo óptimo para un servo)
//el parámetro t indica el tiempo que emplea en desplazarse hasta ese punto, para tener un control de la velocidad de movimiento
void mover_d(int a0,int a1, int a2, int a3, int a4, float t){
  leer_ang_actual();
  for(int i=0;i<5;i++){
    ang_i_robot[i]=ang_actual_robot[i];
  }
  ang_f_robot[0]=a0;
  ang_f_robot[1]=a1;
  ang_f_robot[2]=a2;
  ang_f_robot[3]=a3;
  ang_f_robot[4]=a4;
  float n_pasos=t/0.02;
  float dif_ang[4];
  for(int i=0;i<4;i++){
    dif_ang[i]=(ang_f_robot[i]-ang_i_robot[i])/n_pasos;
  }
  for(int i=0;i<=n_pasos;i++){
    s_base_write(map(ang_i_robot[0]+i*dif_ang[0],90,-90,0,180));
    s_hombro_write(map(ang_i_robot[1]+i*dif_ang[1],0,90,0,180));
    s_codo.write(map(ang_i_robot[2]+i*dif_ang[2],0,-180,0,180));
    s_herr.write(map(ang_i_robot[3]+i*dif_ang[3],-90,90,170,45));
    delay(20);
  }
  if(ang_f_robot[4]==1){s_pinza.write(ang_pinza_1);}
  else{s_pinza.write(ang_pinza_0);}
}
//Función para mover el robot empleando ángulos (cambia la posición cada 20ms que es lo óptimo para un servo)
//el parámetro t indica el tiempo que emplea en desplazarse hasta ese punto, para tener un control de la velocidad de movimiento

//Función para mover el robot empleando coordenadas (cambia la posición cada 20ms que es lo óptimo para un servo)
//el parámetro t indica el tiempo que emplea en desplazarse hasta ese punto, para tener un control de la velocidad de movimiento
void mover_inv(float x,float y, float z, float alfa, float p, float t){
  int q0,q1,q2,q3;
  float aux;
  leer_coord_actual();
  for(int i=0;i<5;i++){
    coord_i_robot[i]=coord_actual_robot[i];
  }
  coord_f_robot[0]=x;
  coord_f_robot[1]=y;
  coord_f_robot[2]=z;
  coord_f_robot[3]=alfa;
  coord_f_robot[4]=p;
  float n_pasos=t/0.02;
  float dif_coord[4];
  for(int i=0;i<4;i++){
    dif_coord[i]=(coord_f_robot[i]-coord_i_robot[i])/n_pasos;
  }
  for(int i=0;i<=n_pasos;i++){
    
    float x_=coord_i_robot[0]+i*dif_coord[0];
    float y_=coord_i_robot[1]+i*dif_coord[1];
    float z_=coord_i_robot[2]+i*dif_coord[2];
    float alfa_=coord_i_robot[3]+i*dif_coord[3];
    
    q0=57.296*atan2(x_,y_);
    aux=(x_*x_+y_*y_+(z_-12)*(z_-12)-1017)/1008;
    q2=57.296*atan2(-sqrt(1-aux*aux),aux);
    q1=57.296*(atan2((z_-12),(sqrt(x_*x_+y_*y_)))-atan2(21*sin(q2*(0.017453)),24+21*aux));
    q3=alfa_-q1-q2;
    
    s_base_write(map(q0,90,-90,0,180));
    s_hombro_write(map(q1,0,90,0,180));
    s_codo.write(map(q2,0,-180,0,180));
    s_herr.write(map(q3,-90,90,170,45));
    
    delay(20);
  }
  if(p>0.5){s_pinza.write(ang_pinza_1);}
  else{s_pinza.write(ang_pinza_0);}
}
//Función para mover el robot empleando coordenadas (cambia la posición cada 20ms que es lo óptimo para un servo)
//el parámetro t indica el tiempo que emplea en desplazarse hasta ese punto, para tener un control de la velocidad de movimiento

//Función para leer las cordenadas del extremo del último eslabón empleando cinemática directa
void leer_coord_actual(){
  float aux=0;
  leer_ang_actual();
  float rad[4];
  for(int i=0;i<4;i++){
    rad[i]=ang_actual_robot[i]*((6.283)/360.0);
  }
  coord_actual_robot[0]=sin(rad[0])*(24*cos(rad[1])+21*cos(rad[1]+rad[2]));
  coord_actual_robot[1]=cos(rad[0])*(24*cos(rad[1])+21*cos(rad[1]+rad[2]));
  coord_actual_robot[2]=12+24*sin(rad[1])+21*sin(rad[1]+rad[2]);
  coord_actual_robot[3]=(rad[1]+rad[2]+rad[3])*((360.0)/(6.283));
  if(s_pinza.read()>90){aux=1;}
  coord_actual_robot[4]=aux;
}
//Función para leer las cordenadas del extremo del último eslabón empleando cinemática directa

//Función para leer los ángulos relativos del extremo del último eslabón
void leer_ang_actual(){
  int aux=0;
  ang_actual_robot[0]=map(analogRead(pin_rea_base),40,600,90,-90);
  ang_actual_robot[1]=map(analogRead(pin_rea_hombro),40,600,0,90);
  ang_actual_robot[2]=map(s_codo.read(),0,180,0,-180);
  ang_actual_robot[3]=map(s_herr.read(),170,45,-90,90);
  if(s_pinza.read()>90){aux=1;}
  ang_actual_robot[4]=aux;
  }
//Función para leer los ángulos relativos del extremo del último eslabón

//Función para mapear en float
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//Función para mapear en float

//Función para mostrar los ángulos relativos del robot, sus coordenadas y la posición actual de la pinza
void mostrar_datos(){
  lcd.clearLCD();
  leer_coord_actual();
  lcd.gotoLine(1);
  Serial.print("   Info. posicion");
  lcd.gotoLine(3);
  Serial.print("X:");Serial.print(coord_actual_robot[0]);lcd.gotoPosition(48,16);Serial.print(" q1_base:");Serial.print(ang_actual_robot[0]);
  lcd.gotoLine(4);
  Serial.print("Y:");Serial.print(coord_actual_robot[1]);lcd.gotoPosition(48,24);Serial.print(" q2_homb:");Serial.print(ang_actual_robot[1]);
  lcd.gotoLine(5);
  Serial.print("Z:");Serial.print(coord_actual_robot[2]);lcd.gotoPosition(48,32);Serial.print(" q3_codo:");Serial.print(ang_actual_robot[2]);
  lcd.gotoLine(6);
  Serial.print("a:");Serial.print(coord_actual_robot[3]);lcd.gotoPosition(48,40);Serial.print(" q4_herr:");Serial.print(ang_actual_robot[3]);
  lcd.gotoLine(7);

  if(ang_actual_robot[4]==1){
  Serial.print("   Pinza abierta");
  }
  else{Serial.print("   Pinza cerrada");}
}
//Función para mostrar los ángulos relativos del robot, sus coordenadas y la posición actual de la pinza

//Función para colocar los potenciómetros en una posición segura antes de comenzar el control manual directo, para evitar movimientos descontrolados
void asegurar_ang(){
  lcd.clearLCD();
  int q0,q1,q2;
  do{
    q0=map(analogRead(pin_p_base),0,1023,90,-90);
    q1=map(analogRead(pin_p_hombro),0,1023,0,90);
    q2=map(analogRead(pin_p_codo),0,1023,-180,0);
    lcd.gotoLine(2);
    Serial.print("  Angulo peligroso");
    lcd.gotoLine(4);
    Serial.print("base:");Serial.print(q0);lcd.gotoPosition(58,24);Serial.print("(-15,15)");
    lcd.gotoLine(5);
    Serial.print("homb:");Serial.print(q1);lcd.gotoPosition(58,32);Serial.print("(80,90)");
    lcd.gotoLine(6);
    Serial.print("codo:");Serial.print(q2);lcd.gotoPosition(58,40);Serial.print("(-140,-150)");
    delay(200);
    lcd.clearLCD();
  }while(q0>15||q0<-15||q1>90||q1<80||q2>-140||q2<-150);
}
//Función para colocar los potenciómetros en una posición segura antes de comenzar el control manual directo, para evitar movimientos descontrolados

//Función para colocar los potenciómetros en una posición segura antes de comenzar el control manual inverso, para evitar movimientos descontrolados
void asegurar_coord(){
  lcd.clearLCD();
  int q0,q1,q2;
  do{
    q0=analogRead(pin_p_base);
    q1=analogRead(pin_p_hombro);
    q2=analogRead(pin_p_codo);
    lcd.gotoLine(2);
    Serial.print("  Angulo peligroso");
    lcd.gotoLine(4);
    Serial.print("A:");Serial.print(q0);lcd.gotoPosition(58,24);Serial.print("(400,600)");
    lcd.gotoLine(5);
    Serial.print("B:");Serial.print(q1);lcd.gotoPosition(58,32);Serial.print("(300,400)");
    lcd.gotoLine(6);
    Serial.print("C:");Serial.print(q2);lcd.gotoPosition(58,40);Serial.print("(300,500)");
    delay(200);
    lcd.clearLCD();
  }while(q0>600||q0<400||q1>400||q1<300||q2>500||q2<300);
}
//Función para colocar los potenciómetros en una posición segura antes de comenzar el control manual inverso, para evitar movimientos descontrolados

//Funciones para sustituir a servo.write(); emplea un PID para controlar la posición del motor
void s_base_write(int pos){
  base_Setpoint = pos;
  base_Input=map(analogRead(pin_rea_base),40,600,0,180);
  base_PID.Compute();
  s_base.write(base_Output);
}

void s_hombro_write(int pos){
  hombro_Setpoint = pos;
  hombro_Input=map(analogRead(pin_rea_hombro),40,600,0,180);
  hombro_PID.Compute();
  s_hombro.write(hombro_Output);
}
//Funciones para sustituir a servo.write(); emplea un PID para controlar la posición del motor
