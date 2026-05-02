/////////////SENSADO DE QTR8A////////////////////////////////////

const int pines_sensores[8] = {A7, A6, A5, A4, A3, A2, A1, A0};
int sensores[8];
int digital[8];
int lectura_fondo[8];
int lectura_linea[8];
int umbral[8]; 


/////////////////////////BOTONES Y LEDS//////////////////////////////////////

#define BOTOND   8 // Este es tu pin del YK04
#define BOTONB   2
#define LEDI     9
#define LEDD     12
///////////////////////MOTORES///////////////////////////////////////
#define pwmi     3   //PWM LEFT MOTOR
#define izq1     4   //LEFT1 MOTOR
#define izq2     5   //LEFT2 MOTOR 

#define pwmd     11  //PWM RIGHT MOTOR
#define der1     7   //RIGHT1 MOTOR 
#define der2     6   //RIGHT2 MOTOR 
/////////////////////////////////////////////////////////////////
long int sumap, suma, pos, poslast, position;
////////////////////////////////////////////////////////////////////////
////////////////////////////PID/////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
float KP=0.2;//constante proporcional
float KD=6;//constante derivativa
float KI=0.002;//constante integral
int vel=255;
int veladelante=200;//VELOCIDAD DEL FRENO DIRECCIÓN ADELANTE
int velatras=100;//VELOCIDAD DEL FRENO DIRECCIÓN ATRÁS

///datos para la integral///////////
int error1=0;
int error2=0;
int error3=0;
int error4=0;
int error5=0;
int error6=0;
///////////////////////////////////

///////////variable PID///////////////
int proporcional=0;
int integral=0;
int derivativo=0;
int diferencial=0;
int last_prop;
int setpoint=350;
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
void setup() {
  // Serial.begin(115200); // Comentado para ahorrar memoria y tiempo de CPU
  TCCR2B = TCCR2B & B11111000 | B00000011;
  pinMode(izq1,OUTPUT);
  pinMode(izq2,OUTPUT);
  pinMode(der1,OUTPUT);
  pinMode(der2,OUTPUT);
  pinMode(LEDI, OUTPUT);
  pinMode(LEDD, OUTPUT);
  pinMode(BOTOND, INPUT); 
  pinMode(BOTONB, INPUT); 
  digitalWrite(LEDI, 1);  

  /////////////////Calibración de Fondo///////////////////////////////
  // Serial.println("Esperando boton para calibrar FONDO...");
  while(!digitalRead(BOTOND)); 
  
  // Serial.println("Calibrando fondo...");
  for(int i = 0; i < 50; i++){
    fondos();
    digitalWrite(LEDI, 0); 
    delay(20);
    digitalWrite(LEDI, 1); 
    delay(20);
  }

  /////////////////////Calibración de Línea/////////////////////////////////
  // Serial.println("Esperando boton para calibrar LINEA...");
  while(!digitalRead(BOTOND)); 
  
  // Serial.println("Calibrando linea...");
  for(int i = 0; i < 50; i++){
    lineas();
    digitalWrite(LEDI, 0); 
    delay(20);
    digitalWrite(LEDI, 1); 
    delay(20);
  }

  ////////////////////////Cálculo de Promedios/////////////////////////////
  // Serial.println("Calculando promedios (Umbrales)...");
  promedio(); 

// --- 4. Arranque ---
  // 1. Espera a que PRESIONES el botón del control
  while(!digitalRead(BOTONB)); 
  
  // 2. Espera a que SUELTES el botón del control (para evitar el auto-freno)
  while(digitalRead(BOTONB)); 
  
  // 3. Un pequeño delay de seguridad de medio segundo para que prepares la vista
  delay(500); 
  
  digitalWrite(LEDI, 0); 
}

///////////////////////////////////////////////////////////////////loop///////////////////////////////////////////
void loop() {
  // 1. BUCLE DE CARRERA
  while(true){
    int go = digitalRead(BOTONB); 
    frenos();
    lectura();
    PID();
    
    // control remoto para detenerlo
    if(go == 1){ 
      motores(-20, -20); 
      delay(50);         
      break;             
    }
  }
  
  // 2. BUCLE DE PARADA TOTAL
  while(true){
    motores(0, 0); 
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// --------------------------------------------------------
// FUNCIONES DE CALIBRACIÓN
// --------------------------------------------------------

void fondos() {
  for(int i = 0; i < 8; i++){
    lectura_fondo[i] = analogRead(pines_sensores[i]);
    // Serial.print(lectura_fondo[i]);
    // Serial.print("\t"); 
  }
  // Serial.println(" "); 
}

void lineas() {
  for(int i = 0; i < 8; i++){
    lectura_linea[i] = analogRead(pines_sensores[i]);
    // Serial.print(lectura_linea[i]);
    // Serial.print("\t");
  }
  // Serial.println(" ");
}

void promedio() {
  for(int i = 0; i < 8; i++){
    umbral[i] = (lectura_fondo[i] + lectura_linea[i]) / 2;
    // Serial.print(umbral[i]);
    // Serial.print("\t");
  }
  // Serial.println(" ");
}

// --------------------------------------------------------
// LECTURA PRINCIPAL
// --------------------------------------------------------

int lectura(void) {
  suma = 0;
  
  for(int i = 0; i < 8; i++){
    sensores[i] = analogRead(pines_sensores[i]);

    if(sensores[i] <= umbral[i]) {
      digital[i] = 0;
    } else {
      digital[i] = 1;
    }
    
    // Serial.print(digital[i]);
    // Serial.print("\t");
    
    suma += digital[i]; 
  }
  

  // Matemática de posición centrada a 8 sensores (Máximo 700)
  sumap = (700L * digital[0] + 600L * digital[1] + 500L * digital[2] + 400L * digital[3] + 
           300L * digital[4] + 200L * digital[5] + 100L * digital[6] + 0L * digital[7]);

  if (suma != 0) {
    pos = sumap / suma; 
  } else {
    pos = -1; 
  }

  // Lógica de inercia (Centro es 350)
  if (poslast <= 350 && pos == -1) {
    pos = 0;   
  }
  if (poslast >= 350 && pos == -1) {
    pos = 700; 
  }

  if (pos != -1) {
    poslast = pos;
  }

  // Serial.print("-> Pos: ");
  return pos;
}

void PID(){
  proporcional=pos-setpoint;
  derivativo=proporcional-last_prop;
  integral=error1+error2+error3+error4+error5+error6;
  last_prop=proporcional;
  error6=error5;
  error5=error4;
  error4=error3;
  error3=error2;
  error2=error1;
  error1=proporcional;
  int diferencial=(proporcional*KP) + (derivativo*KD) + (integral*KI);
  if(diferencial > vel) diferencial=vel;
  else if(diferencial < -vel) diferencial=-vel;
  (diferencial < 0)?
  motores(vel, vel+diferencial):motores(vel-diferencial, vel);
}

void frenos(){
  if(pos<=100){
    motores(veladelante, -velatras);
  }
  if(pos>=600){
    motores(-velatras, veladelante);
  }
}

// --------------------------------------------------------
// CONTROL DE MOTORES (TB6612FNG)
// --------------------------------------------------------

void motores(int izq, int der){ // 0 hasta 255  |  0 hasta -255
  //////////////// motor LEFT "IZQUIERDO" ////////////////////////
  if(izq >= 0){
    digitalWrite(izq1, HIGH);
    digitalWrite(izq2, LOW);
  }
  else{
    digitalWrite(izq1, LOW);
    digitalWrite(izq2, HIGH);
    izq = izq * (-1);
  }
  analogWrite(pwmi, izq);
  
  //////////////// motor RIGHT "DERECHO" ////////////////////////
  if(der >= 0){
    digitalWrite(der1, HIGH);
    digitalWrite(der2, LOW);
  }
  else{
    digitalWrite(der1, LOW);
    digitalWrite(der2, HIGH);
    der = der * (-1);
  }
  analogWrite(pwmd, der);
}
