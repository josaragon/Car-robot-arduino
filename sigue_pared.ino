#include <TimerOne.h>
#include <NewPing.h>

//Configuración de los pines digitales de los motores
int in2md = 23;
int in1md = 25;
int in2mi = 22;
int in1mi = 24;

//Configuración de los pwm_ms de los motores
int pwm_md = 44;
int pwm_mi = 7;

//Variables
int pserie;
bool int_m = false; //Flag de interrupción
int modo = 1;
char t_filtro = 0;

//Variables de Control
int ref = 50;
int velc_i;
int velc_d;
int err_i;
int err_d;
#define Kp 3
#define Kii 0.001
#define Kid 0.001
int err_med = 0;
int lim = 2;
int vel = 100;
int dist_inf;
int dist_sup;
int error_min;
int error_relativo;
int sum_err_i = 0;
int sum_err_d = 0;

//ULTRASONIDO IZQUIERDO
#define Trig_i 32
#define Echo_i 33
int MAX_DIST_I = 400; // Máxima distancia para el primer ultrasonido en cm
int dist_i = 0;
int dist_i_ant = 0;
int dist_i_ant2 = 0;

//ULTRASONIDO DERECHO
#define Trig_d 30
#define Echo_d 31
int MAX_DIST_D = 400; // Máxima distancia para el segundo ultrasonido en cm
int dist_d = 0;
int dist_d_ant = 0;
int dist_d_ant2 = 0;

#define SONAR_NUM 2 // Numero de ultrasonidos a usar
NewPing sonar[SONAR_NUM] = {
  NewPing(Trig_i, Echo_i, MAX_DIST_I), // Trigger, echo y máxima distancia de cada sensor
  NewPing(Trig_d, Echo_d, MAX_DIST_D)
};

//Interrupción que activa un flag que se vigila continuamente en el bucle principal
void intTimer() {
  int_m = true;
}

void setup() {
  Serial.begin(38400);              // Se inicializa el puerto serie para la conexión bluetooth
  pinMode(in1md, OUTPUT);
  pinMode(in2md, OUTPUT);
  pinMode(in1mi, OUTPUT);
  pinMode(in2mi, OUTPUT);
  pinMode(pwm_md, OUTPUT);
  pinMode(pwm_mi, OUTPUT);
  digitalWrite(in1md, LOW);
  digitalWrite(in2md, LOW);
  digitalWrite(in1mi, LOW);
  digitalWrite(in2mi, LOW);
  Timer1.initialize(50000);         // Dispara cada 50 ms
  Timer1.attachInterrupt(intTimer); // Activa la interrupcion y la asocia a IntTimer
}

void loop() {
  if (int_m == true) {  //Se comprueba que se ha activado el flag
    t_filtro++;
    //Se lee por puerto serie la nueva referencia si la hubiera y se transforma a entero
    if (Serial.available() > 0) {
      String buff = "";
      while (Serial.available() > 0) {
        buff += (char)Serial.read();
      }
      ref = buff.toInt();
    }
    //Se escribe por puerto serie la información necesaria para la telemetría
    Serial.print("50"); Serial.print(" ");
    Serial.print(dist_i); Serial.print(" ");
    Serial.print(dist_d); Serial.print(" ");
    Serial.print(ref); Serial.print(" ");
    Serial.print(modo); Serial.print(" ");
    Serial.print(velc_i); Serial.print(" ");
    Serial.print(velc_d); Serial.println(" ");

    dist_inf = ref - lim;
    dist_sup = ref + lim;

    dist_i = sonar[0].ping_cm(); // Medida en centimetros IZQ
    dist_d = sonar[1].ping_cm(); // Medida en centimetros DER

    if (t_filtro >= 60) { //Este filtro suaviza las medidas que sean muy dispares entre tres medidas realizadas
      if (abs(dist_i - dist_i_ant > 8)) {
        dist_i = (dist_i + 2 * dist_i_ant + 2 * dist_i_ant2) / 5;
      }
      if (abs(dist_d - dist_d_ant > 8)) {
        dist_d = (dist_d + 2 * dist_d_ant + 2 * dist_i_ant2) / 5;
      }
    }

    err_i = ref - dist_i;
    err_d = ref - dist_d;
    err_med = (err_i + err_d) / 2;
    error_relativo = dist_i - dist_d;

    // Control de distancia con la pared
    if ((dist_i < dist_inf) && (dist_d < dist_inf)) {
      velc_i = vel + 0;
      velc_d = vel + Kp * err_med + Kid * sum_err_d * 50;
    }
    else if ((dist_i > dist_sup) && (dist_d > dist_sup)) {
      velc_i = vel + Kp * err_med + Kii * sum_err_i * 50;
      velc_d = vel + 0;
    }
    //Control de desvío de trayectoria
    else {
      if (error_relativo < -lim) {
        velc_i = vel + 0;
        velc_d = vel + Kp * error_relativo;
      }
      else if (error_relativo > lim) {
        velc_i = vel + Kp * error_relativo;
        velc_d = vel + 0;
      }
      else {
        velc_i = vel;
        velc_d = vel;
        sum_err_i = 0;
        sum_err_d = 0;
      }
    }

    //Anti wind-up
    if (velc_i < 255 && velc_i > -255) {
      sum_err_i += err_i;
    }
    if (velc_d < 255 && velc_d > -255) {
      sum_err_d += err_d;
    }

    //Saturaciones
    if (velc_i > 255) {
      velc_i = 255;
    }
    if (velc_i < -255) {
      velc_i = -255;
    }
    if (velc_d > 255) {
      velc_d = 255;
    }
    if (velc_d < -255) {
      velc_d = -255;
    }

    //Actualización de las variables de control
    vel_control(velc_i, velc_d);
    dist_i_ant2 = dist_i_ant;
    dist_d_ant2 = dist_d_ant;
    dist_d_ant = dist_d;
    dist_i_ant = dist_i;
    int_m = false;
  }
}

//Función que interpreta el sentido y velocidad de los motores
void vel_control(int vel_i, int vel_d) {
  if (vel_i > 0) {
    digitalWrite(in1md, HIGH);
    digitalWrite(in2md, LOW);
    digitalWrite(in1mi, HIGH);
    digitalWrite(in2mi, LOW);
  }
  else {
    digitalWrite(in1md, LOW);
    digitalWrite(in2md, HIGH);
    digitalWrite(in1mi, LOW);
    digitalWrite(in2mi, HIGH);
  }
  if (vel_d > 0) {
    digitalWrite(in1md, HIGH);
    digitalWrite(in2md, LOW);
    digitalWrite(in1mi, HIGH);
    digitalWrite(in2mi, LOW);
  }
  else {
    digitalWrite(in1md, LOW);
    digitalWrite(in2md, HIGH);
    digitalWrite(in1mi, LOW);
    digitalWrite(in2mi, HIGH);
  }
  analogWrite(pwm_md, abs(vel_d));
  analogWrite(pwm_mi, abs(vel_i - 15));
}
