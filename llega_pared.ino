#include <TimerOne.h>
#include <NewPing.h>

//Configuración de los pines digitales de los motores
int in2md = 23;
int in1md = 25;
int in2mi = 22;
int in1mi = 24;

//Configuración de los pwm_ms de los motores
int pwm_md = 6;
int pwm_mi = 7;

//Variables
int pserie;
bool int_m = false; //Flag de interrupción
int modo = 1;
char t_filtro = 0;

//Variables de Control
int ref = 20;
int velc_i;
int velc_d;
int vel;
int err_i;
int err_d;
int dif_err;
#define Kpi 25
#define Kpd 25
#define Kp 10
int sum_err_i = 0;
int sum_err_d = 0;
#define Kii 0.005
#define Kid 0.005
int err_ai = 0;
int err_ad = 0;
int err_amed = 0;
int err_med = 0;
#define Kpi2 5
#define Kpd2 5

//ULTRASONIDO IZQUIERDO
#define Trig_i 12
#define Echo_i 13
int MAX_DIST_I = 400; // Máxima distancia para el primer ultrasonido en cm
int dist_i = 0;
int dist_i_ant = 0;

//ULTRASONIDO DERECHO
#define Trig_d 10
#define Echo_d 11
int MAX_DIST_D = 400; // Máxima distancia para el segundo ultrasonido en cm
int dist_d = 0;
int dist_d_ant = 0;

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
  Timer1.initialize(100000);        // Dispara cada 100 ms
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
    Serial.print("100"); Serial.print(" ");
    Serial.print(dist_i); Serial.print(" ");
    Serial.print(dist_d); Serial.print(" ");
    Serial.print(ref); Serial.print(" ");
    Serial.print(modo); Serial.print(" ");
    Serial.print(velc_i); Serial.print(" ");
    Serial.print(velc_d); Serial.println(" ");

    dist_i = sonar[0].ping_cm(); // Medida en centimetros IZQ
    dist_d = sonar[1].ping_cm(); // Medida en centimetros DER

    if (t_filtro >= 30) { //Este filtro suaviza las medidas que sean muy dispares entre dos medidas realizadas
      if (abs(dist_i - dist_i_ant > 10)) {
        dist_i = (dist_i + dist_i_ant) / 2;
      }
      if (abs(dist_d - dist_d_ant > 10)) {
        dist_d = (dist_d + dist_d_ant) / 2;
      }
    }

    err_i = ref - dist_i;
    err_d = ref - dist_d;
    err_med = (err_i + err_d) / 2;

    if (abs(err_med) > 3) //Si el error medio de las medidas es mayor que 3 cm, se activa un control P
    {
      velc_i = -1 * Kpi * err_med;
      velc_d = velc_i;
      sum_err_i = 0;
      sum_err_d = 0;
    }

    else {  //Si el error medio es menor o igual que 3 cm, se activa un control PI para cada rueda

      //ControlPI
      if (err_i != 0) {
        velc_i = -1 * (Kpi2 * err_i + Kii * sum_err_i * 100);
      }
      else {
        velc_i = 0;
        sum_err_i = 0;
      }
      if (err_d != 0) {
        velc_d = -1 * (Kpd2 * err_d + Kid * sum_err_d * 100);
      }
      else {
        velc_d = 0;
        sum_err_d = 0;
      }

      //Zona Muerta
      if (err_i < 0 ) {
        velc_i = velc_i + 66;
      }
      if (err_i > 0) {
        velc_i = velc_i - 66;
      }
      if (err_d < 0 ) {
        velc_d = velc_d + 66;
      }
      if (err_d > 0) {
        velc_d = velc_d - 66;
      }

      //Anti wind-up
      if (velc_i < 255 && velc_i > -255) {
        sum_err_i += err_i;
      }
      if (velc_d < 255 && velc_d > -255) {
        sum_err_d += err_d;
      }
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
    err_ai = err_i;
    err_ad = err_d;
    err_amed = err_med;
    dist_d_ant = dist_d;
    dist_i_ant = dist_i;
    int_m = false;
  }
}


//Función que interpreta el sentido y velocidad de los motores
void vel_control(int vel_i, int vel_d) {
  if (vel_i < 0) {
    digitalWrite(in1md, LOW);
    digitalWrite(in2md, HIGH);
    digitalWrite(in1mi, LOW);
    digitalWrite(in2mi, HIGH);
  }
  else {
    digitalWrite(in1md, HIGH);
    digitalWrite(in2md, LOW);
    digitalWrite(in1mi, HIGH);
    digitalWrite(in2mi, LOW);
  }
  if (vel_d < 0) {
    digitalWrite(in1md, LOW);
    digitalWrite(in2md, HIGH);
    digitalWrite(in1mi, LOW);
    digitalWrite(in2mi, HIGH);
  }
  else {
    digitalWrite(in1md, HIGH);
    digitalWrite(in2md, LOW);
    digitalWrite(in1mi, HIGH);
    digitalWrite(in2mi, LOW);
  }
  analogWrite(pwm_md, abs(vel_d));
  analogWrite(pwm_mi, abs(vel_i));
}


//Las funciones siguientes se usaron para la configuración inicial correcta del robot móvil
void lineal (int pwm_m, bool sentido)
{
  int pwm_ml;
  pwm_ml = pwm_m;
  if (sentido == 1) {
    digitalWrite(in1md, HIGH);
    digitalWrite(in2md, LOW);
    digitalWrite(in1mi, HIGH);
    digitalWrite(in2mi, LOW);
    if (pwm_ml < 15) {
      pwm_ml = 15;
    }
    analogWrite(pwm_md, pwm_ml - 15);
    analogWrite(pwm_mi, pwm_m);
  }
  if (sentido == 0) {
    digitalWrite(in1md, LOW);
    digitalWrite(in2md, HIGH);
    digitalWrite(in1mi, LOW);
    digitalWrite(in2mi, HIGH);
    if (pwm_ml < 13) {
      pwm_ml = 13;
    }
    analogWrite(pwm_md, pwm_ml - 13);
    analogWrite(pwm_mi, pwm_m);
  }
}

void rotacion (int pwm_m, bool sentido)
{
  if (sentido == 1) {
    digitalWrite(in1md, LOW);
    digitalWrite(in2md, HIGH);
    digitalWrite(in1mi, HIGH);
    digitalWrite(in2mi, LOW);
  }
  if (sentido == 0) {
    digitalWrite(in1md, HIGH);
    digitalWrite(in2md, LOW);
    digitalWrite(in1mi, LOW);
    digitalWrite(in2mi, HIGH);
  }
  analogWrite(pwm_md, pwm_m);
  analogWrite(pwm_mi, pwm_m);
}

void giro(int pwm_m, bool sentido, bool giro)
{
  int pwm_mc;
  pwm_mc = pwm_m;
  if (sentido == 1) {
    digitalWrite(in1md, HIGH);
    digitalWrite(in2md, LOW);
    digitalWrite(in1mi, HIGH);
    digitalWrite(in2mi, LOW);
  }
  if (sentido == 0) {
    digitalWrite(in1md, LOW);
    digitalWrite(in2md, HIGH);
    digitalWrite(in1mi, LOW);
    digitalWrite(in2mi, HIGH);
  }
  if (giro == 1) {
    if (pwm_m < 50) {
      pwm_mc = 50;
    }
    analogWrite(pwm_md, pwm_m);
    analogWrite(pwm_mi, pwm_mc - 50);
  }
  if (giro == 0) {
    if (pwm_m < 65) {
      pwm_mc = 65;
    }
    analogWrite(pwm_md, pwm_mc - 65);
    analogWrite(pwm_mi, pwm_m);
  }
}
