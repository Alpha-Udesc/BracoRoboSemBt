//Código do braço (sem bluetooth):
#include <Arduino.h>
#include <Servo.h>




#define numeros_servo_motores 4 // Numero de servos motores na garra


// Joystick é um potênciometro, por isso é conectado aos pinos analógicos
#define joystick_pin_direita_x A4 // Pino analógico que o eixo horizontal do joystick a direita está
#define joystick_pin_direita_y A5 // Pino analógico que o eixo vertical do joystick a direita está
#define joystick_pin_esquerda_x A1 // Pino analógico que o eixo horizontal do joystick a esquerda está
#define joystick_pin_esquerda_y A0 // Pino analógico que o eixo vertical do joystick a esquerda está


enum servo_nums { //enumerações dos servos dos braços do braço robótico.
arm = 0,
claw,
ant_arm,
base,
}mapeamento_range_servos;


enum joystick_signals { //sinais dos joysticks ( Não utilizado no código )
x_direita = 0,
y_direita,
x_esquerda,
y_esquerda,
}joystick_signals;




//variaveis de mapeamento das portas dos componentes
int portas_joystick[numeros_servo_motores] = {joystick_pin_direita_x,joystick_pin_esquerda_x,joystick_pin_esquerda_y,joystick_pin_direita_y}; // Vetor que armazena sentidos do analógico.
int portas_servos[numeros_servo_motores] = {2,5,3,4}; // Vetor que guarda as portas dos servos.




//variaveis de mapeamentos dos ranges e valores inicias dos servo motores
int default_value_servo_map[numeros_servo_motores] = {60,70,100,80}; // Valores iniciais dos servos motores.
int max_range_servos[numeros_servo_motores] =       {100,170,140,170}; // Valores máximos dos servos motores.
int min_range_servos[numeros_servo_motores] =         {20,50,60,10}; // Valores minimos dos servos motores.


typedef struct servo_info{ //estrutura de configuração do servo motor
  boolean ligando = false; // Variável que diz se o arduino está ligando.
  Servo servo; // Criação do servo motor.
  int min_range, max_range, servo_num; // Ranges máximos e mínimos do servo e seu número de identificação.
  }servo_info;


servo_info servos_especs[4]; //declaração dos servo motores


int servos_positions_value[numeros_servo_motores] = {default_value_servo_map[arm],default_value_servo_map[claw],default_value_servo_map[ant_arm],default_value_servo_map[base]}; // Valores padrões de cada servo( arm, claw, ant_arm, base ).






void setup() {
Serial.begin(9600);


  servo_initialize();


}


void loop() {
  joystick_handler();
}


void servo_initialize(void){ //inicializador dos servos
  for(int i = 0; i<numeros_servo_motores; i++){
    servos_especs[i].servo.attach(portas_servos[i]); // Indexação das portas que estão sendo utilizadas para cada servo.
    servos_especs[i].min_range =  min_range_servos[i]; // Especificação do valor mínimo do servo.
    servos_especs[i].max_range =  max_range_servos[i]; // Especificação do valor máximo do servo.
    servos_especs[i].servo_num = i; // Número de identificação do servo.
    servo_write(&servos_especs[i],default_value_servo_map[i]); // Chamada da função que controla o servo, nesse caso, inicializamos com o valor default de cada servo.
  }
  }


void servo_write(servo_info *servo_motor, int value){ //função que controle os servo motores
  if(value>=servo_motor->min_range&&value<=servo_motor->max_range){ // Atendida se o valor passado está entre o mínimo e o máximo do servo.
    servo_motor->servo.write(value); // Passa o ângulo para o servo.
  }
}


void joystick_handler(void){  //função que captura os valores dos controles
  int auxiliar_valor = 0;
  for(int contador = 0; contador < numeros_servo_motores; contador++){ // "Varredura" dos analógicos.
    auxiliar_valor = analogRead(portas_joystick[contador]); // Leitura dos dados do analógico.
    if(auxiliar_valor>600||auxiliar_valor<400){ // Range de ação do analógico( Se estiver no centro, não faz nada ).
      robot_handler(contador,auxiliar_valor); // Chamada da função que controla o braço.
    }
  }
}


void robot_handler(int servo, int valor){ //função que controla os servo motores
  Serial.println(valor); // Mostra o valor que os analógicos estão
  if(valor>550){
    if(servos_positions_value[servo]<max_range_servos[servo]){ // Se o valor da leitura do analógico é maior que 550 e a posição( ângulo ) do servo controlado por esse eixo do analógico é menor que a máxima dele, a condição é aceita.
    // delay(valor*-0.211+221);
      servos_positions_value[servo]++; // Atualização da posição no vetor.
      }
    }
  if(valor<450){
     if(servos_positions_value[servo]>min_range_servos[servo]){ // Se o valor da leitura do analógico é menor que 450 e a posição( ângulo ) do servo controlado por esse eixo do analógico é maior que a mínima dele, a condição é aceita.
     // delay(valor*0.211+5);
      servos_positions_value[servo]--; // Atualização da posição no vetor.
      }
    }
   delay(5);
   servo_write(&servos_especs[servo],servos_positions_value[servo]);  // Passa o novo valor da posição vetor para atualizar no servo.
  }






 




