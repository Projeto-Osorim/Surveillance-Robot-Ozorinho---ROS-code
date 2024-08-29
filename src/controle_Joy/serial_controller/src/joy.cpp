#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <serial/serial.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
/*
  rosparam set joy_node/dev "/dev/input/js0"
  rosrun joy joy_node

  Ao abrir o terminal primeira coisa deve se colocar o parametro de imput para setar o Joystick
  depois dar um source.... 
  igual os codigos acima
*/

//variaveis Statica
serial::Serial arduino;
int serial_port;


void callback(const sensor_msgs::Joy::ConstPtr& msg) {
  /*
    nessa parte do codigo, setamos uma estrutura terminos, tty
    e damos um serial_port = open, para abrir a comunicação com a porta serial
  */
  struct termios tty;
  serial_port = open("/dev/ttyUSB0", O_RDWR);
  if (serial_port < 0) {
    perror("Erro ao abrir a porta serial");
    return ;
  }

  /*
    Aqui setamos os parametros da porta serial
  */
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(serial_port, &tty) != 0) {
    perror("Erro ao obter os atributos da porta serial");
    return ;
  }

  /*
    8 bits por caractere, habilita leitura, ignora controle de modem
  */
  tty.c_cflag = CS8 | CREAD | CLOCAL; 

  /*
    Ignora erros de paridade
  */
  tty.c_iflag = IGNPAR; 

  /*
    Modo não canônico
  */
  tty.c_oflag = 0;
  tty.c_lflag = 0; 

  /*
    Define a velocidade de transmissão (9600 bps)
  */
  cfsetospeed(&tty, B9600); 

  /*
    Aplica as configurações à porta serial
  */
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    perror("Erro ao aplicar os atributos da porta serial");
    return ;
  }

  /*
    Ao passar por todas essas verificaçoes a porta serial esta aberta e configurada
    corretamente para ser utilizada.
    String a ser enviada
  */  
  char message;

  /*
    Leio atraves da "msg", q basicamente é uma classe do tipo Joy
    passada por parametro na função callback
    leio o vetor axes, que tem duas posiçoes [0,1]
  */

 
  if (msg->axes[1] == 1) {
    message = 'W';
  } else if (msg->axes[1] == -1) {
    message = 'S';
  } else if (msg->axes[0] == 1) {
    message = 'A';
  } else if (msg->axes[0] == -1) {
    message = 'D';
  } else if (msg->buttons[0] == 1){
    message = 'P';
    //break;
  }

  // Envia a string para a porta serial 
  printf("%c\n",message);
  write(serial_port, &message, strlen(&message));
  //write(serial_port, &message, strlen(&message));
    
  // Fecha a porta serial
  close(serial_port);

}

int main(int argc, char** argv) { 
  ros::init(argc, argv, "joystick_control");

  ros::NodeHandle nh;

  ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1, callback);

  ros::spin();

  return 0;
}
