//Programa : Comunicacao Xbee utilizando Arduino Xbee Shield
//Autor : FILIPEFLOP
 
//Armazena os valores recebidos da serial
int valores = 0;
//Armazena o estado do led
String estado;
  
void setup()
{
  //Define o pino 13 - LED embutido no Arduino - como saida
  pinMode(13, OUTPUT);
  Serial.begin(9600);
}
  
void loop()
{
  //Aguarda dados na serial
  if (Serial.available() > 0)
  {
    valores = Serial.read();
    //Caso seja recebido 0, apaga o led
    if(valores == '0')
    {
      digitalWrite(13, LOW);
      estado = "apagado";
    }
    //Caso seja recebido 1, acende o led
    else if(valores == '1')
    {
      digitalWrite(13, HIGH);
      estado = "aceso";
    }    
    //Envia mensagem de confirmacao
    Serial.print(" Led ");
    Serial.print(estado);
    Serial.write(10);
  }
}