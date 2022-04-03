/*Aluno: Alderi Leocádio da Silva Filho - Matrícula: 118210951
 *
 * Sprint10_Funcionalidade_Livre.c
 *
 * Código de um tacômetro para medir RPM do motor, distância percorrida e diâmetrodo pneu do carro.
 * Apresenta as infromações em display OLED, como também a velocidade em um conjunto de 3 displays de 7 segmentos.
 * Lógica de acionamento do motor do veículo e através de uma ponte H o veículo possui 3 modos de operação.
 * Um potenciômetro	simula o pedal do acelerador, esse sinal é capturado por um conversor AD e gera um sinal PWM para controlar a velocidade do motor.
 * Os valores do diâmetro e da distância percorrida são salvos na memória EEPROM.
 * Um sonar detecta a distância entre o veículo e um obstáculo a sua frente.
 * Dependendo da distância (menor que 3m) e da velocidade (maior que 20km/h) controla o duty cicle do PWM reduzindo-o para 10%.
 * Apresenta no display OLED o percentual de tensão da bateria e a temperatura da bateria.
 * O valor da temperatura máxima da bateria é salvo na EEPROM.
 * Enviando o comando 'd' pela UART é retornado o valor da temperatura máxima da bateria. Enviando o comando 'l' o valor é limpado.
 *
 */ 

#define F_CPU 16000000UL //Frequência de trabalho da CPU
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "SSD1306.h"
#include "Font5x8.h"

//Variáveis globais utilizadas ao decorrer do código
int velocidade_veiculo = 0;
uint16_t tamanho_pneu = 0;
uint64_t distancia_percorrida_cm = 0;
uint16_t distancia_percorrida_km = 0;
int tempoX = 0;
int tempo_anterior_us = 0;
int intervalo_de_tempo = 0;
int quant_voltas_pneu = 0;
int RPM = 0;
uint32_t acelerador = 0;
uint32_t acelerador_proporcional = 0;
int tempo_borda_subida = 0;
int variacao_tempo = 0;
int distancia_colisao = 0;
uint32_t bateria = 0;
float bateria_proporcional = 0;
float Rt = 0;
uint32_t temperatura_bateria = 0;
float temperatura_bateria_proporcional = 0;
int temperatura_maxima = 0;
float autonomia = 0;

//Flags
int flag1 = 0;
int flag2 = 0;

//Variáveis auxiliares
int auxiliar1 = 0;
int auxiliar2 = 0;

ISR(USART_RX_vect){ //Interrupação USART, sempre que receber um dado serial o código é desviado para essa interrupção
	
	char recebido;
	recebido = UDR0; //O registrador UDR0 contem o valor da palavra que chegou pela serial
	
	if(recebido=='l'){ //Se receber o caractere l
		temperatura_maxima = 0; //Zera a temperatura máxima
	}
	
	if(recebido=='d'){ //Se receber o caractere d
		USART_Transmit(eeprom_read_byte(4)); //Transmite o valor da temperatura máxima que está salvo na EEPROM
	}
}

void USART_Init(unsigned int ubrr){
	
	UBRR0H = (unsigned char)(ubrr>>8); //Ajusta a taxa de transmissão, parte alta
	UBRR0L = (unsigned char)ubrr; //Ajusta a taxa de transmissao, parte baixa
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0); //Habilita a interrupçao de recepção, como também as opeações de recepção e transmissão
	UCSR0C = (1<<USBS0)|(3<<UCSZ00); //Ajusta o formato do frame: 8 bits de dados e 2 de parada
	
}

// ||Função para envio de um frame de 5 a 8bits||
void USART_Transmit(unsigned char data){
	
	while(!( UCSR0A & (1<<UDRE0)));//Espera a limpeza do registr. de transmissão
	UDR0 = data; //Coloca o dado no registrador e o envia
}

// ||Função para recepção de um frame de 5 a 8bits||
unsigned char USART_Receive(void){
	
	while(!(UCSR0A & (1<<RXC0))); //Espera o dado ser recebido
	return UDR0; //Lê o dado recebido e retorna
}

ISR(PCINT2_vect){ //Interrupção externa para os pinos PD
	
	if((PIND&0b00010000) == 0) //Teste para identificar se o botão de aumentar o diâmetro está pressionado
	{
		if(tamanho_pneu < 200){ //Limita o tamanho do pneu ao máximo de 200cm de diâmetro
			tamanho_pneu++; //Aumenta o diâmetro do pneu
		}
	}
	
	if((PIND&0b00100000) == 0){ //Teste para identificar se o botão de diminuir o diâmetro está pressionado
		tamanho_pneu--; //Diminui o diâmetro do pneu
		
		if(tamanho_pneu <= 0){ //Não permite o tamanho do pneu passar abaixo de zero
			tamanho_pneu = 0;
		}
	}
}

ISR(TIMER0_COMPA_vect){ //Interrupção do timer TC0 a cada 1ms
	
	tempoX++;
	
	if((tempoX%5) == 0){ //Verdadeiro a cada 5ms
		flag1 = 1;
	}
	
	if((tempoX%300) == 0){ //verdadeiro a cada 300ms
		flag2 = 1;
	}
}

ISR(TIMER1_CAPT_vect){ //Interrupção por captura do TC1
	
	if(TCCR1B & (1<<ICES1)){ //Lê o valor de contagem do TC1 na borda de subida do sinal
		tempo_borda_subida = ICR1; //Recebe a primeira contagem para determinar a largura do pulso
	}
	else{ //Lê o valor de contagem do TC1 na borda de descida do sinal
		variacao_tempo = (ICR1 - tempo_borda_subida)*16; //Cada incremento do TC1 corresponde a 16us
	}
	
	TCCR1B ^= (1<<ICES1); //Inverte a borda de captura
	
	distancia_colisao = variacao_tempo/58; //Calculo da distancia (em cm) entre o carro e o objeto	
}

ISR(ADC_vect){ //Interrupção do ADC
	
	if(auxiliar2 == 1){
		ADMUX = 0b01000000; // Tensão de referência em VCC com o pino PC0 como fonte do sinal
		acelerador = ADC; //Variavél irá armazenar o valor do registrador ADC que é recebido pelo pino PC0
	}
	
	if(auxiliar2 == 2){
		ADMUX = 0b01000001; // Tensão de referência em VCC com o pino PC1 como fonte do sinal
		bateria = ADC; //Variavél irá armazenar o valor do registrador ADC que é recebido pelo pino PC1
	}
	
	if(auxiliar2 == 3){
		ADMUX = 0b01000010; // Tensão de referência em VCC com o pino PC2 como fonte do sinal
		temperatura_bateria = ADC; //Variavél irá armazenar o valor do registrador ADC que é recebido pelo pino PC2
		auxiliar2 = 0;
	}
	auxiliar2++;
}

ISR(INT0_vect){ //Interrupção externa INT0 relacionada ao tacômetro
	
	if(quant_voltas_pneu == 5){
		intervalo_de_tempo = tempoX - tempo_anterior_us; //Calculo do tempo que leva para percorrer a distância entre 2 bordas de descida, ou seja, o tempo que leva para percorrer o comprimento do pneu do carro
		
		tempo_anterior_us = tempoX; //Atualiza o valor da variável tempo_anterior_us
		
		RPM = 300000/intervalo_de_tempo; //Cálculo das Rotações Por Minuto do carro [(5voltas*60min*1000ms)/intervalo_de_tempo]
		
		velocidade_veiculo = (565.47*tamanho_pneu)/intervalo_de_tempo; //Cálculo da velocidade do carro [((5voltas*3.1415*10*3.6)*tamanho_pneu/intervalor_de_tempo]
		
		quant_voltas_pneu = 0;
	}
	
	quant_voltas_pneu++; //Variável que armazena a quantidade de voltas do pneu do carro
	
	distancia_percorrida_cm += 3.1415*tamanho_pneu; //Cálculo da distância percorrida em cm, a cada volta do pneu é incrementado a distância percorrida
	distancia_percorrida_km = distancia_percorrida_cm/100000; //Converte a distância percorrida de cm para km
	
	if(distancia_colisao < 300 && velocidade_veiculo > 20){ //Se a distância for menor que 3m e a velocidade maior que 20Km/h reduz o duty cicle para 10%
		acelerador_proporcional = (acelerador*255)/10230; //Ajusta a escala do potenciômetro/acelerador passando de analógico para digital
	}	
	else{
		acelerador_proporcional = (acelerador*255)/1023; //Ajusta a escala do potenciômetro/acelerador passando de analógico para digital
	}
		
	bateria_proporcional = (bateria*100)/1023; //Ajusta a escala da tensão da bateria
	
	Rt = (temperatura_bateria*4.88758)/(5 - (temperatura_bateria*0.004887)); //Calcula o valor do resistor RTD ajustando a escala [Rt = (Vt*R1)/(Vcc-Vt)], [Vt = temperatura_bateria*5/1023]
	temperatura_bateria_proporcional = ((float)2.597402)*(Rt-100); //Calcula a temperatura da bateria
	
	if(temperatura_bateria_proporcional - ((int)temperatura_bateria_proporcional)>= (1/2)){	//Faz o arredondamento do valor da temperatura
		temperatura_bateria_proporcional++;
	}
	
	if(temperatura_bateria_proporcional > temperatura_maxima){
		temperatura_maxima = (unsigned int)temperatura_bateria_proporcional;
	}
	
	if(temperatura_bateria_proporcional < 0){
		temperatura_bateria_proporcional = 0;
	}
	
	autonomia = 4.4*bateria_proporcional; //Calcula a autonomia do carro com realação a porcentagem da bateria
	
	if(distancia_colisao < 100){
		PORTC |= (1<<3);
	}
	else{
		PORTC = (0<<3);
	}
}

int main(void)
{
	//Configuração dos pinos
	DDRB = 0b11111110; //Habilita os pinos PB1 ao PB7 como saídas e o pino PB0 como entrada
	DDRD = 0b00001011; //Habilita os pinos PD2, PD4, PD5, PD6 e PD7  como entradas e PD0, PD1 e PD3 como saídas
	DDRC = 0b10111000; //Habilita os pinos PC0, PC1 e PC2 como entrada e o restante dos pinos PC como saídas
	
	//Habilitando os resistores de pull-up dos pinos de entrada
	PORTD |= (1<<2); //Habilita o resistor de pull-up do pino PD2
	PORTD |= (1<<4); //Habilita o resistor de pull-up do pino PD4
	PORTD |= (1<<5); //Habilita o resistor de pull-up do pino PD5
	PORTB |= (1<<0); //Habilita o resistor de pull-up do pino PB0
	
	//Configuração das interrupções
	EIMSK |= 0b00000001; //Habilita a interrupção externa INT0
	EIMSK |= 0b00000010; //Habilita a interrupção externa INT1
	
	EICRA |= 0b00000010; //Interrupção externa INT0 na borda de descida
	EICRA |= 0b00001000; //Interrupção externa INT1 na borda de descida
	
	PCICR  = 0b00000100; //Habilita interrupção externa PCINT2 para os pinos PD
	PCMSK2 = 0b00110000; //Habilita interrupção externa pinos PD4 e PD5
	
	//Configuração do timer T0
	TCCR0A = 0b00000010; //Habilita o modo CTC do TC0
	TCCR0B = 0b00000011; //TC0 com prescaler igual a 64
	OCR0A = 249; //Ajusta o comparador para o TC0 contar até 249
	TIMSK0 = 0b00000010; //Habilita a inturrupção por iguadade comparando com OCR0A. A interrupção ocorre a cada 1ms = (x+1)*64/16MHz, logo x+1 = 250  -> x = 249
	
	//Configuração registradores do ADC
	ADMUX = 0b01000000; // Tensão de referência em VCC com o pino PC0 como fonte do sinal
	ADCSRA= 0b11101111; // Habilita o AD, habilita o trigger automático, modo de conversão contínua e define o prescaler igual a 128
	ADCSRB= 0b00000000; // Auto trigger com modo de conversão contínua (ao final de uma conversão outra conversão é imediatamente disparada)
	DIDR0 = 0b00111000; // Habilita os pinos PC0, PC1 e PC2 como fonte de entrada do ADC
	
	//Configuração do timer T2
	TCCR2A = 0b00100011;// Habilita o PWM no modo rápido não invertido para o pino OC0B (pino PD3)
	TCCR2B = 0b00000100;// Configura o prescaler
	
	//Configuração do timer T1 modo captura
	TCCR1B = (1<<ICES1)|(1<<CS12); //Captura na borda de subida, TC1 com prescaler = 256. Estouro a cada 256*(2^16)/16MHz = (2^16)*16us = 1,04s
	TIMSK1 = 1<<ICIE1; //Habilita a interrupção por captura
	
	USART_Init(MYUBRR);
	
	sei(); //Habilita a chave geral de interrupções (interrupções globais)
	
	read_EEPROM(); //Chamando função para ler a memória EEPROM
	
	//Ligando display OLED
	GLCD_Setup(); //Inicializa a biblioteca do display OLED
	GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite); //Seta a Font5x8
	GLCD_InvertScreen(); //Inverte a cor de fundo do display OLED para branco
	
	while (1)
	{		
		OCR2B = acelerador_proporcional; //Registrador de comparção, armazena o valor do acelerador e gera o PWM para o pino OC2B
		
		mostrar_velocidade(&flag1); //Chamando função para imprimir a velocidade no display de 7segmentos
		
		mostrar_OLED(&flag2); //Chamando função para imprimir as informações (no display OLED) de diâmetro, RPM, distância a um obstáculo, distância percorrida, modo de operação, tensão da bateria e temperatura da bateria
		
		if(flag1){
			writ_EEPROM(); //Chamando função para escrever na memória EEPROM
		}
	}
}

void mostrar_velocidade(int *flag1m){
	
	if(*flag1m){
		switch (auxiliar1){
			case 1:
			PORTB &= 0b00000001; //Resetando os pinos PB1 a PB7
			PORTB |= 0b11000000; //Resetando o pino PB5, pino que habilita o display das unidades
			PORTB |= ((((velocidade_veiculo/1)%10) << 1) & 0b00011111); //Calculo para imprimir a unidade no display 7 segmentos com uma máscara para evitar lixo em caso de uma conta errada
			break;
			
			case 2:
			PORTB &= 0b00000001; //Resetando os pinos PB1 a PB7
			PORTB |= 0b10100000; //Resetando o pino PB6, pino que habilita o display das dezenas
			PORTB |= ((((velocidade_veiculo/10)%10) << 1) & 0b00011111); //Calculo para imprimir a dezana no display 7 segmentos com uma máscara para evitar lixo em caso de uma conta errada
			break;
			
			case 3:
			PORTB &= 0b00000001; //Resetando os pinos PB1 a PB7
			PORTB |= 0b01100000; //Resetando o pino PB7, pino que habilita o display das centenas
			PORTB |= ((((velocidade_veiculo/100)%10) << 1) & 0b00011111); //Calculo para imprimir a centena no display 7 segmentos com uma máscara para evitar lixo em caso de uma conta errada
			auxiliar1 = 0;
			break;
		}
		auxiliar1++;
		*flag1m = 0;
	}
}

void mostrar_OLED(int *flag2m){
	
	if(*flag2m && !(PINC & (1<<6))){
		//Funções para imprimir no display OLED
		GLCD_Clear(); //Apaga o conteúdo para reescrever
		
		GLCD_GotoXY(1, 1); //Define a posição para escrita
		GLCD_PrintString("LASD Car"); //Escreve a string no display OLED
		GLCD_DrawLine(1,10,48,10,GLCD_Black); //Escreve uma linha abaixo da string
		
		GLCD_GotoXY(1, 16); //Define a posição para escrita
		GLCD_PrintInteger(RPM); //Escreve o RMP do carro no display OLED
		GLCD_PrintString(" rpm"); //Escreve a string no display OLED
		
		GLCD_GotoXY(1, 25); //Define a posição para escrita
		GLCD_PrintString("Sonar: "); //Escreve a string no display OLED
		GLCD_PrintInteger(distancia_colisao); //Escreve a distância do objeto no display OLED
		GLCD_PrintString("cm");
		
		GLCD_GotoXY(1, 34); //Define a posição para escrita
		GLCD_PrintString("D. pneu: "); //Escreve a string no display OLED
		GLCD_PrintInteger(tamanho_pneu); //Escreve o diâmetro do pneu no display OLED
		GLCD_PrintString("cm");
		
		GLCD_GotoXY(20, 50); //Define a posição para escrita
		GLCD_PrintInteger(distancia_percorrida_km); //Escreve a distância percorrida no display OLED
		GLCD_PrintString(" Km"); //Escreve a string no display OLED
		
		GLCD_DrawRectangle(13,47,64,60,GLCD_Black); //Quadrado distância percorrida
		GLCD_DrawRectangle(87,47,97,60,GLCD_Black); //Quadrado modo de operação
		
		if(!(PIND & (1<<7))){ //Se o pino PD7 estiver recebendo 0 mostrará letra P no LCD indicando o modo park
			GLCD_GotoXY(90, 50); //Define a posição para escrita
			GLCD_PrintString("P"); //Escreve a string no display OLED
		}
		if(PIND & (1<<7)){ //Se o pino PD7 estiver recebendo 1
			if(PIND & (1<<6)){ //Se o pino 6 estiver recebendo 1 mostrará letra R no LCD indicando o modo reverse
				GLCD_GotoXY(90, 50); //Define a posição para escrita
				GLCD_PrintString("R"); //Escreve a string no display OLED
			}
			else if(!(PIND & (1<<6))){ //Se o pino 6 estiver recebendo 0 mostrará letra D no LCD indicando o modo drive
				GLCD_GotoXY(90, 50); //Define a posição para escrita
				GLCD_PrintString("D"); //Escreve a string no display OLED
			}
		}
		*flag2m = 0;
		GLCD_Render(); //Atualiza a tela do display para exibir o conteúdo
	}
	
	if(PINC & (1<<6)){
		if(*flag2m){
			//Funções para imprimir no display OLED
			GLCD_Clear(); //Apaga o conteúdo para reescrever
		
			GLCD_GotoXY(1, 1); //Define a posição para escrita
			GLCD_PrintString("LASD Car"); //Escreve a string no display OLED
			GLCD_DrawLine(1,10,48,10,GLCD_Black); //Escreve uma linha abaixo da string
		
			GLCD_GotoXY(30, 30); //Define a posição para escrita
			GLCD_PrintString("Autonomia"); //Escreve a string no display OLED
			GLCD_GotoXY(40, 40); //Define a posição para escrita
			GLCD_PrintInteger(autonomia); //Escreve o RMP do carro no display OLED
			GLCD_PrintString("Km"); //Escreve a string no display OLED
			
			GLCD_GotoXY(90, 3); //Define a posição para escrita
			GLCD_PrintInteger(bateria_proporcional); //Escreve a porcentagem da tensão da bateria no display OLED
			GLCD_PrintString(" %"); //Escreve a string no display OLED
			
			GLCD_GotoXY(90, 16); //Define a posição para escrita
			GLCD_PrintInteger(temperatura_bateria_proporcional); //Escreve a temperatura da bateria no display OLED
			GLCD_PrintString(" C"); //Escreve a string no display OLED
			
			GLCD_DrawRectangle(80,1,124,27,GLCD_Black); //Quadrado bateria e temperatura da bateria
		
			*flag2m = 0;
			GLCD_Render(); //Atualiza a tela do display para exibir o conteúdo
		}
	}
}

void read_EEPROM(){ //Função para ler a memória EEPROM
	
	distancia_percorrida_km = eeprom_read_word(0); //Lê a distância percorrida que foi salva na memória
	tamanho_pneu = eeprom_read_word(2); //Lê o diâmetro do penu que foi salvo na memória
	temperatura_maxima = eeprom_read_byte(4);
	
	distancia_percorrida_cm = distancia_percorrida_km*100000; //Recalculando a distância em cm para não iniciar com zero
}

void writ_EEPROM(){ //Função para escrever na memória EEPROM
	
	eeprom_write_word(0, distancia_percorrida_km); //Escreve o valor da distância percorrida na memória
	eeprom_write_word(2, tamanho_pneu); //Escreve o valor do diâmetro do pneu na memória
	eeprom_write_byte(4, temperatura_maxima);
	
}

