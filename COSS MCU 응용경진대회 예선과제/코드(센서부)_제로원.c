#define F_CPU 16000000L
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/io.h>
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#define UART_H_

// 수위센서 출력 제어핀 : PB5
#define waterPin PB5;

// 스텝모터 제어 핀 데이터: IN1, IN2, IN3, IN4
#define IN1 PB0
#define IN2 PB1
#define IN3 PB2
#define IN4 PB3

// dht11 온습도 센서 핀
#define DHTPIN PD2      // DHT11 데이터 핀 (Arduino 핀 2)
#define OUTPUT_PIN PD3  // 임계값 이상일 때 HIGH 출력할 핀 (Arduino 핀 3)



// 스텝 시퀀스 배열 (Half Step)
uint8_t stepSequence[8] = {
	0b0001, // IN1
	0b0011, // IN1+IN2
	0b0010, // IN2
	0b0110, // IN2+IN3
	0b0100, // IN3
	0b1100, // IN3+IN4
	0b1000, // IN4
	0b1001  // IN4+IN1
};

// 스텝 모터 제어하는 함수
void setup() {
	// 제어 핀을 출력으로 설정
	DDRB |= (1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4);
}

void stepMotor(int step) {
	// PORTB의 하위 4비트를 사용하여 스텝 수행
	PORTB = (PORTB & 0xF0) | stepSequence[step];
}

void stepForward(int steps) {
	for (int i = 0; i < steps; i++) {
		for (int j = 0; j < 8; j++) {
			stepMotor(j);
			_delay_ms(10);  // 지연 시간을 통해 속도 조절, 필요 시 조절
		}
	}
}

// 수위센서 제어하는 함수
void UART_INIT(void) {
	UCSR0A |= _BV(U2X0);

	UBRR0H = 0x00;
	UBRR0L = 207;

	UCSR0C |= 0x06;
	
	UCSR0B |= _BV(RXEN0);
	UCSR0B |= _BV(TXEN0);
}

unsigned char UART_receive(void)
{
	while( !(UCSR0A & (1<<RXC0)) );
	return UDR0;
}

void UART_transmit(unsigned char data)
{
	while( !(UCSR0A & (1<<UDRE0)) );
	UDR0 = data;
}

void UART_printString(char *str)
{
	for(int i = 0; str[i]; i++)
	UART_transmit(str[i]);
}

void UART_print8bitNumber(uint8_t no)
{
	char numString[4] = "0";
	int i, index = 0;
	
	if(no > 0){
		for(i = 0; no != 0 ; i++)
		{
			numString[i] = no % 10 + '0';
			no = no / 10;
		}
		numString[i] = '\0';
		index = i - 1;
	}
	
	for(i = index; i >= 0; i--)
	UART_transmit(numString[i]);
}

void UART_print16bitNumber(uint16_t no)
{
	char numString[6] = "0";
	int i, index = 0;
	
	if(no > 0){
		for(i = 0; no != 0 ; i++)
		{
			numString[i] = no % 10 + '0';
			no = no / 10;
		}
		numString[i] = '\0';
		index = i - 1;
	}
	
	for(i = index; i >= 0; i--)
	UART_transmit(numString[i]);
}

void UART_print32bitNumber(uint32_t no)
{
	char numString[11] = "0";
	int i, index = 0;
	
	if(no > 0){
		for(i = 0; no != 0 ; i++)
		{
			numString[i] = no % 10 + '0';
			no = no / 10;
		}
		numString[i] = '\0';
		index = i - 1;
	}
	
	for(i = index; i >= 0; i--)
	UART_transmit(numString[i]);
}

void ADC_INIT(unsigned char channel){
	ADMUX |=0x40; // AVcc 전압 참조 설정
	ADCSRA |= 0x07; // 분주비 설정
	ADCSRA |= (1<<ADEN); // ADC 활성화
	ADCSRA |= (1<<ADATE); // 자동 변환 모드 설정
	
	ADMUX = (ADMUX & 0xE0) | channel; // 채널 설정
	ADCSRA |= (1<<ADSC); // 변환 시작
}

int read_ADC(void){
	while (!(ADCSRA & (1<<ADIF))); // 변환 완료 대기
	return ADC; // ADC 값 반환
}

void int_to_string(int n, char *buffer){
	sprintf(buffer, "%04d", n);
	buffer[4] = '\0'; // 문자열 종료
}

// dht11 제어하는 함수
uint8_t readDHT11(uint8_t* temperature) {
	uint8_t bits[5] = {0};
	uint8_t i, j = 0;

	// DHT11 시작 신호
	DDRD |= (1 << DHTPIN);  // 출력 설정
	PORTD &= ~(1 << DHTPIN);// LOW로 설정
	_delay_ms(18);          // 18ms 기다림
	PORTD |= (1 << DHTPIN); // HIGH로 설정
	_delay_us(40);
	DDRD &= ~(1 << DHTPIN); // 입력으로 설정
	_delay_us(10);

	// DHT11 응답 확인 (80us 대기)
	if (PIND & (1 << DHTPIN)) return 1;
	_delay_us(80);
	if (!(PIND & (1 << DHTPIN))) return 1;
	_delay_us(80);

	// 데이터 읽기 40 비트 (5 바이트)
	for (j = 0; j < 5; j++) {
		for (i = 0; i < 8; i++) {
			while (!(PIND & (1 << DHTPIN))); // HIGH 대기
			_delay_us(30);

			if (PIND & (1 << DHTPIN))
			bits[j] |= (1 << (7 - i)); // 1 비트 읽기

			while (PIND & (1 << DHTPIN)); // 다음 LOW 대기
		}
	}
	
	// 핀 초기화
	DDRD |= (1 << DHTPIN);
	PORTD |= (1 << DHTPIN);

	// 체크섬 검사
	if ((uint8_t)((bits[0] + bits[1] + bits[2] + bits[3]) & 0xFF) != bits[4])
	return 1;

	*temperature = bits[2]; // 온도 값 반환 (정수)
	return 0;
}




int main(void)
{
	// 초기화
	int read;
	char buffer[5];
	
	UART_INIT();
	ADC_INIT(0);

	// PB5 핀을 출력으로 설정 (LED 제어)
	DDRB |= (1 << PB5);
	PORTB |= (1 << PB5); // 기본 상태는 LED OFF

	while (1)
	{
		read = read_ADC(); // 수위 센서 값 읽기
		int_to_string(read, buffer); // 정수를 문자열로 변환
		UART_printString(buffer); // UART로 출력
		UART_printString("\n");

		// 수위 센서 값에 따라 LED 제어
		if (read >= 400) {
			PORTB &= ~(1 << PB5); // LED OFF = 물이 충분하다


			} else {
			PORTB |= (1 << PB5); // LED ON = 물이 부족하다
			stepForward(256);  // 더 적은 수의 측정으로 결과 시운전 가능
			_delay_ms(1000);   // 1초 대기
			
		}
		_delay_ms(1000); // 1초 대기
		
		// 온수 조절
		uint8_t temperature = 0;

		// OUTPUT_PIN 설정
		DDRD |= (1 << OUTPUT_PIN);
		PORTD &= ~(1 << OUTPUT_PIN); // 기본값은 LOW

		
		if (readDHT11(&temperature) == 0) {
			if (temperature < 31) {
				PORTD |= (1 << OUTPUT_PIN); // 출력 HIGH
				} else {
				PORTD &= ~(1 << OUTPUT_PIN); // 출력 LOW
			}
			_delay_ms(2000); // 2초 대기
		}

	}

}
