#define F_CPU 16000000L
#include <avr/io.h>
#include <util/delay.h>

#define PRESCALER 1024 // 분주비
#define TRIG_PIN PB4
#define ECHO_PIN PD7

uint8_t step_data[]={0x01, 0x02, 0x04, 0x08};
int step_index = -1; // 스텝모터 변수 선언

uint8_t step_forward(void) // 정방향
{
	step_index++;
	if(step_index>=4) step_index=0;
	
	return step_data[step_index];
	
}
uint8_t step_backward(void) // 역방향
{
	step_index--;
	if(step_index<0) step_index=3;
	
	return step_data[step_index];
}

void Timer_init() // 분주비 설정 함수
{
	TCCR0B |= (1 << CS02) | (1 << CS00); // 분주비를 1024로 설정
}

uint8_t measure_distance() // 거리 측정 함수
{
	// 트리거 펄스 생성
	PORTB |= (1 << TRIG_PIN);
	_delay_us(10);
	PORTB &= ~(1 << TRIG_PIN);
	
	// 에코 핀이 HIGH될 때까지 대기
	TCNT0 = 0;
	while(!(PIND & 0x80))
	if(TCNT0 > 250) return 255;
	
	// 에코 핀이 LOW될 떄까지의 시간 측정
	TCNT0 = 0;
	while(PIND & 0x80)
	{
		if(TCNT0 > 250)
		{
			TCNT0 = 0;
			break;
		}
	}
	
	double pulse_width = TCNT0 * PRESCALER * 1000000.0 / F_CPU ;
	
	return pulse_width / 58;
}

int main(void)
{
	while(1){
		uint8_t distance;
		DDRB |= 0x1F; // TRIG 핀, 스텝 모터1 (노즐 on) 을 출력으로
		DDRC |= 0x3F; // DC모터, 스텝 모터2 (물분사 제어) 를 출력으로
		DDRD &= ~(0xBF); // PD0 ~ 5, ECHO(PD7) 핀을 입력으로
		PORTD |= 0x3F; // PD0~5 풀업저항 사용
		
		Timer_init();
		int angle = 0;
		int loop;
		distance = measure_distance();
		
		if (distance > 10){
			loop = 0;
		} else loop = 1;
		
		while(loop)
		{
			if((PIND&0x01)==0){
				for(int i=0;i<200;i++) {
					PORTB=step_forward();
					_delay_ms(5);
				}
				PORTD |= (1 << PD0);
			}
			if((PIND&0x02) == 0){ // 물 분사 버튼 누르면 스텝모터 회전
				for(int i=0;i<171;i++){
					PORTC=step_forward();
					_delay_ms(10);
				}
				angle += 171;
				PORTD |= (1 << PD1);
			}
			if((PIND&0x04)==0){ // up 버튼
				if(angle<514 && angle>0) {
					for(int i=0; i<171; i++){
						PORTC=step_forward();
						_delay_ms(10);
					}
					angle += 171;
				}
				PORTD |= (1 << PD2);
			}
			if((PIND&0x08)==0){
				if(angle>171){
					for(int i=0; i<171; i++){
						PORTC=step_backward();
						_delay_ms(10);
					}
					angle -= 171;
				}
				PORTD |= (1 << PD3);
			}
			if((PIND&0x10)==0){ // 정지
				if(angle>0){
					for(int i=0; i<angle; i++){
						PORTC=step_backward();
						_delay_ms(10);
					}
				}
				PORTD |= (1 << PD4);
			}
			if((PIND&0x20)==0){ // 건조 버튼 누르면 팬 회전
				if(angle == 0) {
					PORTC |= 0x30;
					} else if(angle > 0) {
					for(int i=0; i<angle; i++){
						PORTB=step_backward();
						_delay_ms(10);
					}
					PORTC |= 0x30;
				}
				PORTD |= (1 << PD5);
			}
		} // while 괄호
	}// 추가한 while 괄호
} // main 괄호