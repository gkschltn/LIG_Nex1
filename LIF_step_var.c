#include "xparameters.h"
#include "xuartlite.h"
#include "stdio.h"
#include "string.h"
#include "sleep.h"      // usleep 함수 사용 가능
#include "stdlib.h"     // atoi 함수 사용을 위해 추가
#include "xil_printf.h"
#include "xil_io.h"

#define LIF_BASE_ADDR           0x44A00000
#define LIF_REG_INPUT           (LIF_BASE_ADDR + 0x00) // slv_reg 0
#define LIF_REG_SPIKE_OUT       (LIF_BASE_ADDR + 0x04) // slv_reg 1
#define LIF_REG_STEP            (LIF_BASE_ADDR + 0x08) // slv_reg 2
#define LIF_REG_SPIKE_COUNT     (LIF_BASE_ADDR + 0x0C) // slv_reg 3
#define LIF_REG_SP_STEPS        (LIF_BASE_ADDR + 0x10) // slv_reg 4 (sp_steps)
#define LIF_REG_THRESHOLD       (LIF_BASE_ADDR + 0x14) // slv_reg 5 (threshold)

// UART 드라이버 인스턴스 선언
XUartLite UartLite;

/*
 * 사용자 입력을 통해 입력값, step 수, threshold 값을 결정
 * 사용자 입력을 u32 정수로 변환하여 반환
 */
u32 getUserInput(const char* prompt){
	u8 userInput[20] = {0};
	u8 receivedChar;
	int index = 0;

	xil_printf(prompt); // 입력 요청 메시지 출력

	while(1) {
		// 데이터가 수신될 때까지 대기
		while (XUartLite_Recv(&UartLite, &receivedChar, 1) == 0);

		if (receivedChar == '\r') { // Enter 키 입력 시 종료
			userInput[index] = '\0';
			xil_printf("\r\n");
			break;
		}
		else if (receivedChar >= '0' && receivedChar <= '9') {
			if (index < 19) {
				userInput[index++] = receivedChar;
				xil_printf("%c", receivedChar); // Echo
			}
		}
	}
	return atoi((const char*)userInput); // 문자열을 정수로 변환하여 반환
}

int main() {
    u32 input_current;
    u32 sp_steps;
    u32 threshold;

    // UART 드라이버 초기화
    int status = XUartLite_Initialize(&UartLite, XPAR_UARTLITE_0_DEVICE_ID);
	if (status != XST_SUCCESS) {
		xil_printf("UART Lite Initialization Failed\r\n");
		return XST_FAILURE;
	}

    xil_printf("\r\n--- LIF Neuron Controller ---\r\n");

    // 1. Parameter값 입력 받기
	sp_steps      = getUserInput("1. Enter sp_steps (e.g., 64): ");
	threshold     = getUserInput("2. Enter Threshold (0-255): ");
	xil_printf("---------------------------\r\n");

	// 2. 입력받은 parameter 값들을 각각의 FPGA 레지스터에 쓰기
	Xil_Out32(LIF_REG_THRESHOLD, threshold);
	xil_printf("Sent Threshold: %d\r\n", threshold);

	Xil_Out32(LIF_REG_SP_STEPS, sp_steps);
	xil_printf("Sent sp_stepss: %d\r\n", sp_steps);

	usleep(100000);

	while (1) {
		// 3. input current 값 입력
		u8 userInput[20] = {0};
		u8  receivedChar;
		int index = 0;

		// 예외 처리
		xil_printf("Enter New Input Current (or type 'exit' to quit): ");

		// 'exit' 문자열을 받기 위해 숫자 외의 문자도 허용
		while(1) {
			while (XUartLite_Recv(&UartLite, &receivedChar, 1) == 0);

			if (receivedChar == '\r') {
				userInput[index] = '\0';
				xil_printf("\r\n");
				break;
			}
			// 백스페이스 처리 (선택사항이지만 사용자 편의를 위해 추가)
			else if (receivedChar == '\b' || receivedChar == 127) {
				 if (index > 0) {
					  index--;
					  // 터미널에서 글자를 지우는 제어 코드
					  xil_printf("\b \b");
				 }
			}
			else if (index < 19) {
				userInput[index++] = receivedChar;
				xil_printf("%c", receivedChar);
			}
		}

		// 입력된 문자열이 "exit" 인지 확인
		if (strcmp((const char*)userInput, "exit") == 0) {
			break; // "exit"가 맞으면 메인 루프 탈출
		}

		xil_printf("--- Running Simulation ---\r\n");

		// "exit"가 아니면 숫자로 변환하여 시뮬레이션 진행
		input_current = atoi((const char*)userInput);
		Xil_Out32(LIF_REG_INPUT, input_current);
		xil_printf("Sent Input Current: %d\r\n", input_current);
		usleep(100000);

		while (1) {
			u32 step = Xil_In32(LIF_REG_STEP);
			xil_printf("Time Step: %d\r\n", step);
			usleep(100000);

			u32 spike = Xil_In32(LIF_REG_SPIKE_OUT);
			if (spike == 1){
				xil_printf(">> Spike Out: %d\r\n", spike);
			}
			usleep(100000);

			if (step >= sp_steps){
				u32 count = Xil_In32(LIF_REG_SPIKE_COUNT);
				xil_printf("---------------------------\r\n");
				xil_printf("Total Spike Count: %d\r\n", count);
				xil_printf("Simulation Finished.\r\n");
				xil_printf("---------------------------\r\n\r\n");
				break;
			}
		}
	}

	xil_printf("Program terminated. \r\n");
	return 0; // main 함수 종료
}
