module LIF_Neuron (
    input clk,
    input reset,
	
	// Flag signal
	input data_en,		// pixel값이 입력된 경우 Neuron 동작 
    input clear_spike,	// spike_out 값을 clear 시키기 위함
	input step_en,		// 각 step 단계를 수행
	input end_step,		// step >= 64 && spike_count 읽기 완료
	
	// input signal
    input [7:0] input_current,
	input [7:0] sp_steps,
	input [7:0] threshold,
	
	// output signal
    output reg spike_out,               	// spike_register에 따라 값 반영
	output reg [7:0] step,					// time step 값
	output reg [7:0] spike_count			// input_current에 대한 spike_out 발생 횟수
);

    // parameter 선언
	parameter n_sp_activate = 8'd5;
    // parameter THRESHOLD = 8'd50;
    parameter LEAK = 8'd1;
    parameter Beta = 16'd209;    // 0.819 * 256 ? 209
    parameter Weight = 16'd102;  // 0.4 * 256 ? 102
								 // Q8.8 형식 (정수 8비트 + 소수 8비트)
	
	// LFSR signal
	reg [7:0] random;
	
	// 내부 신호
	reg spike;
	reg [7:0] num_out;
	reg [7:0] membrane_potential;
    reg [15:0] mem_tmp;
    reg [15:0] input_tmp;
    
	reg data_en_d;
	
	// 유지
	reg spike_register;
	reg [7:0] count_register;

    always @(posedge clk or posedge reset) begin

		if (reset) begin
			// 초기값 설정
			random <= 8'b00000001;
			spike <= 0;
			num_out <= 0;
			membrane_potential <= 0;
			mem_tmp <= 0;
			input_tmp <= 0;
			step <= 1;
			spike_register <= 0;
			count_register <= 0;
			
			data_en_d <= 0;
			
		end else begin
			// data_en rising edge: 새 픽셀 평가 시작
			
			data_en_d <= data_en;
			
            if (data_en && !data_en_d) begin
                step               <= 1;
                membrane_potential <= 0;
                num_out            <= 0;
                spike_register     <= 0;
				count_register 	   <= 0;
            end
			
			// 매 step마다 동작
			if (data_en && step_en) begin
				
				// 1. LFSR random number 생성
				random <= {random[6:0], random[7] ^ random[5] ^ random[4] ^ random[3]};
				step <= step + 1;
				
				// 2. rate coding
				if (random < input_current)
					spike <= 1;
				else spike <= 0;
				
				// 3. spike 발생 : 입력값 적용 | spike 발생 X : 입력값 적용 X
				mem_tmp   <= membrane_potential * Beta;
				input_tmp <= input_current * Weight;
				membrane_potential <= (mem_tmp + (spike? input_tmp : 0)) >> 8;
				
				// 4. Threshold 넘는 횟수 누적
				if (membrane_potential >= threshold) begin
					num_out <= num_out + 1;
					membrane_potential <= membrane_potential - threshold;
				end
			
				// 5. spike_out 발생
				if (num_out >= n_sp_activate) begin
					spike_register <= 1'b1;  // spike값은 latch 상태로 유지
					membrane_potential <= 0; // membrane_potential 초기화
					num_out <= 0;			 // 횟수 초기화
					count_register <= count_register + 1; // spike_out 발생 count
				end				
			end
			
			// 6. spike_out 발생 시 clear_spike
			if (clear_spike) begin		// spike_out 읽기 + spike_out = 1
				spike_register <= 1'b0;
			end	
				
			// 7. time step을 초과한 경우 전체 reset
			if (end_step) begin
				step <= 1;
				membrane_potential <= 0;
				num_out <= 0;
				spike_register <= 0;
				count_register <= 0;
			end
		end
	end
	
    always @(posedge clk or posedge reset) begin
	  if (reset) begin
		spike_out   <= 1'b0;
		spike_count <= 8'd0;
	  end else begin
		spike_out   <= spike_register;
		spike_count <= count_register;
	  end
	end

endmodule
