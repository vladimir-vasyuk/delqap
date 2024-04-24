//=================================================================================
// Реализация контроллера Ethernet DELQA.
//---------------------------------------------------------------------------------
// Модуль программного сброса.
//=================================================================================
module soft_reset(
	input			clk,		// Тактовая
	input			rst,		// Сигнал сброса
	input			csr_sr,	// Требование программного сброса
	output		block,	// Сигнал блокировки внешней шины
	output		reset		// Сигнал программного сброса
);

reg  [1:0]	reset_r;
wire combrst = rst | reset_r[1];
assign reset = reset_r[1];
assign block = reset_r[0];

always @(posedge clk) begin
	if(combrst)
		reset_r <= 2'b0;
	else begin
		if(csr_sr)
			reset_r[0] <= 1'b1;
		else begin
			if(reset_r[0])
				reset_r[1] <= 1'b1;
		end
	end
end

endmodule
