//=================================================================================
// Реализация контроллера Ethernet на основе приемо-передатчика RTL8211EG
//---------------------------------------------------------------------------------
// Sanity timer
//=================================================================================
module santim(
	input			clock_i,		// 2,5 MHz тактовая
	input			ena_i,		// Разрешение работы
	input			gen_i,		// Сигнал генерации
	output		out_o			// Выход, активный 1
);

reg			nbdcok;
reg  [3:0]	bdcok_cnt;
localparam	BDCOK_LIMIT = 10;
assign out_o = ~nbdcok;

// Генерация отрицательного BDCOK ~4 msec.
always @(posedge clock_i) begin
	if(ena_i & gen_i) begin
		if(bdcok_cnt != BDCOK_LIMIT) begin
			bdcok_cnt <= bdcok_cnt + 1'b1;
			nbdcok <= 1'b0;
		end
		else
			nbdcok <= 1'b1;
	end
	else begin
		nbdcok <= 1'b1; bdcok_cnt <= 4'b0;
	end
end

endmodule
