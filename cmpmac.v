//=================================================================================
// Реализация контроллера Ethernet DELQA на основе процессора М4 (LSI-11M)
//=================================================================================
// Модуль сравнения MAC адреса принятого пакета со списокм разоешенных адресов
// Передача данных в модуль возможна только при установленном бите S-пакета
// Для инициализации:
//		- установить адоес (запись в BASE);
//		- записать значение адоеса (BASE+2, BASE+4, BASE+6)
//		- цикл на 14 значений.
// По окончании инициализации установить адрес = 0 !!!
//=================================================================================
module cmpmac(
// внутренняя шина
	input				wb_clk_i,	// тактовая частота шины
	input				rst_i,		// сброс
	input  [2:0]	wb_adr_i,	// адрес
	input  [15:0]	wb_dat_i,	// входные данные
	output [15:0]	wb_dat_o,	// выходные данные
	input				wb_cyc_i,	// начало цикла шины
	input				wb_we_i,		// разрешение записи (0 - чтение)
	input				wb_stb_i,	// строб цикла шины
	input  [1:0]	wb_sel_i,
	output			wb_ack_o,	// подтверждение выбора устройства
// Ethernet
	input	 [2:0]	eth_pms_i,	// режим прослушивания/широковещания/установки
	input				eth_clk_i,	// тактовая частота
	input				eth_macr_i,	// MAC принят
	input  [47:0]	eth_macd_i,	// принятый MAC адоес
	output			cmp_done_o,	// сигнал "операция сравнения завершена"
	output			cmp_res_o	// результат сравнения
);

// Массив разрешенных адресов
reg [47:0]  stdmac[13:0];
reg [3:0]	adr;
reg [15:0]	wb_dat;
reg [15:0]	buf1, buf2;
reg [7:0]	buf3;
assign wb_dat_o = wb_dat;

wire			stpac, promisc;
reg			cmp_res, cmp_done;
assign stpac = eth_pms_i[0];
assign promisc = eth_pms_i[2];
assign cmp_res_o = promisc? 1'b1 : cmp_res;
assign cmp_done_o = promisc? 1'b1 : cmp_done;

// Сигналы упраления обменом с шиной
wire			bus_strobe, bus_write_req, bus_read_req;
assign bus_strobe = wb_cyc_i & wb_stb_i & ~wb_ack_o & stpac;	// строб цикла шины
assign bus_read_req = bus_strobe & ~wb_we_i;							// запрос чтения
assign bus_write_req = bus_strobe & wb_we_i;							// запрос записи

reg  [1:0]	ack;
always @(posedge wb_clk_i) begin
	ack[0] <= wb_cyc_i & wb_stb_i;
	ack[1] <= wb_cyc_i & ack[0];
end
assign wb_ack_o = wb_cyc_i & wb_stb_i & ack[1];

// Инициализация массива адресов
genvar i;
generate
for (i = 0; i < 14; i = i + 1)
begin : reg_init
    initial
        stdmac[i] = 48'b0;
end
endgenerate

wire clock;				// Тактовая
assign clock = stpac? wb_clk_i : eth_clk_i;

always @(posedge clock, posedge rst_i)  begin
	if(rst_i) begin
		cmp_res <= 1'b0; cmp_done <= 1'b0;
		adr <= 4'b0;
	end
	else if(bus_read_req) begin
		case(wb_adr_i[2:0])
			3'b000: // 24120
				wb_dat <= {12'b0, adr};
			3'b001: // 24122
				wb_dat <= stdmac[adr][15:0];
			3'b010: // 24124
				wb_dat <= stdmac[adr][31:16];
			3'b011: // 24126
				wb_dat <= stdmac[adr][47:32];
		endcase
	end
	else if(bus_write_req) begin
		if(wb_sel_i[1]) begin
			case(wb_adr_i[2:0])
				3'b000: // 24120
					adr <= wb_dat_i[3:0];
				3'b001: // 24122
					buf1[15:0] <= wb_dat_i[15:0];
				3'b010: // 24124
					buf2[15:0] <= wb_dat_i[15:0];
				3'b011: // 24126
					stdmac[adr] <= {wb_dat_i[15:0], buf2, buf1};
			endcase
		end
	end
	else begin
		if(eth_macr_i) begin
			if((adr <= 4'd13) & (cmp_done == 1'b0)) begin
				if(eth_macd_i == stdmac[adr]) begin
					cmp_res <= 1'b1;
					cmp_done <= 1'b1;
				end
				adr <= adr + 1'b1;
			end
			else
				cmp_done <= 1'b1;
		end
		else begin
			if(cmp_done) begin
				adr <= 4'b0;
				cmp_res <= 1'b0;
				cmp_done <= 1'b0;
			end
		end
	end
end

endmodule
