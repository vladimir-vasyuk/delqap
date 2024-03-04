//=================================================================================
// Реализация контроллера Ethernet DELQA на основе процессора М4 (LSI-11M)
//=================================================================================
// Модуль контроля блока Ethernet (не используется, удален из проекта).
//=================================================================================
module ethctl(
// Bus
	input				wb_clk_i,
	input  [2:0]	wb_adr_i,
	input  [15:0]	wb_dat_i,
	output [15:0]	wb_dat_o,
	input				wb_cyc_i,
	input				wb_we_i,
	input  [1:0]	wb_sel_i,
	input				wb_stb_i,
	output			wb_ack_o,
// Ethernet
	output [10:0]	e_txcntb_o,
	input  [10:0]	e_rxcntb_i,
	output [7:0]	e_mode_o,
	input  [7:0]	e_sts_errs_i,
	output [15:0]	e_mdval_o,
	input  [15:0]	e_mdval_i,
	output [6:0]	e_mdctrl_o,
	input  [7:0]	e_mdstatus_i,
//
	output			eth_txmode_o,
	output			eth_rxmode_o,
// Sanity timer
	output			santm_res_o,
// Indication
	output [2:0]	dev_ind_o
);

// Сигналы упраления обменом с шиной
wire			bus_read_req, bus_write_req, bus_strobe;
assign bus_strobe = wb_cyc_i & wb_stb_i & ~wb_ack_o;	// строб цикла шины
assign bus_read_req = bus_strobe & ~wb_we_i;				// запрос чтения
assign bus_write_req = bus_strobe & wb_we_i;				// запрос записи

reg  [15:0]	wb_dat;
reg			ack;
always @(posedge wb_clk_i)
   if (wb_stb_i & wb_cyc_i)
		ack <= 1'b1;
   else
		ack <= 1'b0;
assign wb_ack_o = ack & wb_stb_i;
assign wb_dat_o = wb_dat;

reg  [7:0]	e_mode;			// Регистр режима работы				-- 24040
// {rxdone, txrdy, bdl_h, setup, extmode, intemode, intmode, rxmode}
// wire [7:0]	e_sts_errs;	// статус и ошибки приема/передачи	-- 24040
// {rxrdy, txdone, 0, crs_err, mdc_err, e_txer, rx_err, rx_crc_err}
reg  [10:0]	e_txcntb;		// Регистры кол-ва байт 				-- 24042 (прием/передача)
//wire [10:0]	e_rxcntb;	// Регистр кол-ва принятых байт		-- 24042
reg  [15:0]	e_mdval;			// входные/выходные данные MD 		-- 24044
reg  [6:0]	e_mdctrl;		// сигналы управления MD 				-- 24046
// управление (e_mdctrl):
//		6:		1/0 - write/read
//		5:		1 - start
//		4:0	reg. address
// статус (e_mdstatus_i)
//		7: 	1/0 - ready/busy
//		6,5:	10-1000Мб/с; 01-100Мб/с; 00-10Мб/с; 11-зарезервированно
//		4:		1-полный дуплекс; 0-полудуплекс
//		3:		зарезервированно (0)
//		2:		1-MDI crossover; 0-MDI
//		1:		1-приемник готов; 0-приемник не готов
//		0:		1-связь есть; 0-связи нет

wire			eth_mode_tx, eth_mode_rx;
assign e_txcntb_o = e_txcntb;
assign e_mode_o = e_mode;
assign e_mdval_o = e_mdval;
assign e_mdctrl_o = e_mdctrl;
assign eth_rxmode_o = e_mode[0] | e_mode[1] | e_mode[2] | e_mode[4];
assign eth_txmode_o = e_mode[6];

// Дополнительные регистры
reg  [1:0]	santm_res;		// сброс sanity -- 24054
reg  [2:0]	devind;			// регистры индикации -- 24056
assign santm_res_o = |santm_res;
assign dev_ind_o = devind;

//**************************************************
// Работа с шиной
//
always @(posedge wb_clk_i) begin
	// Чтение регистров
	if (bus_read_req == 1'b1) begin
		case (wb_adr_i[2:0])
			3'b000:	// 24040
				wb_dat <= {8'b0, e_sts_errs_i};
			3'b001:	// 24042
				wb_dat <= {5'b0, e_rxcntb_i[10:0]};
			3'b010:	// 24044
				wb_dat <= e_mdval_i;
			3'b011:	// 24046
				wb_dat <= {8'b0, e_mdstatus_i[7:0]};
			default:
				wb_dat <= 16'b0;
		endcase
	end
	// Запись регистров
	else if (bus_write_req == 1'b1) begin
		if (wb_sel_i[0] == 1'b1) begin   // Запись младшего байта
			case (wb_adr_i[2:0])
				3'b000:	// 24040
					e_mode[7:0] <= wb_dat_i[7:0];
				3'b001:	// 24042
					e_txcntb[7:0] <= wb_dat_i[7:0];
				3'b010:	// 24044
					e_mdval[7:0] <= wb_dat_i[7:0];
				3'b011:	// 24046
					e_mdctrl[6:0] <= wb_dat_i[6:0];
				3'b110:	// 24054
					santm_res[0] <= wb_dat_i[0];
				3'b111:	// 24056
					devind[2:0] <= wb_dat_i[2:0];
			endcase
		end
		if(wb_sel_i[1] == 1'b1) begin    // Запись старшего байта
			case (wb_adr_i[2:0])
				3'b001:	// 24042
					e_txcntb[10:8] <= wb_dat_i[10:8];
				3'b010:	// 24044
					e_mdval[15:8] <= wb_dat_i[15:8];
			endcase
		end
	end
	else begin
		santm_res[1:0] <= {santm_res[0], 1'b0};
		if(~e_sts_errs_i[7] & e_mode[7])
			e_mode[7] <= 1'b0;
	end
end

endmodule
