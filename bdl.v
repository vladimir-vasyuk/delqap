//=================================================================================
// Реализация контроллера Ethernet DELQA на основе процессора М4 (LSI-11M)
//=================================================================================
// Модуль BDL
//=================================================================================
module bdl(
// Внутренняя шина 
	input				wb_clk_i,	// тактовая частота шины
	input  [2:0]	wb_adr_i,	// адрес
	input  [15:0]	wb_dat_i,	// входные данные
	input				wb_cyc_i,	// начало цикла шины
	input				wb_we_i,		// разрешение записи (0 - чтение)
	input  [1:0]	wb_sel_i,	// выбор байтов для записи 
	input				wb_stb_i,	// строб цикла шины
	output			wb_ack_o,	// подтверждение выбора устройства
// Шина ПДП/DMA
	input  [2:0]	dma_adr_i,	// адрес
	input  [15:0]	dma_dat_i,	// входные данные
	input				dma_we_i,	// разрешение записи (0 - чтение)
	input				dma_stb_i,	// строб
// Общий выход данных
	output [15:0]	bdl_dat_o
);

// Формирование сигнала подтверждения выбора устройства
reg			ack;
always @(posedge wb_clk_i)
   if (wb_stb_i & wb_cyc_i)
		ack <= 1'b1;
   else
		ack <= 1'b0;
assign wb_ack_o = ack & wb_stb_i;

wire [15:0]	bdldin;			// входные данные BDL
wire [15:0]	bdldout;			// выходные данные BDL
wire [2:0]	bdladr;			// адрес BDL
wire			bdlwe;			// сигнал записи BDL

assign bdlwe = dma_stb_i? dma_we_i : (wb_stb_i? wb_we_i : 1'b0);
assign bdladr = dma_stb_i? dma_adr_i[2:0] : wb_adr_i[2:0];
assign bdldin = dma_stb_i? dma_dat_i : wb_dat_i;
assign bdl_dat_o = bdldout;

regf #(.NUM(6)) bdl(
   .clk(wb_clk_i),
   .addr(bdladr),
   .data(bdldin),
   .we(bdlwe),
   .q(bdldout)
);

endmodule
