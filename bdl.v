//=================================================================================
// Реализация контроллера Ethernet DELQA на основе процессора М4 (LSI-11M)
//
// 
//
//=================================================================================
// Модуль BDL
//=================================================================================
module bdl(
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
// DMA
	input  [2:0]	dma_adr_i,
	input  [15:0]	dma_dat_i,
	output [15:0]	dma_dat_o,
	input				dma_we_i,
	input				dma_stb_i
);

// Сигналы упраления обменом с шиной
wire			bus_strobe, bus_read_req, bus_write_req;
assign bus_strobe = wb_cyc_i & wb_stb_i & ~wb_ack_o;	// строб цикла шины
assign bus_read_req = bus_strobe & ~wb_we_i;				// запрос чтения
assign bus_write_req = bus_strobe & wb_we_i;				// запрос записи

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
assign bdladr = dma_stb_i? dma_adr_i[2:0] : (wb_stb_i? wb_adr_i[2:0] : 3'b0);
assign bdldin = dma_stb_i? dma_dat_i : wb_dat_i;
assign dma_dat_o = dma_stb_i? bdldout : 16'b0;
assign wb_dat_o = bdldout;

regf #(.NUM(6)) bdl(
   .clk(wb_clk_i),
   .addr(bdladr),
   .data(bdldin),
   .we(bdlwe),
   .q(bdldout)
);

endmodule
