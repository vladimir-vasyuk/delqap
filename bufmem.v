//=================================================================================
// Реализация контроллера Ethernet DELQA на основе процессора М4 (LSI-11M)
// Всевозможные модули памяти
//
//=================================================================================
// Модуль регистровой памяти
//=================================================================================
module regf #(parameter NUM=6)
(
	input						clk,		// тактовая частота
	input  [NUM/2-1:0]	addr,		// адрес
	input  [15:0]			data,		// выходные данные
	input						we,		// разрешение записи
	output [15:0]			q			// выходные данные
);

reg[15:0] x[NUM-1:0];
assign q = x[addr];

always @(posedge clk) begin
   if(we) begin
      x[addr] <= data;
   end
end
endmodule


//=================================================================================
// Модуль ROM
//=================================================================================
module firmrom(
	input          wb_clk_i,	// тактовая частота шины
	input  [15:0]  wb_adr_i,	// адрес
	output [15:0]  wb_dat_o,	// выходные данные
	input          wb_cyc_i,	// начало цикла шины
	input          wb_stb_i,	// строб цикла шины
	output         wb_ack_o		// подтверждение выбора устройства
);

// Формирование сигнала подтверждения выбора устройства
reg  [1:0] ack;
always @(posedge wb_clk_i)
begin
   ack[0] <= wb_cyc_i & wb_stb_i;
   ack[1] <= wb_cyc_i & ack[0];
end
assign wb_ack_o = wb_cyc_i & wb_stb_i & ack[1];
/*
reg ack;
always @(posedge wb_clk_i)
   if (wb_stb_i & wb_cyc_i)
		ack <= 1'b1;
   else
		ack <= 1'b0;
assign wb_ack_o = ack & wb_stb_i;
*/
// Блок ПЗУ
rom bdrom(
   .address(wb_adr_i[11:1]),
   .clock(wb_clk_i),
   .q(wb_dat_o)
);
endmodule


//=================================================================================
// Модуль RXBUF
//=================================================================================
module rxbuf(
	input				wb_clk_i,	// тактовая частота шины
	input  [10:0]	wb_adr_i,	// адрес
	input  [15:0]	wb_dat_i,	// входные данные
	output [15:0]	wb_dat_o,	// выходные данные
	input				wb_cyc_i,	// начало цикла шины
	input				wb_we_i,		// разрешение записи (0 - чтение)
	input				wb_stb_i,	// строб цикла шины
	output			wb_ack_o,	// подтверждение выбора устройства
// ПДП (DMA)
	input				dma_stb_i,	// строб
	input  [10:0]	dma_adr_i,	// адрес
	output [15:0]	dma_dat_o,	// выходные данные
// Ethernet
	input	 [9:0]	eth_adr_i,	// адрес
	input	 [15:0]	eth_dat_i,	// входные данные
	input				eth_clk_i,	// тактовая
	input				eth_we_i,	// разрешение записи

	input  [1:0]	adr_mode_i	// режим работы (шина, DMA, Ethernet)
);

// Сигналы упраления обменом с шиной
wire bus_strobe = wb_cyc_i & wb_stb_i & ~wb_ack_o;	// строб цикла шины
wire bus_we = bus_strobe & wb_we_i;						// запрос записи

// Формирование сигнала подтверждения выбора устройства
reg  [1:0] ack;
always @(posedge wb_clk_i) begin
   ack[0] <= wb_cyc_i & wb_stb_i;
   ack[1] <= wb_cyc_i & ack[0];
end
assign wb_ack_o = wb_cyc_i & wb_stb_i & ack[1];

wire [15:0]	dat_o;	// выходные данные
assign wb_dat_o = dat_o;
assign dma_dat_o = dat_o;

// Мультиплексер адреса, данных и управляющих сигналов
reg  [10:0]	adr_i;	// адрес
reg  [15:0]	dat_i;	// входные данные
reg			clk_i;	// тактовая
reg			we_i;		// разрешение записи

always @(*) begin
	case (adr_mode_i)
		2'b00: begin	// режим щины
			adr_i[10:0] = wb_adr_i[10:0];
			dat_i = wb_dat_i;
			clk_i = wb_clk_i;
			we_i = bus_we;
		end
		2'b01: begin	// режим ПДП (DMA)
			adr_i[10:0] = dma_adr_i[10:0];
			dat_i = 16'b0;
			clk_i = wb_clk_i;
			we_i = 1'b0;
		end
		2'b10: begin	// режим Ethernet
			adr_i = {1'b0, eth_adr_i[9:0]};
			dat_i = eth_dat_i;
			clk_i = eth_clk_i;
			we_i = eth_we_i;
		end
		default: begin	// режим ПДП (DMA)
			adr_i[10:0] = dma_adr_i[10:0];
			dat_i = 16'b0;
			clk_i = wb_clk_i;
			we_i = 1'b0;
		end
	endcase
end

// Блок памяти
buf2kw bufrx(
	.address(adr_i),
	.clock(clk_i),
	.data(dat_i),
	.wren(we_i),
	.q(dat_o)
);
endmodule


//=================================================================================
// Модуль TXBUF
//=================================================================================
module txbuf(
	input				wb_clk_i,	// тактовая частота шины
	input  [9:0]	wb_adr_i,	// адрес
	input  [15:0]	wb_dat_i,	// входные данные
	output [15:0]	wb_dat_o,	// выходные данные
	input				wb_cyc_i,	// начало цикла шины
	input				wb_we_i,		// разрешение записи (0 - чтение)
	input				wb_stb_i,	// строб цикла шины
	output			wb_ack_o,	// подтверждение выбора устройства
// ПДП (DMA)
	input				dma_stb_i,	// строб
	input  [9:0]	dma_adr_i,	// адрес
	input  [15:0]	dma_dat_i,	// входные данные
	input				dma_we_i,	// разрешение записи (0 - чтение)
// Ethernet
	input	 [9:0]	eth_adr_i,	// адрес
	output [15:0]	eth_dat_o,	// выходные данные
	input				eth_clk_i,	// тактовая частота
//
	input  [1:0]	adr_mode_i	// режим работы (шина, DMA, Ethernet)
);

// Сигналы упраления обменом с шиной
wire bus_strobe = wb_cyc_i & wb_stb_i & ~wb_ack_o;	// строб цикла шины
wire bus_we = bus_strobe & wb_we_i;						// запрос записи

// Формирование сигнала подтверждения выбора устройства
/*
reg ack;
always @(posedge wb_clk_i)
   if (wb_stb_i & wb_cyc_i)
		ack <= 1'b1;
   else
		ack <= 1'b0;
assign wb_ack_o = ack & wb_stb_i;
*/
reg  [1:0] ack;
always @(posedge wb_clk_i) begin
   ack[0] <= wb_cyc_i & wb_stb_i;
   ack[1] <= wb_cyc_i & ack[0];
end
assign wb_ack_o = wb_cyc_i & wb_stb_i & ack[1];

wire [15:0]	dat_o;	// выходные данные
wire			dma_we;
assign dma_we = dma_stb_i & dma_we_i;
assign wb_dat_o = dat_o;
assign eth_dat_o = dat_o;

// Мультиплексер адреса, данных и управляющих сигналов
reg  [9:0]	adr_i;	// адрес
reg  [15:0]	dat_i;	// входные данные
reg			clk_i;	// тактовая
reg			we_i;		// разрешение записи

always @(*) begin
	case (adr_mode_i)
		2'b00: begin	// режим щины
			adr_i[9:0] = wb_adr_i[9:0];
			dat_i = wb_dat_i;
			clk_i = wb_clk_i;
			we_i = bus_we;
		end
		2'b01: begin	// режим ПДП (DMA)
			adr_i[9:0] = dma_adr_i[9:0];
			dat_i = dma_dat_i;
			clk_i = wb_clk_i;
			we_i = dma_we;
		end
		2'b10: begin	// режим Ethernet
			adr_i = eth_adr_i[9:0];
			dat_i = 16'b0;
			clk_i = eth_clk_i;
			we_i = 1'b0;
		end
		default: begin	// режим щины
			adr_i[9:0] = wb_adr_i[9:0];
			dat_i = wb_dat_i;
			clk_i = wb_clk_i;
			we_i = bus_we;
		end
	endcase
end

// Блок памяти
buf1kw bufwr(
	.address(adr_i),
	.clock(clk_i),
	.data(dat_i),
	.wren(we_i),
	.q(dat_o)
);
endmodule


//=================================================================================
// Модуль RAM с управляющей программой + BD ROM
//=================================================================================
module firmware (
   input          wb_clk_i,	// тактовая частота шины
   input  [15:0]  wb_adr_i,	// адрес
   input  [15:0]  wb_dat_i,	// входные данные
   output [15:0]  wb_dat_o,	// выходные данные
   input          wb_cyc_i,	// начало цикла шины
   input          wb_we_i,		// разрешение записи (0 - чтение)
   input  [1:0]   wb_sel_i,	// выбор байтов для записи 
   input          prg_stb_i,	// строб модуля RAM
	input          rom_stb_i,	// строб модуля BD ROM
   output         wb_ack_o		// подтверждение выбора устройства
);

wire [1:0]	enaprg;
assign enaprg = prg_stb_i? (wb_we_i ? wb_sel_i : 2'b11) : 2'b00;

wire [15:0]	prgdat, romdat;
assign wb_dat_o = rom_stb_i? romdat : prgdat;

reg prgack;
reg [1:0] romack;
always @(posedge wb_clk_i) begin
   if (prg_stb_i & wb_cyc_i)
		prgack <= 1'b1;
   else
		prgack <= 1'b0;
   if (rom_stb_i & wb_cyc_i) begin
		romack[0] <= rom_stb_i;
		romack[1] <= romack[0];
	end
   else
		romack <= 2'b0;
end	
assign wb_ack_o = (romack[1] & rom_stb_i) | (prgack & prg_stb_i);

firmw ram(
   .address(wb_adr_i[11:1]),
   .byteena(enaprg),
   .clock(wb_clk_i),
   .data(wb_dat_i),
   .rden(~wb_we_i & wb_cyc_i & prg_stb_i),
   .wren( wb_we_i & wb_cyc_i & prg_stb_i),
   .q(prgdat)
);

rom bdrom(
   .address(wb_adr_i[11:1]),
   .clock(wb_clk_i),
	.rden(~wb_we_i & wb_cyc_i & rom_stb_i),
   .q(romdat)
);

endmodule
