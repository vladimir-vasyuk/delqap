//=================================================================================
// Реализация контроллера Ethernet DELQA на основе процессора М4 (LSI-11M)
// Основной модуль
//=================================================================================
module delqap(
   input					wb_clkp_i,	// тактовая частота шины		wb_clk
	input					wb_clkn_i,	// обратная тактовая
   input					wb_rst_i,   // сброс
   input  [2:0]		wb_adr_i,   // адрес 
   input  [15:0]		wb_dat_i,   // входные данные
   output [15:0]		wb_dat_o,   // выходные данные
   input					wb_cyc_i,   // начало цикла шины
   input					wb_we_i,    // разрешение записи (0 - чтение)
   input					wb_stb_i,   // строб цикла шины
   input  [1:0]		wb_sel_i,   // выбор байтов для записи 
   output				wb_ack_o,   // подтверждение выбора устройства
	output				bdcok,

// обработка прерывания   
   output				irq,        // запрос
   input					iack,       // подтверждение

// DMA
   output				dma_req,    // запрос DMA
   input					dma_gnt,    // подтверждение DMA
   output [21:0]		dma_adr_o,  // выходной адрес при DMA-обмене
   input  [15:0]		dma_dat_i,  // входная шина данных DMA
   output [15:0]		dma_dat_o,  // выходная шина данных DMA
   output				dma_stb_o,  // строб цикла шины DMA
   output				dma_we_o,   // направление передачи DMA (0 - память->модуль, 1 - модуль->память) 
   input					dma_ack_i,  // Ответ от устройства, с которым идет DMA-обмен

// интерфейс ethernet шины
   input					e_rxc,      // Receive clock
   input					e_rxdv,     // Receive data valid
   input					e_rxer,     // Receive error
   input  [7:0]		e_rxd,      // Receive data
   input					e_crs,      // Carrier sense
   input					e_txc,      // Transmit clock
   output				e_txen,     // Tramsmit enable
   output				e_txer,     // Tramsmit error
   output [7:0]		e_txd,      // Transmit data
   output				e_rst,      // Hardware reset, active low
   output				e_mdc,      // MDC clock
   inout					e_mdio,     // MD line
   output				e_gtxc,		// 125 MHz

// переключатели
	input					s3,			// выбор режима
	input					s4,			// выбор дополнений

// Индикация
	output [2:0]		delqa_led_o
);

//************************************************
//* Шины данных, управление, индикация
wire [1:0]	adrmode_rx;	// Мультиплексирование адреса (DMA, proc, ether) канала приема
wire [1:0]	adrmode_tx;	// Мультиплексирование адреса (DMA, proc, ether) канала передачи
wire			eth_txmode;	// Костыль
wire			eth_rxmode;	// Костыль
wire [15:1] dma_lad;		// Внутренний адрес ПДП/DMA
wire [15:0]	bdl_dat;		// Данные регистров BDL
wire [9:0]  etxaddr;		// Регистр адреса блока передачи (память -> ether модуль)
wire [9:0]  erxaddr;		// Регистр адреса блока приема (ether модуль -> память)
wire [15:0] etxdbus;		// Шина данных блока передачи (память -> ether модуль)
wire [15:0] mtxdbus;		// Шина данных блока приема (DMA -> память)
wire [15:0]	mrxdat;		// Шина данных блока приема (память -> DMA)
wire [15:0] erxdbus;		// Шина данных блока приема (ether модуль -> память)
//
wire        mtxwe;
wire        erxwe;
wire			comb_res;	// Сигнал комбинированного сброса
wire [2:0]	indic;		// Сигналы индикации
assign delqa_led_o = indic;

//*******************************************************************
//* Буферная память канала приема
assign adrmode_rx = {eth_rxmode, dma_rxmode};
rxbuf mrxbuf(
	.wb_clk_i(lwb_clkp),
	.wb_adr_i(lwb_adr[11:1]),
	.wb_dat_i(lwb_out),
	.wb_dat_o(lrxb_dat),
	.wb_cyc_i(lwb_cyc),
	.wb_we_i(lwb_we),
	.wb_stb_i(lrxb_stb),
	.wb_ack_o(lrxb_ack),
	.dma_stb_i(drxbuf_stb),
	.dma_adr_i(dma_lad[11:1]),
	.dma_dat_o(mrxdat),
	.eth_adr_i(erxaddr[9:0]),
	.eth_dat_i(erxdbus),
	.eth_clk_i(rxclkb),
	.eth_we_i(erxwe),
	.adr_mode_i(adrmode_rx)
);

//*******************************************************************
//* Буферная память канала передачи
assign adrmode_tx = {eth_txmode, dma_txmode};
txbuf mtxbuf(
	.wb_clk_i(lwb_clkp),
	.wb_adr_i(lwb_adr[10:1]),
	.wb_dat_i(lwb_out),
	.wb_dat_o(ltxb_dat),
	.wb_cyc_i(lwb_cyc),
	.wb_we_i(lwb_we),
	.wb_stb_i(ltxb_stb),
	.wb_ack_o(ltxb_ack),
	.dma_stb_i(dtxbuf_stb),
	.dma_adr_i(dma_lad[10:1]),
	.dma_dat_i(mtxdbus),
	.dma_we_i(ldma_we),
	.eth_adr_i(etxaddr[9:0]),
	.eth_dat_o(etxdbus),
	.eth_clk_i(e_rxc),
	.adr_mode_i(adrmode_tx)
);

//*******************************************************************
// DMA
wire			dma_txmode, dma_rxmode, dma_mode;
wire [15:0]	ldibus;

// Сигнад записи по каналу ПДП/DMA
wire			ldma_we = dma_ack_i & dma_gnt & ~dma_we_o;

// Сигналы подтверждения  выбора периферии
wire			drxbuf_stb, dtxbuf_stb, dbdlbf_stb;
assign drxbuf_stb = (dma_lad[15:12] == 4'b0001)  & dma_mode;			// буфер данных канала приема (10000 - 20000)
assign dtxbuf_stb = (dma_lad[15:11] == 5'b00100)  & dma_mode;			// буфер данных канала передачи (20000 - 24000)
assign dbdlbf_stb = (dma_lad[15:4] == 12'b001010000000) & dma_mode;	// BDL regs (24000-24020)

// Мультиплексор входных шин данных
assign ldibus = (dbdlbf_stb ? bdl_dat : 16'o000000)
				  | (drxbuf_stb ? mrxdat  : 16'o000000);

dma dmamod(
   .clk_i(wb_clkp_i),			// тактовая частота шины
   .rst_i(comb_res),				// сброс
	.wb_adr_i(lwb_adr[3:1]),
	.wb_dat_i(lwb_out),
	.wb_dat_o(ldma_dat),
	.wb_cyc_i(lwb_cyc),
	.wb_we_i(lwb_we),
	.wb_sel_i(lwb_sel),
	.wb_stb_i(ldma_stb),
	.wb_ack_o(ldma_ack),
   .dma_req_o(dma_req),			// запрос DMA
   .dma_gnt_i(dma_gnt),			// подтверждение DMA
   .dma_adr_o(dma_adr_o),		// выходной адрес при DMA-обмене
   .dma_dat_i(dma_dat_i),		// входная шина данных DMA
   .dma_dat_o(dma_dat_o),		// выходная шина данных DMA
   .dma_stb_o(dma_stb_o),		// строб цикла шины DMA
   .dma_we_o(dma_we_o),			// направление передачи DMA (0 - память->модуль, 1 - модуль->память) 
   .dma_ack_i(dma_ack_i),		// Ответ от устройства, с которым идет DMA-обмен
	.lbdata_i(ldibus),
	.lbdata_o(mtxdbus),
   .dma_lad_o(dma_lad),
	.dma_txmode_o(dma_txmode),
	.dma_rxmode_o(dma_rxmode),
	.dma_mode_o(dma_mode)
);

//*******************************************************************
// Ethernet модуль
reg			mcasf;
reg			promf;
reg  [2:0]	sanity;
wire        rxclkb;     // Синхросигнал канала приема (запись в буферную память)
wire [10:0]	txcntb;		// Счетчик байтов передачи
wire [10:0] rxcntb;     // Счетчик байтов приема
wire [9:0]	emode;		// Режим работы модуля Ethernet
wire [7:0]	estse;		// Статус и ошибки приема/передачи
wire [15:0]	md_val_i;
wire [15:0] md_val_o;
wire [6:0]  md_ctrl;
wire [7:0]  md_status;
wire			mac_rdy;		// 
wire [47:0]	mac_data;	//
wire			cmp_done;	// 
wire			cmp_res;		// 

wire ereset;
ethreset ethrstm(
   .clk(wb_clkp_i),
   .rst(wb_rst_i),
   .e_reset(ereset)
);

ether etherm(
   .rst_i(ereset),
   .txcntb_i(txcntb),
	.ethmode_i(emode),
   .etxdbus_i(etxdbus),
   .etxaddr_o(etxaddr),
   .erxaddr_o(erxaddr),
   .erxdbus_o(erxdbus),
   .rxcntb_o(rxcntb),
   .erxwrn_o(erxwe),
   .rxclkb_o(rxclkb),
	.sts_errs_o(estse),
   .e_rxc(e_rxc),
   .e_rxdv(e_rxdv),
   .e_rxer(e_rxer),
   .e_rxd(e_rxd),
   .e_crs(e_crs),
   .e_txc(e_txc),
   .e_txen(e_txen),
   .e_txer(e_txer),
   .e_txd(e_txd),
   .e_rst(e_rst),
   .e_gtxc(e_gtxc),
   .md_clk(md_clock),
   .md_evt(md_evt),
   .e_mdio(e_mdio),
   .md_ctrl(md_ctrl),
   .md_val(md_val_o),
   .md_out(md_val_i),
   .md_status(md_status),
	.mac_rdy(mac_rdy),
	.mac_data(mac_data),
	.cmp_done(cmp_done),
	.cmp_res(cmp_res)
);

//*******************************************************************
// Внутренняя wishbone шина
wire			sys_init;				// шина сброса - от процессора к периферии
wire			dclo;						// сброс процессора
wire			aclo;						// прерывание по сбою питания
wire			lwb_clkp, lwb_clkn;
assign lwb_clkp = wb_clkp_i;		// синхронизация wishbone - от отрицательного синхросигнала / положительного = wb_clk
assign lwb_clkn = wb_clkn_i;		// синхронизация wishbone - от отрицательного синхросигнала / положительного = wb_clk
wire [15:0]	lwb_adr;					// шина адреса
wire [15:0]	lwb_out;					// вывод данных
wire [15:0]	lwb_mux;					// ввод данных
wire			lwb_cyc;					// начало цикла
wire			lwb_we;					// разрешение записи
wire [1:0]	lwb_sel;					// выбор байтов
wire			lwb_stb;					// строб транзакции
wire			lwb_ack;					// ответ от устройства
wire			lirq50;					// сигнал интервального таймера 50 Гц
// Шины векторного прерывания                                       
wire			lvm_istb;				// строб векторного прерывания
wire			lvm_iack;				// подтверждение векторного прерывания
wire [15:0]	lvm_ivec;				// вектор внешнего прерывания
wire			lvm_virq;				// запрос векторного прерывания

// сигналы выбора периферии
wire			lfrm_stb;
wire			lprg_stb;
wire			lrom_stb;
wire			ldma_stb;
wire			lreg_stb;
wire			lerg_stb;
wire			leth_stb;
wire			lrxb_stb;
wire			ltxb_stb;
wire			lbdl_stb;
wire			lcmp_stb;

// линии подтверждения обмена
wire			lfrm_ack;
wire			ldma_ack;
wire			lreg_ack;
wire			lrxb_ack;
wire			ltxb_ack;
wire			lbdl_ack;
wire			lcmp_ack;

// шины данных от периферии к процессору
wire [15:0]	lfrm_dat;
wire [15:0]	ldma_dat;
wire [15:0]	lreg_dat;
wire [15:0]	lrxb_dat;
wire [15:0]	ltxb_dat;
wire [15:0]	lcmp_dat;

//*******************************************************************
// Модуль формирования сбросов 
cpu_reset sysreset (
   .clk_i(lwb_clkp),
   .rst_i(comb_res),		// вход сброса
   .dclo_o(dclo),			// dclo - сброс от источника питания
   .aclo_o(aclo),			// aclo - прерывание по сбою питания
   .irq50_o(lirq50)		// сигнал интервального таймера 50 Гц
);

//*******************************************************************
// Процессор
am4_wb cpu(
	.vm_clk_p(lwb_clkp),	// positive edge clock
	.vm_clk_n(lwb_clkn),	// negative edge clock
	.vm_clk_ena(1'b0),	// slow clock enable
	.vm_clk_slow(1'b0),	// slow clock sim mode
	.vm_init(sys_init),	// peripheral reset output
	.vm_dclo(dclo),		// processor reset
	.vm_aclo(aclo),		// power fail notificaton
	.vm_halt(1'b0),		// halt mode interrupt
	.vm_evnt(lirq50),		// timer interrupt requests
	.vm_virq(1'b0),		// vectored interrupt request
	.wbm_gnt_i(1'b1),		// master wishbone granted
	.wbm_adr_o(lwb_adr),	// master wishbone address
	.wbm_dat_o(lwb_out),	// master wishbone data output
	.wbm_dat_i(lwb_mux),	// master wishbone data input
	.wbm_cyc_o(lwb_cyc),	// master wishbone cycle
	.wbm_we_o(lwb_we),	// master wishbone direction
	.wbm_sel_o(lwb_sel),	// master wishbone byte selection
	.wbm_stb_o(lwb_stb),	// master wishbone strobe
	.wbm_ack_i(lwb_ack),	// master wishbone acknowledgement
	.wbi_ack_i(vm_iack),	// interrupt vector acknowledgement
	.wbi_dat_i(16'b0),	// interrupt vector input
	.wbi_stb_o(lvm_istb),// interrupt vector strobe
// Режим начального пуска
//	00 - start reserved MicROM
//	01 - start from 173000
//	10 - break into ODT
//	11 - load vector 24
	.vm_bsel(2'b11)		// boot mode selector
);
reg vm_iack;
always @(posedge lwb_clkp or posedge wb_rst_i) begin
	if(wb_rst_i)
      vm_iack <= 1'b0;
   else
      vm_iack <= lvm_istb & ~vm_iack;	// в ответ на строб от процессора, а если строб уже был
													// выставлен в предыдущем  такте - то снимаем его
end

//*******************************************************************
//*  Сигналы управления внутренней шины wishbone
//******************************************************************* 
assign lprg_stb = lwb_stb & lwb_cyc & (lwb_adr[15:12] == 4'b0000);					// RAM 000000-010000
assign lrom_stb = lwb_stb & lwb_cyc & (lwb_adr[15:12] == 4'b1110);					// ROM 160000-167777
assign lcmp_stb = lwb_stb & lwb_cyc & (lwb_adr[15:4] == 12'b001010000101);			// регистры MAC - 24120 - 24136
assign lerg_stb = lwb_stb & lwb_cyc & (lwb_adr[15:4] == 12'b001010000100);			// внешние регистры - 24100 - 24116
assign lrxb_stb = lwb_stb & lwb_cyc & (lwb_adr[15:12] == 4'b0001);					// буфер данных канала приема (10000 - 20000)
assign ltxb_stb = lwb_stb & lwb_cyc & (lwb_adr[15:11] == 5'b00100);					// буфер данных канала передачи (20000 - 24000)
assign ldma_stb = lwb_stb & lwb_cyc & (lwb_adr[15:4] == 12'b001010000001);			// регистры ПДП/DMA 24020 - 24036
assign leth_stb = lwb_stb & lwb_cyc & (lwb_adr[15:5] == 11'b00101000001);			// ethernet регистры 24040 - 24076
assign lbdl_stb = lwb_stb & lwb_cyc & (lwb_adr[15:4] == 12'b001010000000);			// регистры BDL (24000-24016)

// Сигналы подтверждения - собираются через OR со всех устройств
assign lwb_ack	= lfrm_ack | lreg_ack | ldma_ack | lrxb_ack | ltxb_ack | lbdl_ack | lcmp_ack;

assign lfrm_stb = lprg_stb | lrom_stb;
assign lreg_stb = lerg_stb | leth_stb;
// Мультиплексор выходных шин данных всех устройств
assign lwb_mux	= (lfrm_stb ? lfrm_dat : 16'o000000)
					| (lbdl_stb ? bdl_dat  : 16'o000000)
					| (ldma_stb ? ldma_dat : 16'o000000)
					| (lreg_stb ? lreg_dat : 16'o000000)
					| (lrxb_stb ? lrxb_dat : 16'o000000)
					| (ltxb_stb ? ltxb_dat : 16'o000000)
					| (lcmp_stb ? lcmp_dat : 16'o000000)
;

//*******************************************************************
// Модуль управляющей программы
firmware prgram(
	.wb_clk_i(lwb_clkp),
	.wb_adr_i(lwb_adr),
	.wb_we_i(lwb_we),
	.wb_dat_i(lwb_out),
	.wb_dat_o(lfrm_dat),
	.wb_cyc_i(lwb_cyc),
	.prg_stb_i(lprg_stb),
	.rom_stb_i(lrom_stb),
	.wb_sel_i(lwb_sel),
	.wb_ack_o(lfrm_ack)
);

//*******************************************************************
//* Модуль BDL
bdl bdlm(
	.wb_clk_i(lwb_clkp),
	.wb_adr_i(lwb_adr[3:1]),
	.wb_dat_i(lwb_out),
	.wb_cyc_i(lwb_cyc),
	.wb_we_i(lwb_we),
	.wb_sel_i(lwb_sel),
	.wb_stb_i(lbdl_stb),
	.wb_ack_o(lbdl_ack),
	.dma_stb_i(dbdlbf_stb),
	.dma_adr_i(dma_lad[3:1]),
	.dma_dat_i(mtxdbus),
	.dma_we_i(ldma_we),
	.bdl_dat_o(bdl_dat)
);

//*******************************************************************
//* Модуль внешних регистров и регистров Ethernet
extregs eregs(
	.lwb_clk_i(lwb_clkp),
	.lwb_adr_i(lwb_adr[4:1]),
	.lwb_dat_i(lwb_out),
	.lwb_dat_o(lreg_dat),
	.lwb_cyc_i(lwb_cyc),
	.lwb_we_i(lwb_we),
	.lwb_sel_i(lwb_sel),
	.lrg_stb_i(lerg_stb),
	.lwb_ack_o(lreg_ack),
	.ewb_rst_i(wb_rst_i),
	.ewb_adr_i(wb_adr_i[2:0]),
	.ewb_dat_i(wb_dat_i),
	.ewb_dat_o(wb_dat_o),
	.ewb_cyc_i(wb_cyc_i),
	.ewb_we_i(wb_we_i),
	.ewb_sel_i(wb_sel_i),
	.erg_stb_i(wb_stb_i),
	.ewb_ack_o(wb_ack_o),
	.combres_o(comb_res),
	.iack_i(iack),
	.irq_o(irq),
	.s3_i(s3),
	.s4_i(s4),
//
	.let_stb_i(leth_stb),
	.e_txcntb_o(txcntb),
	.e_rxcntb_i(rxcntb),
	.e_mode_o(emode),
	.e_stse_i(estse),
	.e_mdval_o(md_val_o),
	.e_mdval_i(md_val_i),
	.e_mdctrl_o(md_ctrl),
	.e_mdstat_i(md_status),
	.eth_txmd_o(eth_txmode),
	.eth_rxmd_o(eth_rxmode),
	.santm_o(santm),
	.dev_ind_o(indic)
);

//*******************************************************************
//* Модуль сравнения MAC адресов
wire [2:0] epms = {emode[9], emode[8], emode[4]};
cmpmac cmpm(
	.wb_clk_i(lwb_clkp),
	.wb_adr_i(lwb_adr[3:1]),
	.wb_dat_i(lwb_out),
	.wb_dat_o(lcmp_dat),
	.wb_cyc_i(lwb_cyc),
	.wb_we_i(lwb_we),
	.wb_stb_i(lcmp_stb),
	.wb_sel_i(lwb_sel),
	.wb_ack_o(lcmp_ack),
	.eth_pms_i(epms),
	.eth_clk_i(e_rxc),
	.eth_macr_i(mac_rdy),
	.eth_macd_i(mac_data),
	.cmp_done_o(cmp_done),
	.cmp_res_o(cmp_res)
);


//*******************************************************************
// Генерация несущей для блока MD
wire md_clock;
wire md_evt;
mdc_clk mclk(
	.clock(wb_clkp_i),
	.rst(comb_res),
	.mdcclk(md_clock),
	.mdsevt(md_evt)
);
assign e_mdc = md_clock;

//************************************************
// Sanity timer
wire santm;				// Генерация BDCOK
santim santmod(
	.clock_i(md_clock),
	.gen_i(santm),
	.out_o(bdcok)
);

endmodule
