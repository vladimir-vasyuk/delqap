//=================================================================================
// Реализация контроллера Ethernet DELQA на основе процессора М4 (LSI-11M)
//=================================================================================
// Модуль внешних регистров и регистров Ethernet.
// Доступ к внешним регистрам с обеих шин
// Доступ к регистрам Ethernet только с внутренней
//=================================================================================
module extregs(
// внутренняя шина
	input				lwb_clk_i,	// тактовая частота шины
	input  [2:0]	lwb_adr_i,	// адрес 
	input  [15:0]	lwb_dat_i,	// входные данные
	output [15:0]	lwb_dat_o,	// выходные данные
	input				lwb_cyc_i,	// начало цикла шины
	input				lwb_we_i,	// разрешение записи (0 - чтение)
	input  [1:0]	lwb_sel_i,	// выбор байтов для записи 
	input				lrg_stb_i,	// строб цикла шины
	output			lwb_ack_o,	// подтверждение выбора устройства
// внешняя шина
	input				ewb_rst_i,	// сброс
	input  [2:0]	ewb_adr_i,	// адрес 
	input  [15:0]	ewb_dat_i,	// входные данные
	output [15:0]	ewb_dat_o,	// выходные данные
	input				ewb_cyc_i,	// начало цикла шины
	input				ewb_we_i,	// разрешение записи (0 - чтение)
	input  [1:0]	ewb_sel_i,	// выбор байтов для записи 
	input				erg_stb_i,	// строб цикла шины
	output			ewb_ack_o,	// подтверждение выбора устройства

	output			combres_o,	// сигнал комбинированного сброса
//	output			santmena_o,	// сигнал разрешения sanity timer
	input				iack_i,		// запрос обработки прерывания
	output			irq_o,		// подтверждение обработки прерывания
	input				s3_i,			// сигнал s3
	input				s4_i,			// сигнал s4
// Ethernet
	input				let_stb_i,	// строб выбора регистров ethernet
	output [9:0]	e_mode_o,	// управляющие сигналы для модуля ethernet
	input  [6:0]	e_stse_i,	// состояние модуля ethernet
	output [10:0]	e_txcntb_o,	// кол-во байт каналы передачи
	input	 [10:0]	e_rxcntb_i,	// кол-во байт каналы приема
	output [15:0]	e_mdval_o,	// данные для блока MDC
	input  [15:0]	e_mdval_i,	// данные из блока MDC
	output [6:0]	e_mdctrl_o,	// управляющие сигналы для блока MDC
	input  [7:0]	e_mdstat_i,	// состояние блока MDC
	output			eth_txmd_o,	// сигнал мультиплексера адреса ethernet буфера передачи
	output			eth_rxmd_o,	// сигнал мультиплексера адреса ethernet буфера приема
// Sanity timer
	output [1:0]	santm_o,
// Indication
	output [2:0]	dev_ind_o
);

reg  [15:0]	ldat, edat, etdat;
reg  [1:0]	lack, eack;
assign ewb_dat_o = edat;

assign lwb_dat_o = lrg_stb_i ? ldat : etdat;
assign lwb_ack_o = lrg_ack | let_ack;

//************************************************
// Сигналы упраления обменом по внутренней шине
wire lrg_ack;
wire lcyc = lwb_cyc_i & lrg_stb_i;
wire lstb = lcyc & ~lrg_ack;						// строб цикла шины
wire lread_req = lstb & bl & ~lwb_we_i;		// запрос чтения
wire lwrite_req = lstb & bl & lwb_we_i;		// запрос записи
// Сигналы упраления обменом по внешней  шине
wire ecyc = ewb_cyc_i & erg_stb_i;
wire estb = ecyc & ~ewb_ack_o;					// строб цикла шины
wire eread_req = estb & be & ~ewb_we_i;		// запрос чтения
wire ewrite_req = estb & be & ewb_we_i;		// запрос записи

//************************************************
// Формирование сигналов подтверждения выбора и приоритезации доступа к данным
// Доступ к внешним регистрам
reg bl;						// разрешение работы внутренней шины
reg be;						// разрешение работы внешней шины
wire block = be | bl;	// сигнал блокировки данных

always @ (posedge lwb_clk_i, posedge comb_res) begin
	if(comb_res) begin
		be = 1'b0; bl = 1'b0;
	end
	else begin
		if(ecyc & ~block)
			be = 1'b1;
		else if(lcyc & ~block)
			bl = 1'b1;
		if(~lcyc & bl) bl = 1'b0;
		if(~ecyc & be) be = 1'b0;
	end
end

always @ (posedge lwb_clk_i) begin
	if(bl) begin
		lack[0] <= lcyc;
		lack[1] <= lwb_cyc_i & lack[0];
	end
	else
		lack <= 2'b0;
	if(be) begin
		eack[0] <= ecyc;
		eack[1] <= ewb_cyc_i & eack[0];
	end
	else
		eack <= 2'b0;
end

assign lrg_ack = lwb_cyc_i & lrg_stb_i & lack[1];
assign ewb_ack_o = ewb_cyc_i & erg_stb_i & eack[1];

//************************************************
// Формирование сигналов подтверждения выбора
// Доступ к регистрам ethernet
wire let_ack;
wire lestb = lwb_cyc_i & let_stb_i & ~let_ack;		// строб цикла шины
wire lerd_req = lestb & ~lwb_we_i;						// запрос чтения
wire lewr_req = lestb & lwb_we_i;						// запрос записи

reg [1:0] etack;
always @(posedge lwb_clk_i) begin
	etack[0] <= lwb_cyc_i & let_stb_i;
	etack[1] <= lwb_cyc_i & etack[0];
end
assign let_ack = lwb_cyc_i & let_stb_i & etack[1];

//************************************************
// Регистр управления/состояния - csr - 174456
//
reg			csr_ri = 1'b0;		// 15	Receive Interrupt Request (RW1)
//reg			csr_pe = 1'b0;		// 14	Parity Error in Memory (RO)
wire			csr_ca;				// 13	Carrier from Receiver Enabled (RO)
//reg			csr_ok = 1'b1;		// 12	Ethernet Transceiver Power OK (RO)
//reg			csr_rr = 1'b0;		// 11	reserved
reg			csr_se = 1'b0;		// 10	Sanity Timer Enable (RW)
reg			csr_el = 1'b0;		// 09	External  Loopback (RW)
reg			csr_il = 1'b0;		// 08	Internal Loopback (RW) active low
reg			csr_xi = 1'b0;		// 07	Transmit Interrupt Request (RW1)
reg			csr_ie = 1'b0;		// 06	Interrupt Enable (RW)
reg			csr_rl = 1'b1;		// 05	Receive List Invalid/Empty (RO)
reg			csr_xl = 1'b1;		// 04	Transmit List Invalid/Empty (RO)
reg			csr_bd = 1'b0;		// 03	Boot/Diagnostic ROM load (RW)
reg			csr_ni = 1'b0;		// 02	Nonexistance-memory timeout Interrupt (RO)
reg			csr_sr = 1'b0;		// 01	Software Reset (RW)
reg			csr_re = 1'b0;		// 00	Receiver Enable (RW)
wire [15:0] csr;
assign csr_ca = (~csr_il)? 1'b0 : 1'b1;//(~errs[4]);
assign csr = {csr_ri,1'b0,1'b0,1'b1,1'b0,csr_se,csr_el,csr_il,csr_xi,csr_ie,csr_rl,csr_xl,csr_bd,csr_ni,csr_sr,csr_re};

//************************************************
// Регистр адреса ветора - var - 174454
//
reg			var_ms;				// Mode select (RW) (After power-up reset reflect s3)
//reg			var_os;				// Option switch (s4) settings (RO) (After power-up reset reflect s4)
reg			var_rs = 1'b1;		// Request self-test (RW)
reg			var_s3 = 1'b1;		// Self test status (RO)
reg			var_s2 = 1'b1;		// Self test status (RO)
reg			var_s1 = 1'b1;		// Self test status (RO)
reg  [7:0]	var_iv;				// Interrupt vector
//reg			var_rr;				//
reg			var_id = 1'b0;		// Identity test bit
reg			blkreg = 1'b0;
wire [15:0]	vareg;
assign vareg = {var_ms,s4_i,var_rs,var_s3,var_s2,var_s1,var_iv[7:0],1'b0,var_id};

//************************************************
// Регистр адреса блока приема (RBDL) - 174444, 174446
//
reg  [15:1]	rbdl_lwr;			// low address bits
reg  [5:0]	rbdl_hir;			// high address bits

//************************************************
// Регистр адреса блока передачи (TBDL) - 174450, 174452
//
reg  [15:1]	tbdl_lwr;			// low address bits
reg  [5:0]	tbdl_hir;			// high address bits

wire			blkbus;
wire			allow_bus_ops = ~blkbus & ~blkreg;

//************************************************
// Блок формирования комбинированного сброса
//
//reg			busy;					// регистр индикации занятости
wire			res_soft;			// сигнал программного сброса
wire        comb_res = res_soft | ewb_rst_i;	// сигнал комбинированного сброса
assign combres_o = comb_res;

soft_reset sftresm(
	.clk(lwb_clk_i),
	.rst(ewb_rst_i),
	.csr_sr(csr_sr),
	.block(blkbus),
	.reset(res_soft)
);

//************************************************
// MAC address ROM
//
wire        sa_rom_chk;       // Checksum signal
wire [63:0] macval;
reg  [2:0]  maddr;
assign sa_rom_chk = csr_el & (~csr_bd) & (~csr_re);
small_rom sarom(
   .clk(lwb_clk_i),
//	 .addr(maddr),
   .q(macval)
);

//************************************************
// Модуль обработки прерывания (внешняя шина)
//
reg	fint;		// выделение фронта сигнала
wire	wint_req = csr_ri | csr_xi;
wire	sint_req = ~fint & wint_req;

always @(posedge lwb_clk_i)
	fint <= wint_req;

bus_int inter(
	.clk_i(lwb_clk_i),
	.rst_i(comb_res),
	.ena_i(csr_ie),
	.req_i(sint_req),
	.ack_i(iack_i),
	.irq_o(irq_o)
);

//************************************************
// Режимы работы модуля Ethernet
//************************************************
reg			rxdon;		// Данные приняты
reg			txrdy;		// Данные готовы к передаче
reg			stpac;		// Установочный пакет (setup)
reg			skipb;		// Пропуск байта
reg			mcast;		// Режим широковещания
reg			promis;		// Режим прослушивания
wire        intmode;		// Internal loopback
wire        intextmode;	// Internal extended loopback
wire        extmode;		// External loopback
wire        rxmode;		// Разрешение приема пакета
wire			bdrom;		// Загрузка BDROM
assign intmode = (~csr_il) & (~csr_el) & (~csr_re);
assign intextmode = (~csr_il) & csr_el & (~csr_re) & (~csr_bd);
assign extmode = csr_il & csr_el & (~csr_re);
assign rxmode = csr_re & (~csr_rl);
assign bdrom = (~csr_il) & csr_el & (~csr_re) & csr_bd;

wire [7:0]	e_mode;			// Регистр режима работы 				-- 24040
assign e_mode = {rxdon, txrdy, skipb, stpac, extmode, intextmode, intmode, rxmode};
// wire [7:0]	e_sts_errs;	// статус и ошибки приема/передачи	-- 24040
// {1'b0, rxrdy, txdone, crs_err, mdc_err, e_txer, rx_err, rx_crc_err}
reg  [10:0]	e_txcntb;		// Регистры кол-ва байт 				-- 24042 (прием/передача)
//wire [10:0]	e_rxcntb;	// Регистр кол-ва принятых байт		-- 24042
reg  [15:0]	e_mdval;			// входные/выходные данные MD 		-- 24044
reg  [6:0]	e_mdctrl;		// сигналы управления MD 				-- 24046
reg			e_mdmux = 1'b0;// мультиплексер данных MD
// управление (e_mdctrl):
//		6:		1/0 - write/read
//		5:		1 - start
//		4:0	reg. address
// статус (e_mdstat_i)
//		7: 	1/0 - ready/busy
//		6,5:	10-1000Мб/с; 01-100Мб/с; 00-10Мб/с; 11-зарезервированно
//		4:		1-полный дуплекс; 0-полудуплекс
//		3:		зарезервированно (0)
//		2:		1-MDI crossover; 0-MDI
//		1:		1-приемник готов; 0-приемник не готов
//		0:		1-связь есть; 0-связи нет

// 24050 - регистр общего назначения
// чтение - (bdrom, 8'b0,  promis, mcast, eadr_mod[1:0], leds[2:0])
// запись - (8'b0, santm_res, promis, mcast, eadr_mod[1:0], leds[2:0])
reg			santm_res;			// генерация BDCOK
reg  [1:0]	eadr_mod;			// подключение к адресной шине ethernet
reg  [2:0]	leds;					// индикация

assign santm_o = {csr_se, santm_res};
assign dev_ind_o = leds;
assign e_txcntb_o = e_txcntb;
assign e_mode_o = {promis, mcast, e_mode[7:0]};
assign e_mdval_o = e_mdval;
assign e_mdctrl_o = e_mdctrl;
assign eth_rxmd_o = (stpac | extmode | intextmode | intmode | rxmode) & eadr_mod[1];
assign eth_txmd_o = txrdy & eadr_mod[0];


//************************************************
// Работа с регистрами
//
//always @(posedge lwb_clk_i, posedge comb_res) begin
always @(posedge lwb_clk_i) begin
	// Сброс регистров
   if(comb_res) begin
      // Сброс регистра управления
      csr_ri  <= 1'b0; csr_se <= 1'b0; csr_el <= 1'b0;
      csr_il <= 1'b0; csr_xi <= 1'b0; csr_ie <= 1'b0; csr_rl <= 1'b1; csr_xl <= 1'b1;
      csr_bd <= 1'b0; csr_ni <= 1'b0; csr_sr <= 1'b0; csr_re <= 1'b0;
      // Сброс регистра вектора
      if(~res_soft) begin	// Не программный сброс
         var_id <= 1'b0; var_iv <= 8'o0; var_rs  <= 1'b1;
         var_s3 <= 1'b1; var_s2 <= 1'b1; var_s1 <= 1'b1;
         var_ms <= s3_i;// var_os <= s4_i;
      end
		// Сброс регистров MD
		e_mdmux <= 1'b0;
	end
	else begin
		// -----------------------------
		// Внешняя шина
		// Чтение регистров
		if (eread_req) begin
			case (ewb_adr_i[2:0])
				3'b000: begin	// Base + 00
					if (sa_rom_chk)
						edat <= {8'hFF, macval[55:48]};
					else
						edat <= e_mdmux? {8'h00, e_mdstat_i[7:0]} : {8'hFF, macval[7:0]};
				end
				3'b001: begin	// Base + 02
					if (sa_rom_chk)
						edat <= {8'hFF, macval[63:56]};
					else
						edat <= e_mdmux? {9'h00, e_stse_i[6:0]} : {8'hFF, macval[15:8]};
				end
				3'b010: begin	// Base + 04
					edat <= {8'hFF, macval[23:16]};
				end
				3'b011: begin	// Base + 06
					edat <= {8'hFF, macval[31:24]};
				end
				3'b100: begin	// Base + 10
					edat <= {8'hFF, macval[39:32]};
				end
				3'b101: begin	// Base + 12
					edat <= {8'hFF, macval[47:40]};
				end
				3'b110: begin	// Base + 14 - VAR
					edat <= vareg;
				end
				3'b111: begin	// Base + 16 - CSR
					edat <= csr;
				end
			endcase 
		end
		//-----------------------------
		// Запись регистров
		else if (ewrite_req) begin
			if (ewb_sel_i[0]) begin   // Запись младшего байта
				case (ewb_adr_i[2:0])
					3'b000:			// Base + 00 - MD
						if(allow_bus_ops) e_mdmux <= ewb_dat_i[7];
//					3'b001: begin	// Base + 02
//					end
					3'b010: 			// Base + 04 - RBDL low
						if(allow_bus_ops) rbdl_lwr[7:1] <= ewb_dat_i[7:1];
					3'b011:			// Base + 06 - RBDL high
						if(allow_bus_ops) rbdl_hir[5:0] <= ewb_dat_i[5:0];
					3'b100:			// Base + 10 - TBDL low
						if(allow_bus_ops) tbdl_lwr[7:1] <= ewb_dat_i[7:1];
					3'b101:			// Base + 12 - TBDL high
						if(allow_bus_ops) tbdl_hir[5:0] <= ewb_dat_i[5:0];
					3'b110: begin	// Base + 14 - VAR
						if(allow_bus_ops) begin
							var_id <= ewb_dat_i[0];
							var_iv[5:0] <= ewb_dat_i[7:2];
						end
					end
					3'b111: begin  // Base + 16 - CSR
						if(allow_bus_ops) begin
							csr_ie <= ewb_dat_i[6];
							if(ewb_dat_i[7] == 1'b1) begin
								csr_xi <= 1'b0;
								csr_ni <= 1'b0;
							end
							csr_re <= ewb_dat_i[0];
							// Только для PDP-11. Для алгоритма смотри доку
							csr_bd <= ewb_dat_i[3];
						end
						csr_sr <= ewb_dat_i[1];  // 1 - 0 => программный сброс
					end
				endcase
			end
			if(ewb_sel_i[1]) begin    // Запись старшего байта
				if(allow_bus_ops) begin
					case (ewb_adr_i[2:0])
						3'b010: begin  // Base + 04 - RBDL low
							rbdl_lwr[15:8] <= ewb_dat_i[15:8];
						end
						3'b011: begin  // Base + 06 - RBDL high
							csr_rl <= 1'b0;
						end
						3'b100: begin  // Base + 10 - TBDL low
							tbdl_lwr[15:8] <= ewb_dat_i[15:8];
						end
						3'b101: begin  // Base + 12 - TBDL high
							csr_xl <= 1'b0;
						end
						3'b110: begin  // Base + 14 - VAR
							var_iv[7:6] <= ewb_dat_i[9:8];
							var_rs <= ewb_dat_i[13];
							var_ms <= ewb_dat_i[15];
						end
						3'b111: begin  // Base + 16 - CSR
							csr_il <= ewb_dat_i[8];
							csr_el <= ewb_dat_i[9];
							csr_se <= ewb_dat_i[10];
							if(ewb_dat_i[15] == 1'b1) csr_ri <= 1'b0;
						end
					endcase
				end
			end
		end
		// -----------------------------
		// Внутренняя шина
      // Чтение регистров 
      if (lread_req) begin
         case (lwb_adr_i[2:0])
				3'b000: begin	// Base + 00
					ldat <= {15'b0, blkreg};
				end
//				3'b001: begin	// Base + 02
//				end
				3'b010: begin	// Base + 04 - RBDL low
					ldat <= {rbdl_lwr[15:1], 1'B0};
				end
            3'b011: begin	// Base + 06 - RBDL high
               ldat <= {10'b0, rbdl_hir[5:0]};
            end
            3'b100: begin	// Base + 10 - TBDL low
               ldat <= {tbdl_lwr[15:1], 1'B0};
            end
            3'b101: begin	// Base + 12 - TBDL high
               ldat <= {10'b0, tbdl_hir[5:0]};
            end
            3'b110: begin	// Base + 14 - VAR
               ldat <= vareg;
            end
            3'b111: begin	// Base + 16 - CSR
               ldat <= csr;
            end
         endcase 
      end
		//-----------------------------
		// Запись регистров
		else if (lwrite_req) begin
			if (lwb_sel_i[0]) begin   // Запись младшего байта
				case (lwb_adr_i[2:0])
					3'b000: begin	// Base + 00
						blkreg <= lwb_dat_i[0];
					end
					3'b111: begin  // Base + 16 - CSR
						csr_ni <= lwb_dat_i[2];
						csr_xl <= lwb_dat_i[4];
						csr_rl <= lwb_dat_i[5];
						csr_xi <= lwb_dat_i[7];
					end
				endcase
			end
			if(lwb_sel_i[1]) begin    // Запись старшего байта
				case (lwb_adr_i[2:0])
					3'b110: begin  // Base + 14 - VAR
						var_s1 <= lwb_dat_i[10];
						var_s2 <= lwb_dat_i[11];
						var_s3 <= lwb_dat_i[12];
						var_rs <= lwb_dat_i[13];
					end
					3'b111: begin  // Base + 16 - CSR
						csr_ri <= lwb_dat_i[15];
					end
				endcase
			end
		end
	end
end

//************************************************
// Работа с регистрами ethernet
//
always @(posedge lwb_clk_i, posedge comb_res) begin
   if(comb_res) begin 
		// Сброс регистров MD
		e_mdctrl <= 7'b0;
	end
	else if (lerd_req == 1'b1) begin
		case (lwb_adr_i[2:0])
			3'b000:	// 24040
				etdat <= {1'b0 , e_stse_i[6:0], e_mode[7:0]};
			3'b001:	// 24042
				etdat <= {5'b0, e_rxcntb_i[10:0]};
			3'b010:	// 24044
				etdat <= e_mdval_i;
			3'b011:	// 24046
				etdat <= {e_mdstat_i[7:0], 1'b0, e_mdctrl[6:0]};
			3'b100:	// 24050
				etdat <= {bdrom, 8'b0, promis, mcast, eadr_mod[1:0], leds[2:0]};
//			3'b110:	// 24054
//			3'b111:	// 24056
		endcase
	end
	// Запись регистров
	else if (lewr_req == 1'b1) begin
		if (lwb_sel_i[0] == 1'b1) begin   // Запись младшего байта
			case (lwb_adr_i[2:0])
				3'b000:			// 24040
					{rxdon, txrdy, skipb, stpac} <= lwb_dat_i[7:4];
				3'b001:			// 24042
					e_txcntb[7:0] <= lwb_dat_i[7:0];
				3'b010:			// 24044
					e_mdval[7:0] <= lwb_dat_i[7:0];
				3'b011:			// 24046
					e_mdctrl[6:0] <= lwb_dat_i[6:0];
				3'b100: begin	// 24050
					santm_res <= lwb_dat_i[7];
					promis <= lwb_dat_i[6];
					mcast <= lwb_dat_i[5];
					eadr_mod[1:0] <= lwb_dat_i[4:3];
					leds[2:0] <= lwb_dat_i[2:0];
				end
//				3'b110: begin	// 24054
//				end
//				3'b111: begin	// 24056
//				end
			endcase
		end
		if(lwb_sel_i[1] == 1'b1) begin    // Запись старшего байта
			case (lwb_adr_i[2:0])
				3'b001:	// 24042
					e_txcntb[10:8] <= lwb_dat_i[10:8];
				3'b010:	// 24044
					e_mdval[15:8] <= lwb_dat_i[15:8];
			endcase
		end
	end
	else begin
		if(~e_stse_i[6] & rxdon)			// Сброс сигнала принятых данных
			rxdon <= 1'b0;
		if(~e_mdstat_i[7] & e_mdctrl[5])	// Сброс сигнала старта MDC
			e_mdctrl[5] <= 1'b0;
	end
end

endmodule
