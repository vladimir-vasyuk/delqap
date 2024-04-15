//=================================================================================
// Реализация контроллера Ethernet на основе приемо-передатчика RTL8211EG
//---------------------------------------------------------------------------------
// Модуль приема кадра данных
//=================================================================================
module ethreceive(
	input					clk,			// Синхросигнал
	input					clr,			// Сигнал сброса
	input					rxena,		// Сигнал разрешения приема
	input      [7:0]	datain,		// Шина входных данных
	input					rxdv,			// Сигнал достоверности данных
	input					rxer,			// Сигнал ошибки
//	input 	  [47:0]	mymac,		// Шина MAC-адреса
	input      [31:0]	crc,			// Шина CRC
	input					rxdone,		// Сигнал подтверждения передачи данных
	output reg [10:0]	rxcntb,		// Счетчик принятых байтов
	output reg [9:0]	rxbaddr,		// Регистр адреса буферной памяти
	output reg [15:0]	rxbdata,		// Шины выходных данных
	output reg			rxwrn,		// Сигнал разрешения записи
	output reg			rxrdy,		// Сигнал завершения приема
	output reg			crcen,		// Cигнал разрешения вычисления CRC
	output reg			crcre,		// Cигнал сброса CRC
	output				err_gen,		// Сигнал общей ошибки
	output				err_crc		// Сигнал ошибки CRC
);

reg  [2:0]	ic;				// Счетчик
reg  [1:0]	bc;				// Счетчик байтов 
reg  [47:0]	recmac;			// Принятый MAC адрес
reg  [7:0]	bufdat;			// Буфер принятых данных
reg			rx_gen_error;	// Сигнал общей ошибки
reg			rx_crc_error;	// Сигнал ошибки CRC

assign {err_gen, err_crc} = {rx_gen_error, rx_crc_error};

// Конечный автомат канала приема
localparam IDLE		= 3'd0;
localparam SIX_55		= 3'd1;
localparam SPD_D5		= 3'd2;
localparam RX_DATA	= 3'd3;
localparam RX_LAST	= 3'd4;
localparam RX_CRC		= 3'd5;
localparam RX_FINISH	= 3'd6;
reg  [3:0]  rx_state;

initial
begin
   rx_state <= IDLE;
   rxrdy <= 1'b0;
   rx_gen_error <= 1'b0;
   rx_crc_error <= 1'b0;
end

// Основной блок
always@(negedge clk, posedge clr) begin
   if(clr) begin
      rx_state <= IDLE;		// Начальное состояние автомата
      rxrdy <= 1'b0;			// Сброс сигнала завершения приема
   end
   else begin
      case(rx_state)
         IDLE: begin			// Состояние ожидания
				crcen <= 1'b0;											// Сброс сигнала разрешения вычисления CRC
				crcre <= 1'b1;											// Установка сигнала сброса CRCC
				rxcntb <= 11'o0000;									// Начальное значения счетчика приема
				rxbaddr <= 10'o1777;									// Начальное значение регистра адреса буферной памяти
				rxwrn <= 1'b0;											// Сброс сигнала разрешения записи
				ic <= 3'o0;												// Сброс счетчика
				bc <= 2'o0;												// Сброс счетчика байтов
				if((rxdv == 1'b1) && (rxena == 1'b1)) begin	// Признак принятых данныхи и сигнал разрешения приема
					if(datain[7:0] == 8'h55) begin				// Данные преамбулы (0x55)?
						rx_gen_error <= 1'b0;						// Сброс сигнала общей ошибки
						rx_crc_error <= 1'b0;						// Сброс сигнала ошибки CRC
						rx_state<=SIX_55;								// Переход к приему преамбулы
					end
					else
						rx_state<=IDLE;
				end
			end
			SIX_55: begin		// Принять еще 6 байтов 0x55
				if(rxer == 1'b0) begin
					if ((datain[7:0] == 8'h55) & (rxdv == 1'b1)) begin
						if(ic == 3'd5) begin
							ic <= 3'd0; rx_state <= SPD_D5;
						end
						else
							ic <= ic + 1'd1;
					end
					else begin
						rx_gen_error <= 1'b1;
						rx_state<=IDLE;
					end
				end
				else begin
					rx_gen_error <= 1'b1;
					rx_state<=IDLE;
				end
			end
			SPD_D5: begin		// Принять байт разделитель (0xd5)
				if(rxer == 1'b0) begin
//					if((datain[7:0] == 8'hd5) && (rxdv == 1'b1) && (rxer == 1'b0)) begin
					if((datain[7:0] == 8'hd5) && (rxdv == 1'b1)) begin
						ic <= 3'd0; rx_state <= RX_DATA;
					end
					else begin
						rx_gen_error <= 1'b1;
						rx_state <= IDLE;
					end
				end
				else begin
					rx_gen_error <= 1'b1;
					rx_state <= IDLE;
				end
			end
			RX_DATA: begin		// Основные данные
				crcen <= 1'b1;											// Разрешить вычисление CRC
				crcre <= 1'b0;											// Убрать сигнал сброса CRC
				if(rxer == 1'b1) begin								// Входная ошибка?
					rx_gen_error <= 1'b1;							// Да - установить сигнал ошибки ...
					rx_state <= IDLE;									// ... и возврат в состояние ожидания
				end
				else begin
					if(rxdv == 1'b1) begin							// Есть разрешение приема данных?
						rxcntb <= rxcntb + 1'd1;					// Да- инкремент счетчика принятых данных.
						if(ic < 3'd6) begin							// Меньше 6 байт?
							recmac <= {recmac[39:0], datain[7:0]};	// Да - формирование MAC адреса назначения, ...
							ic <= ic + 1'd1;								// ... инкремент счетчика.
						end
						case(bc)
							2'o0: begin
								bufdat <= datain[7:0];				// Да - данные во внутренний буфер, ...
								bc <= bc + 1'b1;						// ... инкремент счетчика байтов, ...
								rxwrn <= 1'b0;							// ... запрет записи в буферную память.
							end
							2'o1: begin
//								rxbdata <= {bufdat[7:0], datain[7:0]};	// ... данные на шину буферной памяти, ...
								rxbdata <= {datain[7:0], bufdat[7:0]};	// ... данные на шину буферной памяти, ...
								rxwrn <= 1'b1;							// ... разрешение записи в буферную память, ...
								bc <= 2'o0;								// ... инкремент счетчика байтов, ...
								rxbaddr <= rxbaddr + 1'b1;			// ... инкремент регистра адреса ...
							end
						endcase
					end
					else begin											// Нет разрешения приема данных, ...
						rxwrn <= 1'b0;									// ... отключить запись в буфер, ...
						crcen<=1'b0;									// ... сброс сигнала разрешения вычисления CRC ...
						rx_state <= RX_LAST;							// ... и на запись оставшихся данных
					end
				end
			end
			RX_LAST: begin
				case(bc)
					2'o1: begin											// Если есть не записанные данные...
//						rxbdata <= {bufdat[7:0], 8'b0};			// ... записать 
						rxbdata <= {8'b0,bufdat[7:0]};			// ... записать 
						rxwrn <= 1'b1;
						bc <= 2'o0;
					end
					2'o0: begin											// Все данные записаны, ...
						rxwrn <= 1'b0;									// ... отключить запись в буфер, и  ...
						rx_state <= RX_CRC;							// ... переход на проверку контрольной суммы
					end
				endcase
			end
			RX_CRC: begin		// Проверка CRC
				rx_state <= RX_FINISH;								// На завершение
				rxcntb <= rxcntb - 11'd4;							// Минус 4 байта (CRC)
				if(crc != 32'hC704DD7B) begin						// CRC верен?
					rx_crc_error <= 1'b1;							// Нет - установить сигнал ошибки
				end
			end
			RX_FINISH: begin
				if(rxdone == 1'b1) begin							// Прием завершен?
					rxrdy <= 1'b0;										// Да, сброс сигнала завершения приема
					rx_state <= IDLE;									// Переход в состояние ожидания
				end
				else rxrdy <= 1'b1;									// Установка сигнала завершения приема
			end
			default: rx_state <= IDLE;
		endcase
	end
end

endmodule
