//=================================================================================
// Реализация контроллера Ethernet на основе приемо-передатчика RTL8211EG
//---------------------------------------------------------------------------------
// Модуль MAC адреса
//=================================================================================
module small_rom
#(parameter FILE = "../../hdl/delqap/sarom.txt")
(
//	input				clk,
//	input  [1:0]	addr,
//	output [15:0]	q
	output [63:0]	q
);

reg  [7:0]	mem[7:0];
//wire [1:0]	adr;
assign q = {mem[7], mem[6], mem[5], mem[4], mem[3], mem[2], mem[1], mem[0]};
//assign q = mem[addr];

initial begin
   $readmemh(FILE, mem);
end

endmodule
