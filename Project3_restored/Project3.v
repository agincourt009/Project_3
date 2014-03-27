module Project3(SW,KEY,LEDR,LEDG,HEX0,HEX1,HEX2,HEX3,CLOCK_50);
  input  [9:0] SW;
  input  [3:0] KEY;
  input  CLOCK_50;
  output [9:0] LEDR;
  output [7:0] LEDG;
  output [6:0] HEX0,HEX1,HEX2,HEX3;

  parameter DBITS    =32;
  parameter INSTSIZE =32'd4;
  parameter INSTBITS =32;
  parameter STARTPC  =32'h40;
  parameter REGNOBITS=4;
  parameter IMMBITS=16;
  parameter ADDRKEY  =32'hF0000010;
  parameter ADDRSW   =32'hF0000014;
  parameter ADDRHEX  =32'hF0000000;
  parameter ADDRLEDR =32'hF0000004;
  parameter ADDRLEDG =32'hF0000008;
  parameter IMEMINITFILE="Sorter2.mif";
  parameter IMEMADDRBITS=13;
  parameter IMEMWORDBITS=2;
  parameter IMEMWORDS=2048;
  parameter DMEMINITFILE="DataMem.mif";
  parameter DMEMADDRBITS=13;
  parameter DMEMWORDBITS=2;
  parameter DMEMWORDS=2048;
  parameter OP1_ALUR =4'b0000;
  parameter OP1_ALUI =4'b1000;
  parameter OP1_CMPR =4'b0010;
  parameter OP1_CMPI =4'b1010;
  parameter OP1_BCOND=4'b0110;
  parameter OP1_SW   =4'b0101;
  parameter OP1_LW   =4'b1001;
  parameter OP1_JAL  =4'b1011;
  
  parameter OP2_ALU_ADD =4'b0000;
  parameter OP2_ALU_SUB =4'b0001;
  parameter OP2_ALU_AND =4'b0100;
  parameter OP2_ALU_OR  =4'b0101;
  parameter OP2_ALU_XOR =4'b0110;
  parameter OP2_ALU_NAND=4'b1100;
  parameter OP2_ALU_NOR =4'b1101;
  parameter OP2_ALU_NXOR=4'b1110;
  parameter OP2_ALU_MVHI=4'b1011;

  parameter OP2_CMP_F   =4'b0000;
  parameter OP2_CMP_EQ  =4'b0001;
  parameter OP2_CMP_LT  =4'b0010;
  parameter OP2_CMP_LTE =4'b0011;
  parameter OP2_CMP_EQZ =4'b0101;
  parameter OP2_CMP_LTZ =4'b0110;
  parameter OP2_CMP_LTEZ=4'b0111;
  parameter OP2_CMP_T   =4'b1000;
  parameter OP2_CMP_NE  =4'b1001;
  parameter OP2_CMP_GTE =4'b1010;
  parameter OP2_CMP_GT  =4'b1011;
  parameter OP2_CMP_NEZ =4'b1101;
  parameter OP2_CMP_GTEZ=4'b1110;
  parameter OP2_CMP_GTZ =4'b1111;

  wire clk,lock;
  Pll pll(.inclk0(CLOCK_50),.c0 (clk),.locked(lock));
  wire reset=!lock;
  
  // The PC register and update logic
  reg  [(DBITS-1):0] PC;
  always @(posedge clk) begin
    if(reset)
	   PC<=STARTPC;
	 else if(mispred_B)
	   PC<=pcgood_B;
	 else if(!stall_F)
      PC<=pcpred_F;
  end
  // This is the value of "incremented PC", computed in stage 1
  wire [(DBITS-1):0] pcplus_F=PC+INSTSIZE;
  // This is the predicted value of the PC
  // that we used to fetch the next instruction
	wire [(DBITS-1):0] pcpred_F=pcplus_F;

  // Instruction-fetch
  (* ram_init_file = IMEMINITFILE *)
  reg [(DBITS-1):0] imem[(IMEMWORDS-1):0];
  wire [(DBITS-1):0] inst_F=imem[PC[(IMEMADDRBITS-1):IMEMWORDBITS]];
	// If fetch and decoding stages are the same stage,
	// just connect signals from fetch to decode
	wire [(DBITS-1):0] inst_D=inst_F;
	wire [(DBITS-1):0] pc_D=PC;
	wire [(DBITS-1):0] pcplus_D=pcplus_F;
	wire [(DBITS-1):0] pcpred_D=pcpred_F;
	// Instruction decoding
	// These have zero delay from inst_D
	// because they are just new names for those signals
	wire [3:0] op1_D    =inst_D[31:28];
	wire [3:0] rd_D     =inst_D[27:24];
	wire [3:0] rs_D     =inst_D[23:20];
	wire [3:0] rt_D     =inst_D[19:16];
	wire [3:0] op2_i_D  =inst_D[ 3: 0];
	wire [3:0] op2_d_D  =rd_D;
	wire [3:0] op2_t_D  =rt_D;
	wire [(IMMBITS-1):0] rawimm_D=inst_D[(IMMBITS-1):0];
	// Register-read
//	(* ram_init_file = "Regs.mif" *)
	(* ram_init_file = "Regs.mif", ramstyle="logic" *)
	reg [(DBITS-1):0] regs[31:0];
	// Two read ports, always using rx and ry for register numbers
	wire [4:0] rregno1_D=rs_D, rregno2_D=rt_D, memvaladdr = rt_D;
	wire [(DBITS-1):0] regval1_D=regs[rregno1_D];
	wire [(DBITS-1):0] regval2_D=regs[rregno2_D];
	wire [(DBITS-1):0] sxtimm = {{(DBITS-IMMBITS){rawimm_D[IMMBITS-1]}},rawimm_D};
 
	reg LdMem, aluimm_D, isbranch_D;
	wire [(DBITS-1):0]memaddr;
	wire [(DBITS-1):0] aluin1_A = regval1_D;
	wire [(DBITS-1):0] aluin2_A;
	reg [4:0] alufunc_A;
	assign aluin2_A = aluimm_D?sxtimm: regval2_D;
	// Now the actual ALU
	reg signed [(DBITS-1):0] aluout_A;
	always @(alufunc_A or aluin1_A or aluin2_A)
	case(alufunc_A)
		{1'b0,OP2_ALU_ADD }: aluout_A=aluin1_A+aluin2_A;
		{1'b0,OP2_ALU_SUB }: aluout_A=aluin1_A-aluin2_A;
		{1'b0,OP2_ALU_AND }: aluout_A=aluin1_A&aluin2_A;
		{1'b0,OP2_ALU_OR  }: aluout_A=aluin1_A|aluin2_A;
		{1'b0,OP2_ALU_XOR }: aluout_A=aluin1_A^aluin2_A;
		{1'b0,OP2_ALU_NAND}: aluout_A=~(aluin1_A&aluin2_A);
		{1'b0,OP2_ALU_NOR }: aluout_A=~(aluin1_A|aluin2_A);
		{1'b0,OP2_ALU_NXOR}: aluout_A=~(aluin1_A^aluin2_A);
		{1'b0,OP2_ALU_MVHI}: aluout_A={aluin2_A[15:0],16'b0};
		{1'b1,OP2_CMP_F   }: aluout_A={31'b0,1'b0};
		{1'b1,OP2_CMP_EQ  }: aluout_A={31'b0,aluin1_A==aluin2_A};
		{1'b1,OP2_CMP_LT  }: aluout_A={31'b0,aluin1_A< aluin2_A};
		{1'b1,OP2_CMP_LTE }: aluout_A={31'b0,aluin1_A<=aluin2_A};
		{1'b1,OP2_CMP_EQZ }: aluout_A={31'b0,aluin1_A==32'b0};
		{1'b1,OP2_CMP_LTZ }: aluout_A={31'b0,aluin1_A< 32'b0};
		{1'b1,OP2_CMP_LTEZ}: aluout_A={31'b0,aluin1_A<=32'b0};
		{1'b1,OP2_CMP_T   }: aluout_A={31'b0,1'b1};
		{1'b1,OP2_CMP_NE  }: aluout_A={31'b0,aluin1_A!=aluin2_A};
		{1'b1,OP2_CMP_GTE }: aluout_A={31'b0,aluin1_A>=aluin2_A};
		{1'b1,OP2_CMP_GT  }: aluout_A={31'b0,aluin1_A> aluin2_A};
		{1'b1,OP2_CMP_NEZ }: aluout_A={31'b0,aluin1_A!=32'b0};
		{1'b1,OP2_CMP_GTEZ}: aluout_A={31'b0,aluin1_A>=32'b0};
		{1'b1,OP2_CMP_GTZ }: aluout_A={31'b0,aluin1_A> 32'b0};
	default:  aluout_A={DBITS{1'bX}};
	endcase

	assign memaddr = LdMem? aluout_A:{DBITS{1'bX}} ;
	reg dobranch_A, isjump_A;
	wire [(DBITS-1):0] brtarg_A,jmptarg_A;
	wire [(DBITS-1):0] incpc = pcpred_F;
	wire [(DBITS-1):0] multpc = 4*pcpred_F;
	reg [(DBITS-1):0] brtarg;
	always @(incpc or multpc)
		brtarg = incpc+multpc;
	always @(isbranch_D or aluout_A)
		if(isbranch_D)
			dobranch_A <= aluout_A[0];
		else
			dobranch_A <= 1'b0;
	assign brtarg_A = brtarg;
	assign jmptarg_A = isjump_A?aluout_A: {DBITS{1'bX}};
	wire [(DBITS-1):0] pcgood_B=
		dobranch_A?brtarg_A:
		isjump_A?jmptarg_A:
		pcplus_F;
	wire mispred_B=(pcgood_B!=incpc)&&!isnop_A;

	reg stall_F, flush_D, isnop_A;
	always @ (dobranch_A or isjump_A)begin
		stall_F = 1'b0;
		flush_D = 1'b0;
		if(isjump_A)
			stall_F <= 1'b1;
		else if (dobranch_A)
			flush_D <= 1'b1;
	end
	
	wire [(DBITS-1):0] wmemval_M = regs[memvaladdr];
	reg wrmem_M, wrreg_M;
	wire [(DBITS-1):0]memaddr_M = memaddr;
	
	// Create and connect HEX register
	reg [15:0] HexOut;
	reg [9:0] tempR;
	reg [7:0] tempG;
	SevenSeg ss3(.OUT(HEX3),.IN(HexOut[15:12]));
	SevenSeg ss2(.OUT(HEX2),.IN(HexOut[11:8]));
	SevenSeg ss1(.OUT(HEX1),.IN(HexOut[7:4]));
	SevenSeg ss0(.OUT(HEX0),.IN(HexOut[3:0]));
	always @(posedge clk or posedge reset)
		if(reset)
			HexOut<=16'hDEAD;
		else if(wrmem_M&&(memaddr_M==ADDRHEX))
			HexOut<=wmemval_M[15:0];
		else if(wrmem_M&&(memaddr_M==ADDRLEDR))
		   tempR<=wmemval_M[9:0];
		else if(wrmem_M&&(memaddr_M==ADDRLEDG))
		   tempG<=wmemval_M[7:0];	

	assign LEDR = tempR;
	assign LEDG = tempG;
	// Now the real data memory
	wire MemEnable=!(memaddr_M[(DBITS-1):DMEMADDRBITS]);
	wire MemWE=lock&wrmem_M&MemEnable;
	(* ram_init_file = DMEMINITFILE, ramstyle="no_rw_check" *)
	reg [(DBITS-1):0] dmem[(DMEMWORDS-1):0];
	always @(posedge clk)
		if(MemWE)
			dmem[memaddr_M[(DMEMADDRBITS-1):DMEMWORDBITS]]<=wmemval_M;
	wire [(DBITS-1):0] MemVal=MemWE?{DBITS{1'bX}}:dmem[memaddr_M[(DMEMADDRBITS-1):DMEMWORDBITS]];
	// Connect memory and input devices to the bus
	wire [(DBITS-1):0] memout_M=
		MemEnable?MemVal:
		(memaddr_M==ADDRKEY)?{12'b0,~KEY}:
		(memaddr_M==ADDRSW)? { 6'b0,SW}:
		32'hDEADBEEF;

	// TODO: Decide what gets written into the destination register (wregval_M)
	
	reg [(DBITS-1):0] wregval_M, wregno_M;
	always @(posedge clk)
		if(wrreg_M&&lock)
			regs[wregno_M]<=wregval_M;

  // Decoding logic
  reg isnop_D, selaluout_D, selmemout_D, selpcplus_D;
  reg [(DBITS-1):0] wregno_D;
  always @(inst_D or op1_D or op2_i_D or op2_d_D or op2_t_D or rd_D or reset or flush_D) begin
    {aluimm_D,alufunc_A, LdMem}=
	 {    1'bX,    5'hXX, 1'bX};
	 {isbranch_D,isjump_A,isnop_D,wrmem_M}=
	 {      1'b0,    1'b0,   1'b0,   1'b0};
    {selaluout_D,selmemout_D,selpcplus_D,wregno_D,wrreg_M}=
	 {       1'bX,       1'bX,       1'bX,    rd_D,   1'b0};
	 if(reset|flush_D)
		isnop_D=1'b1;
	 else
	 case(op1_D)
	   OP1_ALUR:
				{aluimm_D,alufunc_A     ,selaluout_D,selmemout_D,selpcplus_D,wrreg_M}=
				{    1'b0,{1'b0,op2_i_D},       1'b1,       1'b0,       1'b0,   1'b1};
	   OP1_ALUI:
				{aluimm_D,alufunc_A     ,selaluout_D,selmemout_D,selpcplus_D,wrreg_M}=
				{    1'b1,{1'b0,op2_t_D},       1'b1,       1'b0,       1'b0,   1'b1};
	   OP1_CMPR:
				{aluimm_D,alufunc_A     ,selaluout_D,selmemout_D,selpcplus_D,wrreg_M}=
				{    1'b0,{1'b1,op2_i_D},       1'b1,       1'b0,       1'b0,   1'b1};
		// TODO: Write the rest of the decoding code
		default:  ;
	 endcase
  end
endmodule
