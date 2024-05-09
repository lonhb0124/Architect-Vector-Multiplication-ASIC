
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/18/2024 12:21:12 PM
// Design Name: 
// Module Name: Asic_240316_0850
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


//---------------------------------------------------------
//  File:   Asic.v
//  Author: Abel Beyene
//  Date:   March 6, 2023
//
//  Description:
//
//  Top-level module for matrix-vector multiplier
//  
//  Interface:  
//  
//  Name                    I/O     Width     Description
//  -------------------------------------------------------
//  clk                     input   1         clock
//  rst                     input   1         reset
//  cmd_ready_o             output  1         control
//  cmd_valid_i             input   1         control
//  cmd_inst_funct_i        input   7         function code
//  cmd_inst_rs2_i          input   5         rs2 register
//  cmd_inst_rs1_i          input   5         rs1 register
//  cmd_inst_xd_i           input   1         valid rd
//  cmd_inst_xrs1_i         input   1         valid rs2
//  cmd_inst_xrs2_i         input   1         valid rs1
//  cmd_inst_rd_i           input   5         rd register
//  cmd_inst_opcode_i       input   7         opcode
//  cmd_rs1_i               input   64        rs1 data
//  mem_req_ready_o         input   128       control
//  mem_req_valid_i         input   1         control
//  mem_req_addr_i          output  32        memory address
//  mem_req_cmd_i           output  5         memory operation
//  mem_req_typ_i           output  3         operation size
//  mem_req_data_i          output  64        operation data
//  mem_resp_ready_i        input   128       control
//  mem_resp_valid_o        output  1         control
//  mem_resp_addr_o         output  32        memory address 
//  mem_resp_cmd_o          output  5         memory operation
//  mem_resp_typ_o          output  3         operation size
//  mem_resp_data_o         output  64        operation data
//---------------------------------------------------------

`include "AsicDefines.vh"

module Asic
(
  input         logic             clk,
  input         logic             reset,

  // PROC CMD Interface
                                
  output	logic             cmd_ready_o,
  input		logic       	  cmd_valid_i,
  input		logic [6:0]       cmd_inst_funct_i,
  input		logic [4:0]       cmd_inst_rs2_i,
  input		logic [4:0]       cmd_inst_rs1_i,
  input		logic        	  cmd_inst_xd_i,
  input		logic        	  cmd_inst_xs1_i,
  input		logic        	  cmd_inst_xs2_i,
  input		logic [4:0]       cmd_inst_rd_i,
  input		logic [6:0]       cmd_inst_opcode_i,
  input		logic [`XLEN-1:0] cmd_rs1_i,

  // PROC RESP Interface

  input		logic        	  resp_ready_i,
  output	logic        	  resp_valid_o,
  output	logic [4:0]       resp_rd_o,
  output	logic [`XLEN-1:0] resp_data_o,

  // MEM REQ Interface

  input  	logic       	  mem_req_ready_i,
  output 	logic       	  mem_req_valid_o,
  output 	logic [39:0]      mem_req_addr_o,
  output 	logic [4:0]	  mem_req_cmd_o,
  output 	logic [2:0]       mem_req_typ_o,
  output 	logic [`XLEN-1:0] mem_req_data_o,

  // MEM RESP Interface

  input 	logic        	  mem_resp_valid_i,
  input 	logic [39:0]      mem_resp_addr_i,
  input 	logic [4:0]       mem_resp_cmd_i,
  input 	logic [2:0]       mem_resp_typ_i,
  input 	logic [`XLEN-1:0] mem_resp_data_i
);

  // ************ PUT YOUR RTL HERE ************ //

  // registers hold addresses for calculation variables
  logic [15:0] a;
  logic [15:0] k;
  logic [3:0] k_prime;
  logic [15:0] M;
  logic [15:0] N;
  logic [31:0] addrwk;
  logic [31:0] addrxk;
  logic [31:0] addrrk;
  logic [31:0] addrwk_next;
  logic [31:0] addrxk_next;
  logic [31:0] addrrk_next;
  logic [5:0] M_reg;
  logic [5:0] M_reg_next;
  logic [5:0] N_reg;
  logic [5:0] N_reg_next;
  logic [3:0] K_reg_load;
  logic [3:0] K_reg_load_next;
  logic [3:0] K_reg_store;
  logic [3:0] K_reg_store_next;
  logic [7:0] w_data_reg [63:0];
  logic [7:0] x_data_reg [63:0];
  logic [31:0] addrr_N;
  logic [31:0] addrr;
  logic [1:0] adress_wxr;
  logic output_type;
  logic cmd_end;
  logic calc_begin; // init acquired all addresses, state transition flags
  logic calc_en;
  logic calc_done;
  logic load_en;
  logic write_en;
  logic everything_done;
  logic cmd_start;
  logic cmd_begin;
  logic load_begin;
  logic everything_end;
  logic load_write;

  // from L12 p17,            *** state description
  localparam STATE_IDLE = 3'd0;
  localparam STATE_CMD= 3'd1;
  localparam STATE_MEMORY = 3'd2;
  localparam STATE_CALC = 3'd3;
  localparam STATE_DONE = 3'd4;
  localparam MEM_IDLE = 4'd0;
  localparam MEM_INIT_X = 4'd1;
  localparam MEM_LOAD_VALID_X = 4'd2;
  localparam MEM_INIT_W= 4'd3;
  localparam MEM_LOAD_VALID_W = 4'd4;
  localparam MEM_NEXT_N = 4'd5;
  localparam MEM_INCREMENT_N = 4'd6;
  localparam MEM_STORE = 4'd7;
  localparam MEM_STORE_VALID = 4'd8;
  localparam MEM_NEXT_M = 4'd9;
  localparam MEM_INCREMENT_M = 4'd10;
  
  int stall_count;

  
  logic [2:0] state_reg;
  logic [2:0] state_next;
  logic [3:0] memory_state_reg;
  logic [3:0] memory_state_next;
  
  always @(posedge clk) begin
    if (!reset) begin
      state_reg <= STATE_IDLE;
      memory_state_reg <= MEM_IDLE;
      $display("ASIC reset has happened, state: %d", state_reg);
     
    end
    else begin
      state_reg <= state_next;
      memory_state_reg <= memory_state_next;
      //stall_count++;
    end
  end

  assign cmd_begin = cmd_valid_i && cmd_ready_o;
  assign load_begin = mem_req_ready_i && mem_req_valid_o;
  assign everything_end = resp_ready_i && resp_valid_o;

  cmd cmd00(clk, reset, cmd_valid_i, cmd_start, cmd_rs1_i, cmd_inst_funct_i, cmd_inst_opcode_i, cmd_ready_o);
  mreq mreq00(mem_req_ready_i, load_en, write_en, load_write, adress_wxr, addrwk, addrxk, addrrk, mem_req_valid_o, mem_req_addr_o, mem_req_cmd_o, mem_req_typ_o);
  calc cal00(mem_resp_data_i, cmd_inst_opcode_i, w_data_reg, x_data_reg, K_reg_store, N_reg, calc_en, output_type, a, calc_done, mem_req_data_o);
  resp resp00(resp_ready_i, everything_done, resp_valid_o, resp_rd_o, resp_data_o);
  
  always @(*) begin
    state_next = state_reg;
    memory_state_next = memory_state_reg;
    case (state_reg)
        STATE_IDLE: if (cmd_begin) begin state_next = STATE_CMD; end // 0-0
        STATE_CMD: if (cmd_end) begin state_next = STATE_MEMORY; end // 1-0
        STATE_MEMORY: begin 
            case (memory_state_reg)
                MEM_IDLE: begin memory_state_next = MEM_INIT_X; end // 2-0
                MEM_INIT_X: if (load_begin) begin memory_state_next = MEM_LOAD_VALID_X; end // 2-1
                MEM_LOAD_VALID_X:  if (mem_resp_valid_i) begin memory_state_next = MEM_INIT_W; end // 2-2
                MEM_INIT_W: if (load_begin) begin memory_state_next = MEM_LOAD_VALID_W; end // 2-3
                MEM_LOAD_VALID_W: if (mem_resp_valid_i) begin memory_state_next = MEM_NEXT_N; end // 2-4
                MEM_NEXT_N: if (N_reg < N - 1) begin memory_state_next = MEM_INCREMENT_N; end  // 2-5
                            else begin memory_state_next = MEM_STORE; state_next = STATE_CALC; end
                MEM_INCREMENT_N: if (M_reg == 0) begin memory_state_next = MEM_INIT_X; end // 2-6
                                 else begin memory_state_next = MEM_INIT_W; end            
                MEM_STORE: if (load_begin) begin memory_state_next = MEM_STORE_VALID; end // 2-7
                MEM_STORE_VALID: if (mem_resp_valid_i) begin memory_state_next = MEM_NEXT_M; end // 2-8
                MEM_NEXT_M: if (M_reg < M - 1) begin memory_state_next = MEM_INCREMENT_M; end // 2-9
                            else begin memory_state_next = MEM_IDLE; state_next = STATE_DONE; end
                MEM_INCREMENT_M: begin memory_state_next = MEM_INIT_W; end // 2-10
            endcase
        end
        STATE_CALC: if (calc_done) begin state_next = STATE_MEMORY; end // 3-7
        STATE_DONE:  if (everything_end) begin state_next = STATE_IDLE; end // 4-0
    endcase
  end


  always @(*) begin
    case (state_reg)
      // state 0
      STATE_IDLE: begin
                  a = 0;
                  k = 0;
                  M = 0;
                  N = 0;
                  addrwk = 0;
                  addrxk = 0;
                  addrrk = 0;
                  calc_begin = 0;
                  //calc_done <= 0;
                  everything_done = 0;
                  cmd_end = 0;
                  load_write = 0;
                  adress_wxr = 0;
                  M_reg = 0;
                  load_en = 0;
                  write_en = 0;
                  cmd_start = 1;
                  K_reg_load = 1;
                  K_reg_load_next = 1;
                  K_reg_store = 1;
                  K_reg_store_next = 1;    
              
                  for (int i = 0; i < 64; i++) begin
                      x_data_reg[i] = 0;
                      w_data_reg[i] = 0;
                  end
                  end
     // state 1
      STATE_CMD:  if (cmd_end) begin
                    //state_next <= STATE_MEMORY;
                    //calc_done <= 0;
                    cmd_start = 0;
                    case (k[2:0])
                        3'b000: k_prime = 4'b1000;
                        3'b001: k_prime = 4'b0001;
                        3'b010: k_prime = 4'b0010;
                        3'b011: k_prime = 4'b0011;
                        3'b100: k_prime = 4'b0100;
                        3'b101: k_prime = 4'b0101;
                        3'b110: k_prime = 4'b0110;
                        3'b111: k_prime = 4'b0111;
                    endcase
                  end
                  else begin
                      case (cmd_inst_funct_i)
                        7'b0000001: begin
                            a = cmd_rs1_i[15:0];
                            k = cmd_rs1_i[31:16];
                            case (cmd_inst_opcode_i)
                            7'b0000000: begin output_type = 0; end // r = y' 8-b wide
                            7'b0000010: begin output_type = 1; end // r = z 8-b wide
                            7'b0000100: begin end // r = \A5\F5 8-b wide
                            endcase
                        end
                        7'b0000010: begin
                            M = cmd_rs1_i[15:0];
                            N = cmd_rs1_i[31:16];
                        end    
                        7'b0000100: begin
                            addrwk = cmd_rs1_i[31:0]; // first element in memory
                        end 
                        7'b0000110: begin
                            addrxk = cmd_rs1_i[31:0];
                        end
                        7'b0001000: begin
                            addrrk = cmd_rs1_i[31:0];
                            cmd_end = 1;
                        end       
                        default: begin 
                            //state_next <= STATE_IDLE;
                            a = 0;
                            k = 0;
                            M = 0;
                            N = 0;
                            addrwk = 0;
                            addrxk = 0;
                            addrrk = 0;
                        end
                      endcase
                  end
                  
      // state 2            
      STATE_MEMORY: begin 
                            case (memory_state_reg) // if stall_count exceed something it goes to MEM_IDLE
                            // state 2-0 
                            MEM_IDLE: begin 
                                //memory_state_next <= MEM_INIT_X;
                                load_write = 1;
                                addrr_N = addrwk;
                                addrwk_next = addrwk;
                                addrxk_next = addrxk;
                                addrrk_next = addrrk;
                                N_reg = 0;
                                N_reg_next = 0;
                                //calc_done <= 0;
                            end
                            
                            // state 2-1 
                            MEM_INIT_X: begin 
                                load_en = 1;
                                //if (load_begin) begin // checking mem_req_ready_i && mem_req_valid_o
                                //    memory_state_next <= MEM_LOAD_VALID_X;
                               // end
                            end
                            
                            // state 2-2 
                            MEM_LOAD_VALID_X: begin // checking mem_resp_valid_i and save actual data
                                load_en = 0;
                                if (mem_resp_valid_i) begin
                                    case (K_reg_load)
                                        1: x_data_reg[N_reg][7:0] = mem_resp_data_i[7:0];
                                        2: x_data_reg[N_reg][7:0] = mem_resp_data_i[15:8];
                                        3: x_data_reg[N_reg][7:0] = mem_resp_data_i[23:16];
                                        4: x_data_reg[N_reg][7:0] = mem_resp_data_i[31:24];
                                        5: x_data_reg[N_reg][7:0] = mem_resp_data_i[39:32];
                                        6: x_data_reg[N_reg][7:0] = mem_resp_data_i[47:40];
                                        7: x_data_reg[N_reg][7:0] = mem_resp_data_i[55:48];
                                        8: x_data_reg[N_reg][7:0] = mem_resp_data_i[63:56];
                                    endcase
                                    //$display("ASIC non-reset x_data_reg[N_reg][7:0]: %b, %h", x_data_reg[N_reg][7:0], x_data_reg[N_reg][7:0]);
                                    //$display("ASIC non-reset N_reg: %b, K_reg_load: %b", N_reg, K_reg_load);
                                    //$display("ASIC non-reset addrxk: %b, %h", addrxk, addrxk);
                                    adress_wxr =  2'b01; // read w address
                                    //memory_state_next <= MEM_INIT_W;
                                end
                            end
                            
                            // state 2-3 
                            MEM_INIT_W: begin
                                //addrxk <= addrxk + N_reg;
                                load_en = 1;
                                //if (load_begin) begin // checking mem_req_ready_i && mem_req_valid_o
                                    //memory_state_next <= MEM_LOAD_VALID_W;
                                //end
                            end
                            
                            // state 2-4 
                            MEM_LOAD_VALID_W: begin
                                load_en = 0;
                                if (mem_resp_valid_i) begin
                                    case (K_reg_load)
                                        1: w_data_reg[N_reg][7:0] = mem_resp_data_i[7:0];
                                        2: w_data_reg[N_reg][7:0] = mem_resp_data_i[15:8];
                                        3: w_data_reg[N_reg][7:0] = mem_resp_data_i[23:16];
                                        4: w_data_reg[N_reg][7:0] = mem_resp_data_i[31:24];
                                        5: w_data_reg[N_reg][7:0] = mem_resp_data_i[39:32];
                                        6: w_data_reg[N_reg][7:0] = mem_resp_data_i[47:40];
                                        7: w_data_reg[N_reg][7:0] = mem_resp_data_i[55:48];
                                        8: w_data_reg[N_reg][7:0] = mem_resp_data_i[63:56];
                                    endcase
                                    //$display("ASIC non-reset w_data_reg[N_reg][7:0]: %b, %h", w_data_reg[N_reg][7:0], w_data_reg[N_reg][7:0]);
                                    //$display("ASIC non-reset N_reg: %b, K_reg_load: %b", N_reg, K_reg_load);
                                    //$display("ASIC non-reset addrwk: %b, %h", addrwk, addrwk);
                                    //w_data_reg[N_reg][7:0] <= mem_resp_data_i[7:0];
                                    //adress_wxr <=  2'b00; // read x address
                                    //memory_state_next <= MEM_NEXT_N;
                                end
                            end
                            
                            // state 2-5
                            MEM_NEXT_N: begin
                                    if (N_reg < N - 1) begin
                                        //memory_state_next <= MEM_INCREMENT_N;
                                        if (addrwk_next == addrwk) begin
                                            N_reg_next = N_reg + 1;
                                            if (K_reg_load < k_prime) begin
                                                K_reg_load_next = K_reg_load + 1;
                                                if (M_reg == 0) begin
                                            	    //addrxk_next <= addrxk + 4'b1000;
                                                    adress_wxr =  2'b00; // read w address
                                                end
                                            end
                                            else begin
                                                addrwk_next = addrwk + 4'b1000;
                                                K_reg_load_next = 1;
                                                if (M_reg == 0) begin
                                            	    addrxk_next = addrxk + 4'b1000;
                                                    adress_wxr =  2'b00; // read w address
                                                end
                                            end
                                        end

                                    end
                                    else begin
                                        K_reg_load_next = 1;
                                        //memory_state_next <= MEM_STORE;
                                        //state_next <= STATE_CALC;
                                        adress_wxr =  2'b10;
                                        load_write = 0;
                                        //i <= 0;
                                        calc_en = 1;
                                    end
                            end

                            // state 2-6
                            MEM_INCREMENT_N: begin
                                addrwk = addrwk_next;
                                addrxk = addrxk_next;
                                K_reg_load = K_reg_load_next;
                                N_reg = N_reg_next;
                                //if (M_reg == 0) begin
                                //   memory_state_next <= MEM_INIT_X;
                                //end 
                                //else begin
                                //    memory_state_next <= MEM_INIT_W;
                                //end
                            end

                            // state 2-7
                            MEM_STORE: begin
                                write_en = 1;
                                //if (load_begin) begin // checking mem_req_ready_i && mem_req_valid_o
                                //    memory_state_next <= MEM_STORE_VALID;
                               // end
                            end
                            
                            // state 2-8
                            MEM_STORE_VALID: begin
                                write_en = 0;
                                //calc_en <= 0;
                                if (mem_resp_valid_i) begin
                                    //memory_state_next <= MEM_NEXT_M;
                                    calc_en = 0;
                                    //everything_done <= 1;
                                    //$display("ASIC non-reset addrrk: %b, %h", addrrk, addrrk);
                                    //$display("ASIC non-reset K_reg_store: %b", K_reg_store);
                                end
                            end
                            
                            // state 2-9
                            MEM_NEXT_M: begin
                                //calc_en <= 0;
                                load_write = 1;
                                if (M_reg < M - 1) begin
                                    //adress_wxr <=  2'b01;
                                    //memory_state_next <= MEM_INCREMENT_M;
                                    if (addrwk_next == addrwk) begin
                                        M_reg_next = M_reg + 1;
                                        if (K_reg_load < k_prime) begin
                                            K_reg_load_next = K_reg_load + 1;
                                            //adress_wxr <=  2'b00; // read w address
                                            if (K_reg_store < k_prime) begin
                                                K_reg_store_next = K_reg_store + 1;
                                            end
                                            else begin
                                                K_reg_store_next = 1;
                                                //addrwk_next <= addrwk + 4'b1000;
                                                addrrk_next = addrrk + 4'b1000;
                                            end
                                        end
                                        else begin
                                            K_reg_load_next = 1;
                                            addrwk_next = addrwk + 4'b1000;
                                            if (K_reg_store < k_prime) begin
                                                K_reg_store_next = K_reg_store + 1;
                                            end
                                            else begin
                                                K_reg_store_next = 1;
                                                //addrwk_next <= addrwk + 4'b1000;
                                                addrrk_next = addrrk + 4'b1000;
                                            end
                                        end
                                    end

                                end
                                else begin
                                    K_reg_store_next = 1; 
                                    K_reg_load_next = 1;
                                    //memory_state_next <= MEM_IDLE;
                                    //state_next <= STATE_DONE;
                                    everything_done = 1;
                                end
                            end
                            
                            // state 2-10
                            MEM_INCREMENT_M: begin
                                adress_wxr =  2'b01;
                                K_reg_load = K_reg_load_next;
                                K_reg_store = K_reg_store_next;
                                addrwk = addrwk_next;
                                addrrk = addrrk_next;
                                M_reg = M_reg_next;
                                //memory_state_next <= MEM_INIT_W;
                            end


                          endcase
                      end
                         
      // state 3                      
      STATE_CALC: if (calc_done) begin 
                      //state_next <= STATE_MEMORY;
                      N_reg = 0;  
                  end

      // state 4              
      STATE_DONE: if (everything_end) begin 
                    //state_next <= STATE_IDLE;
                    //cmd_start <= 1;
//                  cmd_ready_o = 1;
                    a = 0;
                    k = 0;
                    M = 0;
                    N = 0;
                    addrwk = 0;
                    addrxk = 0;
                    addrrk = 0;
                    calc_begin = 0;
                    //calc_done <= 0;
                    everything_done = 0;
                    cmd_end = 0;
                    load_write = 0;
                    adress_wxr = 0;
                    M_reg = 0;
                    load_en = 0;
                    write_en = 0;
                    cmd_start = 1;
                    K_reg_load = 1;
                    K_reg_load_next = 1;
                    K_reg_store = 1;
                    K_reg_store_next = 1;    
  
                    for (int i = 0; i < 64; i++) begin
                       x_data_reg[i] = 0;
                       w_data_reg[i] = 0;
                    end
                end
    endcase
  end

endmodule


module cmd(
    input logic clk, reset, cmd_valid_i, cmd_start,
    input logic [63:0] cmd_rs1_i, 
    input logic [6:0] cmd_inst_funct_i,
    input logic [6:0] cmd_inst_opcode_i, 
    output logic cmd_ready_o);
//  input 
//  output
// assign cmd_ready_o = calc_done;  
assign cmd_ready_o = cmd_start;
  
endmodule

module resp (input logic resp_ready_i, 
   input logic everything_done,
   output logic resp_valid_o,
   output logic [4:0] resp_rd_o,
   output logic [63:0] resp_data_o);

  
assign resp_valid_o = everything_done;
assign resp_rd_o = everything_done;
assign resp_data_o = everything_done;

 
endmodule

module mreq (input logic mem_req_ready_i, load_en, write_en, load_write,
    input logic [1:0] adress_wxr,
    input logic [31:0] addrwk,
    input logic [31:0] addrxk,
    input logic [31:0] addrrk,
    output logic mem_req_valid_o, 
    output logic [39:0] mem_req_addr_o,
    output logic [4:0] mem_req_cmd_o,
    output logic [2:0] mem_req_typ_o
    //output logic [63:0] mem_req_data_o
    );


assign mem_req_valid_o = load_en ^ write_en;
assign mem_req_typ_o = 3'b000;

always_comb
begin
    if (adress_wxr == 2'b00) begin
        mem_req_addr_o = {8'b0, addrxk};
    end
    else if (adress_wxr == 2'b01) begin
        mem_req_addr_o = {8'b0, addrwk};
    end
    else if (adress_wxr == 2'b10) begin
        mem_req_addr_o = {8'b0, addrrk};
    end
    
    if (load_write) begin
      mem_req_cmd_o = 5'b00000;
    end
    else begin
      mem_req_cmd_o = 5'b00001;
    end
end

endmodule

module calc (input logic [63:0] mem_resp_data_i, 
    input logic [6:0] cmd_inst_opcode_i, 
    input logic signed [7:0] w_data_reg [63:0],
    input logic signed [7:0] x_data_reg [63:0],
    input logic [3:0] K_reg_store,
    input logic [5:0] N_reg,
    input logic calc_en,
    input output_type,
    input logic [15:0] a, 
    output logic calc_done,
    //output logic signed [31:0] intermediate_results [63:0],
    output logic [63:0] mem_req_data_o);
//  input

//logic [15:0] intermediate_results;
//int count 0; 
logic signed [31:0] intermediate_results [63:0];
logic signed [7:0] temp_result;
logic [63:0] temp_memory;
logic [31:0] result;

always_comb
begin
    if (calc_en) begin
        for (int j = 0; j < 64; j++) begin
        intermediate_results[j] = x_data_reg[j] * w_data_reg[j];
        //mem_req_data_o <= mem_req_data_o + intermediate_results[j];
        //mem_req_data_o[7:0] += intermediate_results[j];
        result += intermediate_results[j];
        //mem_req_data_o += intermediate_results[j];
        //$display("ASIC non-reset result %b, %h", mem_req_data_o, mem_req_data_o);
        end 
        //$display("ASIC non-reset result %b, %h", mem_req_data_o, mem_req_data_o);
        case (a)
            0: temp_result[7:0] = result[7:0];
            1: temp_result[7:0] = result[8:1];
            2: temp_result[7:0] = result[9:2];
            3: temp_result[7:0] = result[10:3];
            4: temp_result[7:0] = result[11:4];
            5: temp_result[7:0] = result[12:5];
            6: temp_result[7:0] = result[13:6];
            7: temp_result[7:0] = result[14:7];
            8: temp_result[7:0] = result[15:8];
            9: temp_result[7:0] = result[16:9];
            10: temp_result[7:0] = result[17:10];
            11: temp_result[7:0] = result[18:11];
            12: temp_result[7:0] = result[19:12];
            13: temp_result[7:0] = result[20:13];
            14: temp_result[7:0] = result[21:14];
            15: temp_result[7:0] = result[22:15];
            16: temp_result[7:0] = result[23:16];
            17: temp_result[7:0] = result[24:17];
            18: temp_result[7:0] = result[25:18];
            19: temp_result[7:0] = result[26:19];
            20: temp_result[7:0] = result[27:20];
            21: temp_result[7:0] = result[28:21];
            22: temp_result[7:0] = result[29:22];
            23: temp_result[7:0] = result[30:23];
            24: temp_result[7:0] = result[31:24];
            //default: mem_req_data_o[7:0] = result[7:0];
        endcase
        if (output_type) begin
            if (temp_result[7] == 1) begin
                temp_result[7:0] = 0;
            end
        end
        case (K_reg_store)
            1: mem_req_data_o[7:0] = temp_result;
            2: mem_req_data_o[15:0] = {temp_result, temp_memory[7:0]};
            3: mem_req_data_o[23:0] = {temp_result, temp_memory[15:0]};
            4: mem_req_data_o[31:0] = {temp_result, temp_memory[23:0]};
            5: mem_req_data_o[39:0] = {temp_result, temp_memory[31:0]};
            6: mem_req_data_o[47:0] = {temp_result, temp_memory[39:0]};
            7: mem_req_data_o[55:0] = {temp_result, temp_memory[47:0]};
            8: mem_req_data_o[63:0] = {temp_result, temp_memory[55:0]};
        endcase
        temp_memory = mem_req_data_o;
        $display("ASIC non-reset result %b, %h", result, result);
        //$display("ASIC non-reset mem_req_data_o %b, %h", mem_req_data_o, mem_req_data_o);
        calc_done = 1;
    end 
    else begin
        calc_done = 0;
        mem_req_data_o = 0;
        result = 0;
        temp_result = 0;
        for (int k = 0; k < 64; k++) begin
            intermediate_results[k] = 0;
        end 
    end 
end
endmodule
