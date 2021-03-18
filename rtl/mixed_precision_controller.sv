import riscv_defines::*;

module mixed_precision_controller
  (
   input logic                           clk,
   input logic                           rst_n,
   input logic                           illegal_insn_i,
   input logic                           is_decoding_i,
   input logic                           ex_ready_i,
   input                                 ivec_mode_fmt ivec_fmt_i,
   input logic [NBITS_MIXED_CYCLES-1:0]  current_cycle_i,
   input logic [31:0]                    instr_rdata_i,
   input logic [NBITS_MAX_KER-1:0]       skip_size_i, 
   output logic [NBITS_MIXED_CYCLES-1:0] next_cycle_o,
   output logic                          mux_sel_wcsr_o,
   output                                mux_sel_mpc mux_sel_mpc_o                       
   );
  
   mux_sel_mpc     last_ins;
   logic [NBITS_MAX_KER-1:0]             skip_counter_q, skip_counter_n;
                                
   
   always_comb begin   

      skip_counter_n = skip_counter_q;     
      mux_sel_wcsr_o = 0;    
      next_cycle_o = current_cycle_i; //Doesn't update unless the instruction is a dotp or a write to the csr of mixed precision cycle
      
      if ( (illegal_insn_i == 0) && (is_decoding_i == 1) && (ex_ready_i == 1)) begin //Check for illegal instruction
         //Check if the current instruction is a dotp/sdotp, if so add 1 to the cycle counter
         if ( (instr_rdata_i[6:0] == OPCODE_VECOP)  && (instr_rdata_i[31:26] inside {DOTUP, DOTUSP, DOTSP, SDOTUP, SDOTUSP, SDOTSP}) ) begin
            if ( (skip_counter_q+1) < skip_size_i ) begin
               skip_counter_n = skip_counter_q + 1;
            end
            else begin
               skip_counter_n = '0;               
               /***********************************************************************************************************************************
                * Mixed ops have differente number of reset:                                                                                      * 
                * MIXED_2x4, MIXED_4x8, MIXED_8x16 : They are all the twice the size compared to each other, so it takes two cycles to complete;  *
                * MIXED_2x8, MIXED_4x16 : They are 4 times each other so it takes 4 cycle to complete;                                            *
                * MIXED_2x16 : 16 is 8 times 2 so it will need 8 cycles to complete.                                                              *
                ***********************************************************************************************************************************/
               // TO CHECK : LEAVE >= OR JUST ==
               case( ivec_fmt_i )              
                 MIXED_2x4, MIXED_4x8, MIXED_8x16: begin
                    mux_sel_wcsr_o = 1'b1; //Needed to write the next_cycle to the csr                               
                    if( current_cycle_i >= 1 ) 
                      next_cycle_o = 0;                                                  
                    else
                      next_cycle_o = current_cycle_i + 1;                                
                 end

                 MIXED_2x8, MIXED_4x16: begin
                    mux_sel_wcsr_o = 1'b1; //Needed to write the next_cycle to the csr
                    if ( current_cycle_i >= 3 )
                      next_cycle_o = 0;
                    else
                      next_cycle_o = current_cycle_i + 1;
                 end

                 MIXED_2x16: begin
                    mux_sel_wcsr_o = 1'b1; //Needed to write the next_cycle to the csr
                    if ( current_cycle_i >= 7 )
                      next_cycle_o = 0;
                    else
                      next_cycle_o = current_cycle_i + 1;
                 end    

                 default ;            

               endcase // case ( ivec_fmt_i )
            end // else: !if( skip_counter < skip_size_i )            
         end // if ( (instr_rdata_i[6:0] == OPCODE_VECOP)  && (instr_rdata_i[31:26] inside {dotp_ins}) )         
      end // if ( illegal_insn_i == 0 )      
   end // always_comb

   always_ff @(posedge clk, negedge rst_n) begin
 
      if (rst_n == 1'b0) begin
         last_ins <= MPC_CSR;
         skip_counter_q <= '0;         
      end
      //Check if last instruction was a write to csr
      else if( (instr_rdata_i[6:0] == OPCODE_SYSTEM) && (instr_rdata_i[13:12] == 2'b01) && (instr_rdata_i[31:20] == 12'h00D) )
        last_ins <= MPC_CSR_WRITE;
      //Check if last instruction was a dotp
      else if ( (instr_rdata_i[6:0] == OPCODE_VECOP) && (instr_rdata_i[31:26] inside {DOTUP, DOTUSP, DOTSP, SDOTUP, SDOTUSP, SDOTSP} ) ) begin
         last_ins <= MPC_MIX_CNTRL;
         skip_counter_q <= skip_counter_n;
      end
      //Default uses csr value
      else
        last_ins <= MPC_CSR;     
   end // always_ff @
   
   assign mux_sel_mpc_o = last_ins;

endmodule // mixed_precision_controller


