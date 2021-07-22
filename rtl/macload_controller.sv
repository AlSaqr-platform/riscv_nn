import riscv_defines::*;
`include "riscv_config.sv"

module macload_controller(
	input logic			clk_i,
	input logic			rstn_i,
	input logic			update_a_i,  //from ID
	input logic 		update_w_i,  //from ID
	input logic			ex_valid_i,	 //from EX
	input logic 		csr_a_rstn_i,//from CSR
	input logic 		csr_w_rstn_i,//from CSR
	input logic [31:0]	a_address_i, //from CSR
	input logic [31:0]	w_address_i, //from CSR
	input logic [31:0]	a_stride_i,	 //from CSR
	input logic [31:0]	w_stride_i,	 //from CSR
	input logic [31:0]  a_rollback_i,//from CSR
	input logic [31:0]	w_rollback_i,//from CSR
	input logic [31:0]	a_skip_i,	 //from CSR
	input logic [31:0]	w_skip_i,	 //from CSR
	output logic [31:0]	updated_address_o,//to CSR
	output logic [1:0]	csr_op_o,		  //to CSR
	output logic [11:0] csr_address_o	  //to CSR
	);

logic update_a_int, update_w_int;
logic [31:0] a_count_n, a_count_q, w_count_n, w_count_q;

assign update_a_int = update_a_i & ex_valid_i;
assign update_w_int = update_w_i & ex_valid_i;

//Activations updates counter
assign a_count_n = a_count_q + 32'd1;

always_ff @(posedge clk_i, negedge rstn_i) begin
	if(~rstn_i) //resetting if rstn_i == 0 or the address has been updated by the C code in the CSR
		a_count_q <= '0;
	else begin
		if(~csr_a_rstn_i)
			a_count_q <= '0;
		else if((a_count_q == a_skip_i) & update_a_int) // resetting if we reached the number of updates before rollback
			a_count_q <= '0;
		else if(update_a_int)
			a_count_q <= a_count_n;
		else
			a_count_q <= a_count_q;
	end
end

//Weights updates counter
assign w_count_n = w_count_q + 32'd1;

always_ff @(posedge clk_i, negedge rstn_i) begin
	if(~rstn_i) //resetting if rstn_i == 0 or the address has been updated by the C code in the CSR
		w_count_q <= '0;
	else begin
		if(~csr_w_rstn_i)
			w_count_q <= '0;
		else if((w_count_q == w_skip_i) & update_w_int) // resetting if we reached the number of updates before rollback
			w_count_q <= '0;
		else if(update_w_int)
			w_count_q <= w_count_n;
		else
			w_count_q <= w_count_q;
	end
end

//ADDRESS UPDATE LOGIC: the address is incremented by its corresponding stride (s=CH_IN*DIM_KER*DIM_KER for example)
//until is_rollback becomes active, at this point the increment_n corresponds to a rollback operation and it will be
// -3*s + 4 (assuming we're addressing bytes). Both the stride and the rollback values are stored in the CSR.
//CONTROL SIGNALS GENERATION: csr_op_o is set to CSR_OP_WRITE each time there is a valid and active update signal,
// csr_address_o is set each time we have an update according to the received update signal
always_comb begin
	updated_address_o = '0;
	csr_address_o = '0;
	csr_op_o = CSR_OP_NONE;
	if(update_a_int && update_w_int) begin
		updated_address_o = '0;
		csr_address_o = '0;
		csr_op_o = CSR_OP_NONE;
	end else if(update_a_int) begin
		csr_address_o = CSR_A_ADDR;
		csr_op_o = CSR_OP_WRITE;
		if(a_count_q < a_skip_i)
			updated_address_o = a_address_i + a_stride_i;
		else
			updated_address_o = a_address_i + a_rollback_i;
	end else if(update_w_int) begin
		csr_address_o = CSR_W_ADDR;
		csr_op_o = CSR_OP_WRITE;
		if(w_count_q < w_skip_i)
			updated_address_o = w_address_i + w_stride_i;
		else
			updated_address_o = w_address_i + w_rollback_i;
	end else begin
		updated_address_o = '0;
		csr_address_o = '0;
		csr_op_o = CSR_OP_NONE;
	end
end

endmodule : macload_controller