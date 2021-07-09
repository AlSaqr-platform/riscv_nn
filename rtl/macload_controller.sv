import riscv_defines::*;
`include "riscv_config.sv"

module macload_controller(
	input logic			clk_i,
	input logic			rstn_i,
	input logic			update_a_i,  //from ID
	input logic 		update_w_i,  //from ID
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

logic [1:0] update_cfg;
logic [31:0] sel_address_n, sel_address_q;
logic [31:0] increment_n, increment_q;
logic [1:0] csr_op_n, csr_op_q;
logic [11:0] csr_address_n, csr_address_q;
logic [31:0] a_count_n, a_count_q, w_count_n, w_count_q;
typedef enum {idle, update_a, update_w} state;
state cs, ns;

assign update_cfg = {update_a_i, update_w_i};

//Activations updates counter
assign a_count_n = a_count_q + 32'd1;

always_ff @(posedge clk_i) begin
	if(~rstn_i) //resetting if rstn_i == 0 or we reached the number of updates before rollback
		a_count_q <= '0;
	else if(a_count_q == a_skip_i)
		a_count_q <= '0;
	else if(update_a_i)
		a_count_q <= a_count_n;
	else
		a_count_q <= a_count_q;
end

//Weights updates counter
assign w_count_n = w_count_q + 32'd1;

always_ff @(posedge clk_i) begin
	if(~rstn_i) //resetting if rstn_i == 0 or we reached the number of updates before rollback
		w_count_q <= '0;
	else if(w_count_q == w_skip_i)
		w_count_q <= '0;
	else if(update_w_i)
		w_count_q <= w_count_n;
	else
		w_count_q <= w_count_q;
end

//Generation of is_rollback_a --> this signal forces the rollback of the activations buffer
assign is_rollback_a = (a_count_q == a_skip_i) ? 1'b1 : 1'b0;

//Generation of is_rollback_w --> this signal forces the rollback of the weights buffer
assign is_rollback_w = (w_count_q == w_skip_i) ? 1'b1 : 1'b0;

//increment_n generation: the address is incremented by its corresponding stride (s=CH_IN*DIM_KER*DIM_KER for example)
//until is_rollback becomes active, at this point the increment_n corresponds to a rollback operation and it will be
// -3*s + 4 (assuming we're addressing bytes). Both the stride and the rollback values are stored in the CSR.

always_ff @(posedge clk_i) begin
	if(~rstn_i) begin
		sel_address_q <= 0;
		increment_q <= '0;
		csr_op_q <= CSR_OP_NONE;
		csr_address_q <= '0;
	end else begin
		sel_address_q <= sel_address_n;
		increment_q <= increment_n;
		csr_op_q <= csr_op_n;
		csr_address_q <= csr_address_n;
	end
end

assign csr_op_o = csr_op_q;
assign csr_address_o = csr_address_q;

//State update
always_ff @(posedge clk_i) begin
	if(~rstn_i) begin
		cs <= idle;
	end else begin
		cs <= ns;
	end
end

//Combinational FSM
always_comb begin
	ns = idle;
	sel_address_n = '0;
	increment_n = '0;
	csr_op_n = CSR_OP_NONE;
	csr_address_n = '0;
	updated_address_o = '0;
	case (cs)
		idle: begin
			if(update_a_i && update_w_i) begin
				ns = idle;
				csr_op_n = CSR_OP_NONE;
				csr_address_n = '0;
				updated_address_o = '0;
			end else if(update_a_i) begin
				csr_op_n = CSR_OP_WRITE;
				csr_address_n = CSR_A_ADDR;
				if(a_count_q < a_skip_i) begin
					ns = update_a;
					updated_address_o = a_address_i + a_stride_i;
				end else begin
					ns = idle;
					updated_address_o = a_address_i + a_rollback_i;
				end
			end else if(update_w_i) begin 
				csr_op_n = CSR_OP_WRITE;
				csr_address_n = CSR_W_ADDR;
				if(w_count_q < w_skip_i) begin
					ns = update_w;
					updated_address_o = w_address_i + w_stride_i;
				end else begin
					ns = idle;
					updated_address_o = w_address_i + w_rollback_i;
				end
			end else begin
				ns = idle;
				csr_op_n = CSR_OP_NONE;
				csr_address_n = '0;
				updated_address_o = '0;
			end
		end
		update_a: begin 
			if(a_count_q < a_skip_i) begin
				updated_address_o = a_address_i + a_stride_i;
			end else begin
				updated_address_o = a_address_i + a_rollback_i;
			end
			if(update_a_i && update_w_i) begin
				ns = idle;
				csr_op_n = CSR_OP_NONE;
				csr_address_n = '0;
			end else if(update_a_i) begin
				ns = update_a;
				csr_op_n = CSR_OP_WRITE;
				csr_address_n = CSR_A_ADDR;
			end else if(update_w_i) begin 
				csr_op_n = CSR_OP_WRITE;
				csr_address_n = CSR_W_ADDR;
				if(w_count_q < w_skip_i) begin
					ns = update_w;
				end else begin
					ns = update_w;
				end
			end else begin
				ns = idle;
				csr_op_n = CSR_OP_NONE;
				csr_address_n = '0;
			end
		end
		update_w: begin
			if(w_count_q < w_skip_i) begin
				updated_address_o = w_address_i + w_stride_i;
			end else begin
				updated_address_o = w_address_i + w_rollback_i;
			end
			if(update_a_i && update_w_i) begin
				ns = idle;
				csr_op_n = CSR_OP_NONE;
				csr_address_n = '0;
				updated_address_o = '0;
			end else if(update_a_i) begin
				csr_op_n = CSR_OP_WRITE;
				csr_address_n = CSR_A_ADDR;
				if(a_count_q < a_skip_i) begin
					ns = update_a;
				end else begin
					ns = update_a;
				end
			end else if(update_w_i) begin 
				ns = update_w;
				csr_op_n = CSR_OP_WRITE;
				csr_address_n = CSR_W_ADDR;
			end else begin
				ns = idle;
				csr_op_n = CSR_OP_NONE;
				csr_address_n = '0;
			end
		end
		default: begin 
			ns = idle;
			csr_op_n = CSR_OP_NONE;
			csr_address_n = '0;
			updated_address_o = '0;
		end
	endcase
end

endmodule : macload_controller