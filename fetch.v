// inst_mem.v
// Simple 32-bit word-addressed instruction memory

module inst_mem (
    input  wire [31:0] pc, 
	input  rst,clk,flush,read_en,     // byte address
    output  [31:0] instruction
);
    reg [31:0] mem [0:255];

    initial begin
        $readmemh("instructions.hex", mem);  // optional
    end


/*always @(posedge clk) 
begin

        if (rst) 

            instruction <= 32'h00000000; // Reset instruction to NOP

        
	 else if (flush) 

            instruction <= 32'h00000000; // Flush instruction to NOP

         
	else if (read_en) 

            instruction <= mem[pc[11:2]]; // Fetch instruction based on PC
      

    end*/

assign instruction = mem[pc[11:2]];

endmodule




// pc_reg.v
module pc_reg (
    input  wire clk,
    input  wire rst,
    input  wire pc_en,
    input  wire [31:0] next_pc,
    output reg  [31:0] pc
);
    always @(posedge clk or posedge rst) begin
        if (rst)
            pc <= 32'h0000_0000;
        else if (pc_en)
            pc <= next_pc;
    end
endmodule



module if_stage_simple_btb (
    input  wire clk,
    input  wire rst,

    // Hazard stall from hazard unit
    input  wire pc_en,
    input  wire flush,

    // Signals from EX (branch/jump resolution)
    input  wire        modify_pc_ex,
    input  wire [31:0] update_pc_ex,
    input  wire [31:0] pc_ex,
    input  wire [31:0] jump_addr_ex,
    input  wire        update_btb_ex,
    input  wire        ex_branch_taken,

    // Outputs to IF/ID
    output wire [31:0] pc_if,
    output wire [31:0] instr_if,
    output wire        predictedTaken_if,
    output wire [31:0] predictedTarget_if
);

    // ------------------------------------------------------
    // PC Register using pc_reg module
    // ------------------------------------------------------
    wire [31:0] pc_current;
    reg  [31:0] pc_next;

    pc_reg u_pc_reg (
        .clk(clk),
        .rst(rst),
        .pc_en(pc_en),
        .next_pc(pc_next),
        .pc(pc_current)
    );

    assign pc_if = pc_current;

    // ------------------------------------------------------
    // BTB Prediction Lookup
    // ------------------------------------------------------
    wire        btb_valid;
    wire        btb_taken;
    wire [31:0] btb_target;

    btb u_btb (
        .clk(clk),
        .rst(rst),

        // FETCH
        .pc(pc_current),
        .predict_valid(btb_valid),
        .predict_taken(btb_taken),
        .predict_target(btb_target),

        // UPDATE (from EX)
        .update_en(update_btb_ex),
        .update_pc(pc_ex),
        .actual_taken(ex_branch_taken),
        .update_target(jump_addr_ex)
    );

    assign predictedTaken_if  = btb_valid && btb_taken;
    assign predictedTarget_if = predictedTaken_if ? btb_target
                                                  : (pc_current + 32'd4);

    // ------------------------------------------------------
    // NEXT PC selection
    // Priority:
    // 1) modify_pc_ex (redirect)
    // 2) BTB prediction
    // 3) default sequential PC + 4
    // ------------------------------------------------------
    always @(*) begin
        if (modify_pc_ex)
            pc_next = update_pc_ex;
        else if (btb_valid && btb_taken)
            pc_next = btb_target;
        else
            pc_next = pc_current + 32'd4;
    end

    // ------------------------------------------------------
    // INSTRUCTION MEMORY
    // ------------------------------------------------------
    inst_mem u_imem (
        .pc(pc_current),
        .rst(rst),
        .clk(clk),
        .flush(flush),
        .read_en(pc_en),
        .instruction(instr_if)
    );

endmodule



